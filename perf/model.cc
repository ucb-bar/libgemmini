#include "model.h"

#include <cstdlib>

namespace gperf {

namespace {
bool is_loop_cmd(unsigned f) { return (f >= 8 && f <= 13) || f == 24 || f == 25 || f == 31 || f == 32; }
double out_fmt_bytes(unsigned fmt) { return fmt == 3 ? 2.0 : fmt == 0 ? 1.0 : 0.5; }   // BF16 / FP8 / 4-bit codes
const char *kQueueName[Q_COUNT] = {"ld", "ex", "st", "vec"};
}  // namespace

model_t::model_t(const config_t &c)
    : cfg_(c), mem_(c, eq_), spad_(c, eq_), acc_(c, eq_), reader_(c, eq_, mem_, spad_), writer_(c, eq_, mem_),
      rs_(c, eq_), load_(c, eq_, reader_, spad_, rs_), exec_(c, eq_, spad_, acc_, rs_),
      store_(c, eq_, spad_, acc_, writer_, rs_), scales_(c, eq_, mem_), luts_(c, eq_, mem_),
      vpu_(c, eq_, spad_, rs_), sreq_(c, eq_, spad_, rs_), loops_(c), front_(c), host_(c),
      dim_((uint32_t)c.mesh_dim) {
  // anything that frees a resource the command stream may be waiting on wakes it
  rs_.on_room([this] { kick_at(eq_.now()); });
  rs_.on_done([this](int tag) { loops_.completed(tag); kick_at(eq_.now()); });
  scales_.on_room([this] { kick_at(eq_.now()); });
  if (const char *p = getenv("GEMMINI_PERF_TRACE")) {
    trace_ = fopen(p, "w");
    if (trace_) {
      fprintf(trace_, "what,queue,alloc,issue,done\n");
      rs_.on_trace([this](const rs_cmd_t &c, cycle_t a, cycle_t i, cycle_t d) {
        fprintf(trace_, "%s,%s,%lld,%lld,%lld\n", c.what, kQueueName[c.q], (long long)a, (long long)i, (long long)d);
      });
    }
  }
}

model_t::~model_t() {
  if (trace_) fclose(trace_);
}

void model_t::kick_at(cycle_t t) {
  if (t < eq_.now()) t = eq_.now();
  if (step_ev_ && step_at_ <= t) return;
  eq_.cancel(step_ev_);
  step_at_ = t;
  step_ev_ = eq_.at(t, [this] {
    step_ev_ = 0;
    step_at_ = NEVER;
    step();
  });
}

uint64_t model_t::rs_alloc(rs_cmd_t c) { return rs_.alloc(std::move(c)); }

void model_t::step() {
  const cycle_t now = eq_.now();
  bool progress = false;
  // input side: LoopMatmul takes a loop config / LOOP_WS from the front end when its slot is free
  if (front_.head_ready(now) && is_loop_cmd(front_.head().funct) && loops_.config_slot_free()) {
    const rocc_cmd_t c = front_.head();
    front_.pop();
    if (c.funct == 8) loops_.run(c.rs1, c.rs2, now, out_bytes_);
    else loops_.config(c.funct, c.rs1, c.rs2);
    progress = true;
  }
  // output side: one command per cycle
  if (last_out_ < now) {
    loop_cmd_t lc;
    cycle_t wake = NEVER;
    if (loops_.next(now, &lc, &wake)) {
      if (issue_loop(lc)) { progress = true; }
    } else if (loops_.idle() && front_.head_ready(now) && !is_loop_cmd(front_.head().funct)) {
      if (issue_raw(front_.head())) {
        front_.pop();
        last_out_ = now;
        progress = true;
      }
    }
    if (wake != NEVER) kick_at(wake);
  } else {
    kick_at(last_out_ + 1);
  }
  if (progress) kick_at(now + 1);
  else if (!front_.empty() && front_.head_arrival() > now) kick_at(front_.head_arrival());
}

bool model_t::issue_loop(const loop_cmd_t &c) {
  const cycle_t now = eq_.now();
  switch (c.kind) {
    case L_EX: {
      if (!rs_.has_room(Q_EX)) return false;
      rs_cmd_t pre;
      pre.q = Q_EX;
      pre.a = make_span(c.b, dim_, false);
      pre.b = make_span(c.c, dim_ / 4, true);
      pre.tag = LC_EX;
      pre.what = "loop_preload";
      pre.unit_has_room = [this] { return exec_.has_room(); };
      pre.start = [this, c](uint64_t id) {
        execute_unit_t::ex_cmd_t e;
        e.kind = execute_unit_t::ex_cmd_t::PRELOAD;
        e.b = c.b;
        e.c = c.c;
        exec_.accept(id, e);
      };
      rs_alloc(std::move(pre));
      loops_.emitted(c);
      // the compute leaves the next cycle (two commands from the Ex unroller)
      last_out_ = now + 1;
      const loop_cmd_t cc = c;
      eq_.at(now + 1, [this, cc] {
        rs_cmd_t comp;
        comp.q = Q_EX;
        comp.a = make_span(cc.a, cc.rows, false);
        comp.tag = LC_EX;
        comp.what = "loop_compute";
        comp.unit_has_room = [this] { return exec_.has_room(); };
        comp.start = [this, cc](uint64_t id) {
          execute_unit_t::ex_cmd_t e;
          e.kind = execute_unit_t::ex_cmd_t::COMPUTE;
          e.a = cc.a;
          e.rows = cc.rows;
          exec_.accept(id, e);
        };
        rs_alloc(std::move(comp));   // the ex queue may be full for an instant; RTL would stall a cycle
      });
      return true;
    }
    case L_LDA:
    case L_LDB: {
      if (!rs_.has_room(Q_LD)) return false;
      const load_unit_t::mvin_t m{c.local, c.dram, c.rows, c.cols, 0, c.stride};
      rs_cmd_t r;
      r.q = Q_LD;
      r.a = load_.dst_span(m);
      r.tag = LC_LD;
      r.what = c.kind == L_LDA ? "loop_lda" : "loop_ldb";
      r.unit_has_room = [this] { return load_.has_room(); };
      r.start = [this, m](uint64_t id) { load_.accept(id, m); };
      rs_alloc(std::move(r));
      break;
    }
    case L_STC:
    case L_STSPAD: {
      if (!rs_.has_room(Q_ST)) return false;
      store_unit_t::st_cmd_t s;
      s.kind = c.kind == L_STC ? store_unit_t::st_cmd_t::TO_DRAM : store_unit_t::st_cmd_t::TO_SPAD;
      s.src = c.local;
      s.rows = c.rows;
      s.cols = c.cols;
      s.dram = c.dram;
      s.stride = c.stride;
      s.dst = c.dst;
      s.out_bytes = c.out_bytes;
      rs_cmd_t r;
      r.q = Q_ST;
      r.a = store_.src_span(s);
      r.b = c.out_span;
      r.tag = LC_ST;
      r.what = c.kind == L_STC ? "loop_stc" : "loop_stspad";
      r.unit_has_room = [this] { return store_.has_room(); };
      r.start = [this, s](uint64_t id) { store_.accept(id, s); };
      rs_alloc(std::move(r));
      break;
    }
    default: return false;
  }
  loops_.emitted(c);
  last_out_ = now;
  return true;
}

bool model_t::issue_raw(const rocc_cmd_t &cmd) {
  const unsigned f = cmd.funct;
  const uint64_t rs1 = cmd.rs1, rs2 = cmd.rs2;
  if (luts_.busy()) return false;   // MX_LOAD_LUT blocks the stream while it runs
  auto rs_cmd = [&](queue_t q, span_t a, span_t b, bool config, const char *what, std::function<bool()> room,
                    std::function<void(uint64_t)> start) {
    if (!rs_.has_room(q)) return false;
    rs_cmd_t r;
    r.q = q;
    r.a = a;
    r.b = b;
    r.config = config;
    r.what = what;
    r.unit_has_room = std::move(room);
    r.start = std::move(start);
    rs_alloc(std::move(r));
    return true;
  };
  const span_t none;
  switch (f) {
    case 0: {   // CONFIG
      const unsigned type = rs1 & 3;
      if (type == 0) {
        if (!rs_.has_room(Q_EX)) return false;
        if (!((rs1 >> 7) & 1)) out_bytes_ = out_fmt_bytes((rs1 >> 14) & 3);
        return rs_cmd(Q_EX, none, none, true, "config_ex", [this] { return exec_.has_room(); }, [this](uint64_t id) {
          execute_unit_t::ex_cmd_t e;
          e.kind = execute_unit_t::ex_cmd_t::CONFIG;
          exec_.accept(id, e);
        });
      }
      if (type == 1)
        return rs_cmd(Q_LD, none, none, true, "config_ld", nullptr, [this, rs1, rs2](uint64_t) { load_.config(rs1, rs2); });
      return rs_cmd(Q_ST, none, none, true, "config_st", nullptr, [this, rs1, rs2](uint64_t) { store_.config(rs1, rs2); });
    }
    case 1: case 2: case 14: {   // MVIN2 / MVIN / MVIN3
      const int state = f == 2 ? 0 : f == 1 ? 1 : 2;
      const load_unit_t::mvin_t m{local_addr_t{(uint32_t)rs2}, rs1, (uint32_t)((rs2 >> 48) & 0xFFFF),
                                  (uint32_t)((rs2 >> 32) & 0xFFFF), state, 0};
      return rs_cmd(Q_LD, load_.dst_span(m), none, false, "mvin", [this] { return load_.has_room(); },
                    [this, m](uint64_t id) { load_.accept(id, m); });
    }
    case 3: {   // MVOUT
      store_unit_t::st_cmd_t s;
      s.kind = store_unit_t::st_cmd_t::TO_DRAM;
      s.src = local_addr_t{(uint32_t)rs2};
      s.rows = (rs2 >> 48) & 0xFFFF;
      s.cols = (rs2 >> 32) & 0xFFFF;
      s.dram = rs1;
      s.out_bytes = out_bytes_;
      return rs_cmd(Q_ST, store_.src_span(s), none, false, "mvout", [this] { return store_.has_room(); },
                    [this, s](uint64_t id) { store_.accept(id, s); });
    }
    case 6: {   // PRELOAD (raw): the ex unit pairs it with the next COMPUTE
      execute_unit_t::ex_cmd_t e;
      e.kind = execute_unit_t::ex_cmd_t::PRELOAD;
      e.b = local_addr_t{(uint32_t)rs1};
      e.c = local_addr_t{(uint32_t)rs2};
      return rs_cmd(Q_EX, make_span(e.b, dim_, false), make_span(e.c, dim_, true), false, "preload",
                    [this] { return exec_.has_room(); }, [this, e](uint64_t id) { exec_.accept(id, e); });
    }
    case 4: case 5: {   // COMPUTE
      execute_unit_t::ex_cmd_t e;
      e.kind = execute_unit_t::ex_cmd_t::COMPUTE;
      e.a = local_addr_t{(uint32_t)rs1};
      e.rows = (rs1 >> 48) & 0xFFFF;
      return rs_cmd(Q_EX, make_span(e.a, e.rows ? e.rows : dim_, false), none, false, "compute",
                    [this] { return exec_.has_room(); }, [this, e](uint64_t id) { exec_.accept(id, e); });
    }
    case 26: {  // CONFIG_SCALE_MEM: an ex-queue config that drains the mesh; rs2[16] also waits for the scales
      const bool wait = (rs2 >> 16) & 1;
      return rs_cmd(Q_EX, none, none, true, "config_scale_mem", [this] { return exec_.has_room(); },
                    [this, wait](uint64_t id) {
                      execute_unit_t::ex_cmd_t e;
                      e.kind = execute_unit_t::ex_cmd_t::CONFIG;
                      if (wait) e.wait_for = [this](done_t cb) { scales_.when_idle(cb); };
                      exec_.accept(id, e);
                    });
    }
    case 27: {  // MX_LOAD_SCALES: to the scale loader, not the RS
      if (!scales_.has_room()) return false;
      const uint64_t addr = rs1 & 0xFFFFFFFFFFULL, pitch = (rs1 >> 40) & 0xFFFFFF;
      const uint32_t len = (uint32_t)(rs2 & 0xFFFFFFFFu);
      uint32_t rows = (uint32_t)((rs2 >> 46) & 0xFF);
      if (rows == 0) rows = 1;
      scales_.submit(addr, len, rows, pitch ? pitch : len);
      return true;
    }
    case 29:    // MX_LOAD_LUT: one at a time; the stream waits until it is done
      luts_.load(rs1, (uint32_t)(rs2 & 0xFFFFFFFFu), (uint32_t)((rs2 >> 34) & 0x3F),
                 [this](cycle_t t) { kick_at(t); });
      return true;
    case 7: case 22: case 30: case 126:   // FLUSH, CLKGATE_EN, MX_LUT_DISABLE, COUNTER_OP: retire in the Controller
      return true;
    case 33: {  // VPU_EXEC
      const vpu_t::cmd_t v = vpu_t::decode(rs1, rs2);
      return rs_cmd(Q_VEC, vpu_t::read_span(v), vpu_t::write_span(v), false, "vpu",
                    [this] { return vpu_.has_room(); }, [this, v](uint64_t id) { vpu_.accept(id, v); });
    }
    case 34: {  // SPAD_REQUANT
      const spad_requant_t::cmd_t s = spad_requant_t::decode(rs1, rs2);
      return rs_cmd(Q_VEC, sreq_.read_span(s), sreq_.write_span(s), false, "spad_requant",
                    [this] { return sreq_.has_room(); }, [this, s](uint64_t id) { sreq_.accept(id, s); });
    }
    default:
      unmodelled_["funct " + std::to_string(f)]++;
      return true;
  }
}

void model_t::command(unsigned f, uint64_t rs1, uint64_t rs2, uint64_t instret) {
  cmds_++;
  const cycle_t offered = host_.now(instret);
  eq_.advance_to(offered);
  while (front_.full() && eq_.step()) {}   // the core stalls until the path has room
  const cycle_t t = cmax(offered, eq_.now());
  host_.wait_until(instret, t);
  eq_.advance_to(t);
  if (trace_) fprintf(trace_, "host_cmd%u,host,%lld,%lld,%lld\n", f, (long long)offered, (long long)t, (long long)t);
  kick_at(front_.push(f, rs1, rs2, t));
}

void model_t::fence(uint64_t instret) {
  fences_++;
  eq_.advance_to(host_.now(instret));
  const cycle_t offered = host_.now(instret);
  eq_.run_all();   // the core waits for Gemmini to go idle
  host_.wait_until(instret, eq_.last_activity() + (cycle_t)cfg_.host_fence_cycles);
  if (trace_ && host_.now(instret) > offered)
    fprintf(trace_, "host_fence,host,%lld,%lld,%lld\n", (long long)offered, (long long)offered,
            (long long)host_.now(instret));
}

void model_t::finish() { eq_.run_all(); }

void model_t::report(FILE *f) const {
  fprintf(f, "gemmini perf [%s]: %llu commands, %llu loops, %llu fences; last activity %lld; host stalled %lld "
             "cycles; %llu events\n",
          cfg_.preset.c_str(), (unsigned long long)cmds_, (unsigned long long)loops_.loops_run(),
          (unsigned long long)fences_, (long long)eq_.last_activity(), (long long)host_.stall_cycles(),
          (unsigned long long)eq_.events());
  fprintf(f, "  mesh: %llu tiles, %llu busy cycles; VPU: %llu commands\n", (unsigned long long)exec_.tiles(),
          (unsigned long long)exec_.busy_cycles(), (unsigned long long)vpu_.commands());
  fprintf(f, "  load: %llu bytes in %llu Gets; store: %llu reads, %llu Puts; scales: %llu bytes\n",
          (unsigned long long)reader_.gets() ? (unsigned long long)reader_.bytes() : 0ULL,
          (unsigned long long)reader_.gets(), (unsigned long long)store_.reads(), (unsigned long long)writer_.puts(),
          (unsigned long long)scales_.bytes());
  fprintf(f, "  memory: L2 %llu hits, %llu misses; bus busy %llu, DRAM busy %llu cycles\n",
          (unsigned long long)mem_.hits(), (unsigned long long)mem_.misses(), (unsigned long long)mem_.bus_busy(),
          (unsigned long long)mem_.dram_busy());
  fprintf(f, "  ports busy:");
  for (uint32_t b = 0; b < spad_.banks(); b++)
    fprintf(f, " sp%u r%llu/w%llu", b, (unsigned long long)spad_.read_bank(b).busy_cycles(),
            (unsigned long long)spad_.write_bank(b).busy_cycles());
  for (uint32_t b = 0; b < acc_.banks(); b++)
    fprintf(f, " acc%u %llu", b, (unsigned long long)acc_.bank(b).busy_cycles());
  fprintf(f, "\n");
  for (const auto &u : unmodelled_)
    fprintf(f, "  not modelled: %s x%llu\n", u.first.c_str(), (unsigned long long)u.second);
}

}  // namespace gperf
