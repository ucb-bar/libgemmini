#include "gemmini_perf.h"

#include <riscv/processor.h>
#include <riscv/rocc.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "perf/model.h"

// Command functs, mirroring the *_funct constants in gemmini.h (private members there, so not
// reachable from here). Keep the two in step when a command is added.
namespace funct {
  enum : unsigned {
    MVIN2 = 1, MVIN = 2, MVOUT = 3, COMPUTE_PRELOADED = 4, COMPUTE_ACCUMULATED = 5, PRELOAD = 6,
    LOOP_WS = 8, MVIN3 = 14, LOOP_CONV_WS = 15,
    MX_LOAD_SCALES = 27, MX_READ_SMEM = 28, MX_LOAD_LUT = 29,
    VPU_EXEC = 33, SPAD_REQUANT = 34,
  };
}

namespace {

gperf::model_t *g_model = nullptr;   // the instruction overrides have no extension pointer

gemmini_mode_t mode_from_env() {
  const char *m = getenv("GEMMINI_MODE");
  if (!m || !*m || !strcmp(m, "func")) return gemmini_mode_t::FUNC;
  if (!strcmp(m, "perf")) return gemmini_mode_t::PERF;
  if (!strcmp(m, "both")) return gemmini_mode_t::BOTH;
  fprintf(stderr, "gemmini: GEMMINI_MODE=%s is not func|perf|both\n", m);
  abort();
}

uint64_t instret(processor_t *p) { return p->get_state()->minstret->read(); }

// Spike adds a batch's instructions to minstret only when the batch ends (execute.cc). An instruction that needs
// the exact count first returns PC_SERIALIZE_BEFORE: the batch ends there and the instruction runs again at the
// start of the next one, with minstret exact -- what spike's own CSR reads do (validate_csr).
#ifndef PC_SERIALIZE_BEFORE
#define PC_SERIALIZE_BEFORE 3
#endif
bool serialize(processor_t *p) {
  state_t *s = p->get_state();
  if (!s->serialized) return false;
  s->serialized = false;
  return true;
}

insn_func_t g_rocc_orig = nullptr;   // the extension's RoCC handler (gemmini.cc gemmini_custom)

reg_t perf_rocc(processor_t *p, insn_t insn, reg_t pc) {
  if (!serialize(p)) return PC_SERIALIZE_BEFORE;
  return g_rocc_orig(p, insn, pc);
}

void report_at_exit() {
  if (!g_model) return;
  FILE *f = stderr;
  if (const char *path = getenv("GEMMINI_PERF_OUT")) f = fopen(path, "w");
  if (!f) f = stderr;
  g_model->finish();
  g_model->report(f);
  if (f != stderr) fclose(f);
}

// fence: the core waits for Gemmini to go idle (Rocket stalls a fence while rocc.busy).
reg_t perf_fence(processor_t *p, insn_t insn, reg_t pc) {
  (void)insn;
  if (!serialize(p)) return PC_SERIALIZE_BEFORE;
  if (g_model) g_model->fence(instret(p));
  return pc + 4;
}

// rdcycle (csrrs rd, cycle, x0): modelled time.
reg_t perf_rdcycle(processor_t *p, insn_t insn, reg_t pc) {
  if (!serialize(p)) return PC_SERIALIZE_BEFORE;
  const reg_t v = g_model ? (reg_t)g_model->now(instret(p)) : p->get_state()->mcycle->read();
  if (insn.rd() != 0) p->get_state()->XPR.write(insn.rd(), v);
  return pc + 4;
}

// The commands whose functional work is skipped in perf mode: everything that touches data (DRAM,
// scratchpad, accumulator, scale/LUT memories) or computes. Config commands are cheap and keep running,
// so the functional state they set stays as it would be.
bool moves_or_computes_data(unsigned f) {
  switch (f) {
    case funct::MVIN: case funct::MVIN2: case funct::MVIN3: case funct::MVOUT:
    case funct::PRELOAD: case funct::COMPUTE_PRELOADED: case funct::COMPUTE_ACCUMULATED:
    case funct::LOOP_WS: case funct::LOOP_CONV_WS:
    case funct::MX_LOAD_SCALES: case funct::MX_READ_SMEM: case funct::MX_LOAD_LUT:
    case funct::VPU_EXEC: case funct::SPAD_REQUANT:
      return true;
    default:
      return false;
  }
}

}  // namespace

gemmini_perf_t::gemmini_perf_t() = default;
gemmini_perf_t::~gemmini_perf_t() = default;

void gemmini_perf_t::reset() {
  mode_ = mode_from_env();
  if (mode_ == gemmini_mode_t::FUNC || model_) return;   // one model per run, across extension resets
  model_.reset(new gperf::model_t(gperf::config_t::from_env()));
  g_model = model_.get();
  static bool registered = false;
  if (!registered) { atexit(report_at_exit); registered = true; }
  fprintf(stderr, "gemmini: GEMMINI_MODE=%s\n", mode_ == gemmini_mode_t::PERF ? "perf" : "both");
}

bool gemmini_perf_t::on_cmd(processor_t *p, unsigned f, uint64_t rs1, uint64_t rs2) {
  if (mode_ == gemmini_mode_t::FUNC) return false;
  model_->command(f, rs1, rs2, instret(p));
  return mode_ == gemmini_mode_t::PERF && moves_or_computes_data(f);
}

void gemmini_perf_t::add_instructions(std::vector<insn_desc_t> &insns) {
  if (mode_from_env() == gemmini_mode_t::FUNC) return;
  // RoCC commands: serialize first so the model sees the exact instruction count (see serialize())
  for (auto &d : insns)
    if (d.fast_rv64i && d.fast_rv64i != perf_rocc && (d.match & 0x7F) == ROCC_OPCODE3) {
      g_rocc_orig = d.fast_rv64i;
      d.fast_rv64i = d.logged_rv64i = perf_rocc;
    }
  // extension instructions are matched before the base ISA (processor.cc decode_insn)
  push_custom_insn(insns, MATCH_FENCE, MASK_FENCE, perf_fence, perf_fence);
  push_custom_insn(insns, 0xC0002073u, 0xFFFFF07Fu, perf_rdcycle, perf_rdcycle);   // csrrs rd, cycle, x0
}
