#include "loop_matmul.h"

#include <algorithm>

namespace gperf {

namespace {
// command-stream priority of each unroller: lower wins (LoopMatmul.scala:1188-1195)
const int kPriority[L_KINDS] = {/*LDA*/ 3, /*LDB*/ 3, /*EX*/ 2, /*STC*/ 1, /*STSPAD*/ 4, /*LDS*/ 0, /*SCFG*/ 2};
inline uint32_t ceil_div(uint32_t a, uint32_t b) { return (a + b - 1) / b; }
}  // namespace

loop_matmul_t::loop_matmul_t(const config_t &c)
    : max_loops_((int)c.loop_concurrent_loops), mbl_((uint32_t)c.loop_max_block_len),
      lim_ld_((uint32_t)c.loop_max_ld_outstanding), lim_ex_((uint32_t)c.loop_max_ex_outstanding),
      lim_st_((uint32_t)c.loop_max_st_outstanding), dim_((uint32_t)c.mesh_dim),
      acc_half_((uint32_t)(c.acc_total_rows / c.loop_concurrent_loops)),
      spad_half_((uint32_t)(c.spad_banks * c.spad_bank_rows / c.loop_concurrent_loops)),
      tiles_per_mx_block_(std::max(1u, (uint32_t)c.mx_block / (uint32_t)c.mesh_dim)) {}

void loop_matmul_t::config(unsigned funct, uint64_t rs1, uint64_t rs2) {
  switch (funct) {
    case 9:  cfg_.I = rs2 & 0xFFFF; cfg_.J = (rs2 >> 16) & 0xFFFF; cfg_.K = (rs2 >> 32) & 0xFFFF; break;
    case 10: cfg_.A = rs1; cfg_.B = rs2; break;
    case 11: cfg_.D = rs1; cfg_.C = rs2; break;
    case 12: cfg_.A_stride = rs1; cfg_.B_stride = rs2; break;
    case 13: cfg_.C_stride = rs2; break;
    case 24: cfg_.a_sp = (uint32_t)rs1; cfg_.b_end = (uint32_t)rs2; break;
    case 31: cfg_.A_sc = rs1; cfg_.B_sc = rs2; break;
    case 32: cfg_.A_sc_stride = rs1; cfg_.B_sc_stride = rs2; break;
    default: break;   // 25 has no effect in the RTL
  }
}

void loop_matmul_t::run(uint64_t rs1, uint64_t rs2, cycle_t now, double out_bytes, bool reads_act0) {
  loop_t l = cfg_;
  cfg_.A_sc = cfg_.B_sc = 0;   // loop-managed scales are per loop
  l.slot = (uint32_t)(loops_run_ % (uint64_t)max_loops_);
  l.serial = loops_run_;
  l.spad_only = (rs2 >> 9) & 1;
  l.inc_acc = (rs2 >> 8) & 1;
  const bool skip_lda = (rs2 >> 3) & 1, skip_ldb = (rs2 >> 4) & 1, skip_ex = (rs2 >> 6) & 1, skip_st = (rs2 >> 7) & 1;
  l.c_spad = (uint32_t)(rs2 >> 32);
  l.out_bytes = out_bytes;
  l.start = now;
  if (!l.spad_only) {   // DRAM loop: operands land in the slot's half unless an explicit spad id is given
    const uint32_t a_id = (rs1 >> 18) & 3, b_id = (rs1 >> 16) & 3;
    l.a_sp = a_id ? (a_id - 1) * spad_half_ : l.slot * spad_half_;
    l.b_end = b_id ? b_id * spad_half_ : (l.slot + 1) * spad_half_;
  }
  // accumulator double buffer: the C base alternates per loop when C goes to DRAM or inc_acc_addr is set (:1573)
  const bool alternate = !l.spad_only || ((rs2 >> 8) & 1);
  l.acc_base = alternate ? acc_cursor_ : 0;
  if (alternate) acc_cursor_ = (acc_cursor_ + acc_half_) % (acc_half_ * (uint32_t)max_loops_);
  // A-scale reuse (:1517): A resident and this slot's half still holds the same slice
  const a_scale_slice_t &s = asc_[l.slot];
  l.a_reuse = l.A_sc && l.A == 0 && asc_valid_[l.slot] && s.addr == l.A_sc && s.stride == l.A_sc_stride &&
              s.I == l.I && s.K == l.K;

  // a loop-managed config reads act half = slot (ReservationStation.scala: scale_cfg_after_sr)
  l.reads_act0 = l.A_sc ? (l.slot % 2 == 0) : reads_act0;
  l.total[L_LDS] = l.A_sc ? (l.a_reuse ? 1 : 2) : 0;
  l.total[L_SCFG] = l.A_sc ? 1 : 0;
  l.total[L_LDA] = (!l.spad_only && !skip_lda && l.A) ? l.I * ceil_div(l.K, mbl_) : 0;
  l.total[L_LDB] = (!l.spad_only && !skip_ldb && l.B) ? l.K * ceil_div(l.J, mbl_) : 0;
  l.total[L_EX] = skip_ex ? 0 : l.I * l.J * l.K;
  l.total[L_STC] = (!l.spad_only && !skip_st && l.C) ? l.I * stc_per_i(l.J) : 0;
  l.total[L_STSPAD] = (l.spad_only && !skip_st) ? l.I * l.J : 0;
  loops_run_++;
  if (!l.done()) active_.push_back(l);
}

bool loop_matmul_t::make(const loop_t &l, int li, loop_kind_t k, cycle_t now, loop_cmd_t *o) const {
  const uint32_t n = l.next[k];
  if (n >= l.total[k] || now <= l.start) return false;
  const uint32_t Jg = ceil_div(l.J, mbl_), J4 = ceil_div(l.J, 4) * 4;
  const uint32_t b_sp = l.b_end - l.K * l.J * dim_;
  auto acc_tile = [&](uint32_t i, uint32_t j) {
    return local_addr_t{(1u << 31) | (l.acc_base + (i * J4 + j) * (dim_ / 4))};   // single throughput (:407)
  };
  *o = loop_cmd_t{};
  o->kind = k;
  o->loop = li;
  o->out_bytes = l.out_bytes;
  switch (k) {
    case L_LDS: {   // A (unless reused) then B: 2-D, rows = k blocks, row = I or J tiles of DIM bytes, dest = slot
      const bool is_b = l.a_reuse || n == 1;
      o->sel = is_b ? 1 : 0;
      o->half = (int)(l.slot & 1);
      o->dram = is_b ? l.B_sc : l.A_sc;
      o->stride = is_b ? l.B_sc_stride : l.A_sc_stride;
      o->rows = l.K / tiles_per_mx_block_;
      o->len = (is_b ? l.J : l.I) * dim_;
      return true;
    }
    case L_SCFG:   // the loop's managed CONFIG_SCALE_MEM: after its scale loads have left
      if (l.next[L_LDS] < l.total[L_LDS] || out_[LC_EX] + 1 > lim_ex_) return false;
      // it is the Ex unroller's first command for this loop (LoopMatmulExecute: req -> state cfg, :505), and that
      // unroller only takes the loop once every older loop's computes have left
      for (int o = 0; o < li; o++)
        if (active_[(size_t)o].next[L_EX] < active_[(size_t)o].total[L_EX]) return false;
      o->half = (int)(l.slot & 1);
      o->a_reuse = l.a_reuse;
      return true;
    case L_LDA:
    case L_LDB: {
      if (out_[LC_LD] >= lim_ld_) return false;
      if (n == 0 && li > 0 && ld_blocked(l, k)) return false;
      {   // the arbiter forces the other unroller while it still points at the head loop
        const uint64_t other = unroller_loop(k == L_LDA ? L_LDB : L_LDA);
        if (other != UINT64_MAX && other != l.serial && other == active_.front().serial) return false;
      }
      if (k == L_LDA) {
        const uint32_t i = n % l.I, k0 = (n / l.I) * mbl_, blocks = std::min(mbl_, l.K - k0);
        o->dram = l.A + ((uint64_t)i * dim_ * l.A_stride + (uint64_t)k0 * dim_);
        o->stride = l.A_stride;
        o->local = local_addr_t{l.a_sp + (i * l.K + k0) * dim_};
        o->rows = dim_; o->cols = blocks * dim_;
      } else {
        const uint32_t kk = n / Jg, j0 = (n % Jg) * mbl_, blocks = std::min(mbl_, l.J - j0);
        o->dram = l.B + ((uint64_t)kk * dim_ * l.B_stride + (uint64_t)j0 * dim_);
        o->stride = l.B_stride;
        o->local = local_addr_t{b_sp + (kk * l.J + j0) * dim_};
        o->rows = dim_; o->cols = blocks * dim_;
      }
      return true;
    }
    case L_EX: {
      if (out_[LC_EX] + 2 > lim_ex_) return false;
      if (l.next[L_SCFG] < l.total[L_SCFG]) return false;   // the scale config goes first
      const uint32_t kk = n / (l.J * l.I), j = (n / l.I) % l.J, i = n % l.I;
      // the A and B mvins this compute reads must have left LoopMatmul
      if (l.total[L_LDA] && l.next[L_LDA] <= (kk / mbl_) * l.I + i) return false;
      if (l.total[L_LDB] && l.next[L_LDB] <= kk * Jg + j / mbl_) return false;
      o->a = local_addr_t{l.a_sp + (i * l.K + kk) * dim_};
      o->b = i == 0 ? local_addr_t{b_sp + (kk * l.J + j) * dim_} : local_addr_t{0xFFFFFFFFu};
      o->c = acc_tile(i, j);
      o->rows = dim_;
      return true;
    }
    case L_STC:
    case L_STSPAD: {
      if (out_[LC_ST] >= lim_st_) return false;
      uint32_t i, j0, blocks, group_last;
      if (k == L_STSPAD) {
        i = n / l.J; j0 = n % l.J; blocks = 1; group_last = j0;
      } else {   // chunks of stc_tiles() j tiles per (i, j group); a partial last group has fewer (:714-716)
        const uint32_t per_i = stc_per_i(l.J), ct = stc_tiles(), full = stc_chunks(mbl_);
        i = n / per_i;
        const uint32_t r = n % per_i, jg = std::min(r / full, Jg - 1), h = r - jg * full;
        const uint32_t g0 = jg * mbl_, gb = std::min(mbl_, l.J - g0);
        j0 = g0 + h * ct;
        blocks = std::min(ct, gb - std::min(gb, h * ct));
        if (!blocks) blocks = 1;
        group_last = g0 + gb - 1;
      }
      if (l.total[L_EX]) {   // ex-ahead: the group's last-k compute has left; the loop's last store waits for all
        const uint32_t need = (n + 1 == l.total[k]) ? l.total[L_EX] : (l.K - 1) * l.J * l.I + group_last * l.I + i + 1;
        if (l.next[L_EX] < need) return false;
      }
      o->local = acc_tile(i, j0);
      o->rows = dim_; o->cols = blocks * dim_;
      if (k == L_STSPAD) {
        const uint64_t out_rows = (uint64_t)((double)l.I * l.J * dim_ * dim_ * l.out_bytes / dim_ + 0.5);
        o->dst = local_addr_t{l.c_spad};
        o->out_span = make_span(o->dst, out_rows, true);
      } else {
        o->dram = l.C + (uint64_t)(((double)i * dim_ * l.C_stride + (double)j0 * dim_) * l.out_bytes);
        // The RTL gives every chunk of (i, j group) the group's acc address and packed cols = tiles * DIM/4 (<= DIM:
        // one mat), so the RS range is `rows` rows from the group base -- tile row i's packed rows, never row i+1.
        o->rs_span = make_span(acc_tile(i, (j0 / mbl_) * mbl_), dim_, false);
        o->stride = (uint64_t)(l.C_stride * l.out_bytes);
      }
      return true;
    }
    default: return false;
  }
}

// The loop the A or B load unroller points at: the last one it served, moved on through any newer loops that
// have none of its loads (an A = NULL loop still "starts" its LdA unroller, with zero commands, :1474-1476).
uint64_t loop_matmul_t::unroller_loop(loop_kind_t k) const {
  // An unroller takes the next loop's request as soon as it is idle (LoopMatmulLdA/B io.req.fire sets loop_id), so
  // once it has emitted every command of its loop it already points at the next active loop -- unless that loop's
  // request is withheld by ld_blocked (req.valid, LoopMatmul.scala:1475-1476 / 1496-1497): then it keeps the old id.
  // A loop with none of its loads (A = NULL) passes straight through. With no newer loop it keeps its id.
  uint64_t at = k == L_LDA ? lda_loop_ : ldb_loop_;
  for (const loop_t &l : active_) {
    if (at != UINT64_MAX && l.serial < at) continue;
    if (l.serial == at) {
      if (l.next[k] < l.total[k]) return at;   // still busy on it
      continue;
    }
    if (l.next[k] < l.total[k]) {
      if (l.next[k] == 0 && &l != &active_.front() && ld_blocked(l, k)) return at;   // request not sent yet
      return l.serial;
    }
    at = l.serial;
  }
  return at;
}

bool loop_matmul_t::rows_hit(const span_t &s) const {
  if (!s.valid || (s.lo >> 40)) return false;   // accumulator ranges never hit spad rows
  for (const loop_t &l : active_) {
    const uint64_t n = (uint64_t)l.K * l.J * dim_;
    const uint64_t a0 = l.a_sp, a1 = a0 + (uint64_t)l.I * l.K * dim_;
    const uint64_t b0 = l.b_end > n ? l.b_end - n : 0, b1 = l.b_end;
    const uint64_t c0 = l.c_spad, c1 = c0 + 2ull * l.I * l.J * dim_;
    auto hit = [&](uint64_t lo, uint64_t hi) { return s.lo < hi && lo < s.hi; };
    if (hit(a0, a1) || hit(b0, b1) || (l.spad_only && hit(c0, c1))) return true;
  }
  return false;
}

bool loop_matmul_t::resident_sr_blocked() const {
  for (const loop_t &l : active_)
    if (l.reads_act0 && l.next[L_EX] < l.total[L_EX]) return true;
  return false;
}

bool loop_matmul_t::only_stores_left() const {
  for (const loop_t &l : active_) {
    const bool alternates = l.spad_only ? l.inc_acc : l.C != 0;
    if (l.next[L_EX] < l.total[L_EX] || l.next[L_LDA] < l.total[L_LDA] || l.next[L_LDB] < l.total[L_LDB] ||
        l.next[L_SCFG] < l.total[L_SCFG] || !alternates)
      return false;
  }
  return true;
}

bool loop_matmul_t::loads_done() const {
  for (const loop_t &l : active_)
    if (l.next[L_LDA] < l.total[L_LDA] || l.next[L_LDB] < l.total[L_LDB]) return false;
  return true;
}

// LoopMatmul.scala:1376-1388: rows a loop's A / B occupy, and the head-loop overlap that holds a newer loop's loads.
bool loop_matmul_t::ld_blocked(const loop_t &l, loop_kind_t k) const {
  const loop_t &h = active_.front();
  if (h.next[L_EX] >= h.total[L_EX]) return false;   // the head has issued all its computes
  auto a_rows = [&](const loop_t &x) { return std::make_pair(x.a_sp, x.a_sp + x.I * x.K * dim_); };
  auto b_rows = [&](const loop_t &x) {
    const uint32_t n = x.K * x.J * dim_;
    return std::make_pair(x.b_end > n ? x.b_end - n : 0u, x.b_end);
  };
  auto overlap = [](std::pair<uint32_t, uint32_t> x, std::pair<uint32_t, uint32_t> y) {
    return x.first < y.second && y.first < x.second;
  };
  const auto mine = k == L_LDA ? a_rows(l) : b_rows(l);
  return overlap(mine, a_rows(h)) || overlap(mine, b_rows(h));
}

bool loop_matmul_t::next(cycle_t now, loop_cmd_t *out, cycle_t *wake) const {
  *wake = NEVER;
  bool found = false;
  for (int k = 0; k < L_KINDS; k++) {
    // each unroller serves the oldest loop that still has commands of its kind
    for (size_t li = 0; li < active_.size(); li++) {
      const loop_t &l = active_[li];
      if (l.next[k] >= l.total[k]) continue;
      if (now <= l.start) { *wake = std::min(*wake, l.start + 1); break; }
      loop_cmd_t c;
      if (make(l, (int)li, (loop_kind_t)k, now, &c)) {
        bool better = !found || kPriority[c.kind] < kPriority[out->kind];
        if (found && c.kind == L_LDB && out->kind == L_LDA) {
          const loop_t &la = active_[(size_t)out->loop];
          if (out->loop != c.loop) {
            better = c.loop == 0;   // different loops: the head loop's unroller is forced
          } else {
            const uint32_t a_k = (la.next[L_LDA] / std::max(la.I, 1u)) * mbl_;
            const uint32_t jg = std::max(ceil_div(l.J, mbl_), 1u);
            const uint32_t b_k = l.next[L_LDB] / jg, b_j = (l.next[L_LDB] % jg) * mbl_;
            better = (b_k == 0 && b_j == 0) || a_k > b_k;
          }
        }
        if (better) { *out = c; found = true; }
      }
      break;
    }
  }
  return found;
}

void loop_matmul_t::emitted(const loop_cmd_t &c) {
  loop_t &l = active_[(size_t)c.loop];
  l.next[c.kind]++;
  switch (c.kind) {
    case L_LDA: out_[LC_LD]++; lda_loop_ = l.serial; break;
    case L_LDB: out_[LC_LD]++; ldb_loop_ = l.serial; break;
    case L_EX: out_[LC_EX] += 2; break;
    case L_SCFG: out_[LC_EX]++; break;
    case L_STC: case L_STSPAD: out_[LC_ST]++; break;
    case L_LDS:
      if (c.sel == 0) {   // remember the A slice this slot now holds
        asc_valid_[l.slot] = true;
        asc_[l.slot] = {l.A_sc, l.A_sc_stride, l.I, l.K};
      }
      break;
    default: break;
  }
  while (!active_.empty() && active_.front().done()) active_.pop_front();   // the head slot frees
}

void loop_matmul_t::completed(int tag) {
  if (tag >= 0 && tag < 3 && out_[tag] > 0) out_[tag]--;
}

}  // namespace gperf
