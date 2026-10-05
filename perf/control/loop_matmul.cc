#include "loop_matmul.h"

#include <algorithm>

namespace gperf {

namespace {
// command-stream priority of each unroller: lower wins (LoopMatmul.scala:1188-1195)
const int kPriority[L_KINDS] = {/*LDA*/ 2, /*LDB*/ 2, /*EX*/ 1, /*STC*/ 0, /*STSPAD*/ 3};
inline uint32_t ceil_div(uint32_t a, uint32_t b) { return (a + b - 1) / b; }
}  // namespace

loop_matmul_t::loop_matmul_t(const config_t &c)
    : max_loops_((int)c.loop_concurrent_loops), mbl_((uint32_t)c.loop_max_block_len),
      lim_ld_((uint32_t)c.loop_max_ld_outstanding), lim_ex_((uint32_t)c.loop_max_ex_outstanding),
      lim_st_((uint32_t)c.loop_max_st_outstanding), dim_((uint32_t)c.mesh_dim),
      acc_half_((uint32_t)(c.acc_total_rows / c.loop_concurrent_loops)),
      spad_half_((uint32_t)(c.spad_banks * c.spad_bank_rows / c.loop_concurrent_loops)) {}

void loop_matmul_t::config(unsigned funct, uint64_t rs1, uint64_t rs2) {
  switch (funct) {
    case 9:  cfg_.I = rs2 & 0xFFFF; cfg_.J = (rs2 >> 16) & 0xFFFF; cfg_.K = (rs2 >> 32) & 0xFFFF; break;
    case 10: cfg_.A = rs1; cfg_.B = rs2; break;
    case 11: cfg_.D = rs1; cfg_.C = rs2; break;
    case 12: cfg_.A_stride = rs1; cfg_.B_stride = rs2; break;
    case 13: cfg_.C_stride = rs2; break;
    case 24: cfg_.a_sp = (uint32_t)rs1; cfg_.b_end = (uint32_t)rs2; break;
    default: break;   // 25 has no effect in the RTL; 31/32 (loop-managed scales): plan step 4
  }
}

void loop_matmul_t::run(uint64_t rs1, uint64_t rs2, cycle_t now, double out_bytes) {
  loop_t l = cfg_;
  const uint32_t slot = (uint32_t)(loops_run_ % (uint64_t)max_loops_);
  l.spad_only = (rs2 >> 9) & 1;
  const bool skip_lda = (rs2 >> 3) & 1, skip_ldb = (rs2 >> 4) & 1, skip_ex = (rs2 >> 6) & 1, skip_st = (rs2 >> 7) & 1;
  l.c_spad = (uint32_t)(rs2 >> 32);
  l.out_bytes = out_bytes;
  l.start = now;
  if (!l.spad_only) {   // DRAM loop: operands land in the slot's half unless an explicit spad id is given
    const uint32_t a_id = (rs1 >> 18) & 3, b_id = (rs1 >> 16) & 3;
    l.a_sp = a_id ? (a_id - 1) * spad_half_ : slot * spad_half_;
    l.b_end = b_id ? b_id * spad_half_ : (slot + 1) * spad_half_;
  }
  // accumulator double buffer: the C base alternates per loop when C goes to DRAM or inc_acc_addr is set (:1573)
  const bool alternate = !l.spad_only || ((rs2 >> 8) & 1);
  l.acc_base = alternate ? acc_cursor_ : 0;
  if (alternate) acc_cursor_ = (acc_cursor_ + acc_half_) % (acc_half_ * (uint32_t)max_loops_);

  l.total[L_LDA] = (!l.spad_only && !skip_lda && l.A) ? l.I * ceil_div(l.K, mbl_) : 0;
  l.total[L_LDB] = (!l.spad_only && !skip_ldb && l.B) ? l.K * ceil_div(l.J, mbl_) : 0;
  l.total[L_EX] = skip_ex ? 0 : l.I * l.J * l.K;
  l.total[L_STC] = (!l.spad_only && !skip_st && l.C) ? l.I * ceil_div(l.J, mbl_) : 0;
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
    case L_LDA:
    case L_LDB: {
      if (out_[LC_LD] >= lim_ld_) return false;
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
      const bool spad = k == L_STSPAD;
      const uint32_t per_i = spad ? l.J : Jg;
      const uint32_t i = n / per_i, j0 = spad ? n % per_i : (n % per_i) * mbl_;
      const uint32_t blocks = spad ? 1 : std::min(mbl_, l.J - j0);
      if (l.total[L_EX]) {   // ex-ahead: the tile's last-k compute has left; the loop's last store waits for all
        const uint32_t jl = std::min(j0 + blocks - 1, l.J - 1);
        const uint32_t need = (n + 1 == l.total[k]) ? l.total[L_EX] : (l.K - 1) * l.J * l.I + jl * l.I + i + 1;
        if (l.next[L_EX] < need) return false;
      }
      o->local = acc_tile(i, j0);
      o->rows = dim_; o->cols = blocks * dim_;
      if (spad) {
        const uint64_t out_rows = (uint64_t)((double)l.I * l.J * dim_ * dim_ * l.out_bytes / dim_ + 0.5);
        o->dst = local_addr_t{l.c_spad};
        o->out_span = make_span(o->dst, out_rows, true);
      } else {
        o->dram = l.C + (uint64_t)(((double)i * dim_ * l.C_stride + (double)j0 * dim_) * l.out_bytes);
        o->stride = (uint64_t)(l.C_stride * l.out_bytes);
      }
      return true;
    }
    default: return false;
  }
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
        if (found && kPriority[c.kind] == kPriority[out->kind] && c.kind == L_LDB && out->kind == L_LDA) {
          // LdA vs LdB: B goes when A's k is ahead (WeightedArbiter, weightA = 0)
          const uint32_t a_k = (l.next[L_LDA] / std::max(l.I, 1u)) * mbl_;
          const uint32_t b_k = l.next[L_LDB] / std::max(ceil_div(l.J, mbl_), 1u);
          better = a_k > b_k;
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
  if (c.kind == L_LDA || c.kind == L_LDB) out_[LC_LD]++;
  else if (c.kind == L_EX) out_[LC_EX] += 2;
  else out_[LC_ST]++;
  while (!active_.empty() && active_.front().done()) active_.pop_front();   // the head slot frees
}

void loop_matmul_t::completed(int tag) {
  if (tag >= 0 && tag < 3 && out_[tag] > 0) out_[tag]--;
}

}  // namespace gperf
