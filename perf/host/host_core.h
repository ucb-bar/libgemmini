#ifndef GPERF_HOST_CORE_H
#define GPERF_HOST_CORE_H

#include <cstdint>
#include <unordered_map>

#include "../params/config.h"
#include "../sim/types.h"

namespace gperf {

// The CPU's in-order pipeline, approximately (mem.host_tracking 2 + host.icache): every executed instruction (from
// the fetch trace) is decoded once per PC and goes through a register scoreboard -- it issues no earlier than one
// cycle after the previous one and no earlier than its sources are ready; divides / square roots also wait for
// their unpipelined unit; a branch adds its measured average penalty (taken / not taken is exact: the next fetched
// PC). Every latency is measured from the VCS commit traces (tools/commit_latency.py, perf_model_plan.md 13.12).
// The result is stall cycles beyond one per instruction, added to the CPU clock like the L1 miss stalls.
class host_core_t {
public:
  explicit host_core_t(const config_t &c);

  // Instruction at pc retired (bits = its encoding, 2 or 4 bytes). Returns the stall cycles it added.
  double insn(uint64_t pc, uint32_t bits);
  // The core stalled for c cycles for another reason (an L1 miss): the pipeline's clock moves on with it.
  void external_stall(cycle_t c) { cur_ += (double)c; }
  double stall() const { return stall_; }
  uint64_t count(int kind) const { return n_[kind]; }

  enum kind_t { K_ALU, K_LOAD, K_FLOAD, K_STORE, K_FMA, K_FMISC, K_FDIV, K_FSQRT, K_MUL, K_DIV, K_BR, K_JAL, K_JALR,
                K_N };
  struct dec_t { uint8_t kind, len; int8_t rd, rs[3]; };
  static dec_t decode(uint32_t bits);

private:
  double lat_[K_N];
  double br_taken_, br_not_, jal_, jalr_;
  double cur_ = 0, stall_ = 0;
  double ready_[64] = {};       // x0..x31, f0..f31
  double fdiv_free_ = 0, div_free_ = 0;
  bool prev_branch_ = false;
  uint8_t prev_kind_ = K_ALU;
  uint64_t prev_next_pc_ = 0;
  uint64_t n_[K_N] = {};
};

}  // namespace gperf

#endif
