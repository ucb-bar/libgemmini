#ifndef GPERF_LOOP_MATMUL_H
#define GPERF_LOOP_MATMUL_H

#include <cstdint>
#include <deque>

#include "../params/config.h"
#include "../sim/types.h"

namespace gperf {

// LoopMatmul.scala: LOOP_WS unrolled into RS commands. Shared unrollers (LdA, LdB, Ex, StC, StCSpad) each serve
// the configured loops in order; up to loop.concurrent_loops are configured at once, so a second loop's loads and
// computes interleave with the first one's stores -- the overlap double buffering is about. Each cycle the model
// asks for the next command: among the unrollers whose command may go now, the fixed priority
// StC > Ex > LdA/LdB > StCSpad (LdA/LdB balanced on k) picks one. A command may go when
//   * its loop has started,
//   * (Ex) the mvins of the A and B blocks it reads have left LoopMatmul,
//   * (St) the last-k compute of its tile has left (the loop's last store: every compute),
//   * the loop-issued commands of its class still in the RS (not completed) are under loop.max_*_outstanding.
// This class decides WHAT may go now; the model allocates it in the RS and reports back.
enum loop_kind_t { L_LDA, L_LDB, L_EX, L_STC, L_STSPAD, L_KINDS };
enum loop_class_t { LC_LD = 0, LC_EX = 1, LC_ST = 2 };   // RS tag of a loop-issued command

struct loop_cmd_t {
  loop_kind_t kind;
  int loop;                                   // index into the active loops
  // L_EX: one preload (b weights, c accumulator) + one compute (a)
  local_addr_t a{0xFFFFFFFFu}, b{0xFFFFFFFFu}, c{0xFFFFFFFFu};
  // L_LDA / L_LDB: an mvin of rows x cols at dram into local; L_STC / L_STSPAD: a store of the acc tile at local
  uint64_t dram = 0, stride = 0;
  local_addr_t local{0xFFFFFFFFu}, dst{0xFFFFFFFFu};
  uint32_t rows = 0, cols = 0;
  double out_bytes = 2.0;
  span_t out_span;                            // L_STSPAD: the scratchpad rows of the loop's output
};

class loop_matmul_t {
public:
  explicit loop_matmul_t(const config_t &c);

  // Loop config commands (funct 9-13, 24, 25, 31, 32) go to the slot being configured, which must be free.
  bool config_slot_free() const { return (int)active_.size() < max_loops_; }
  void config(unsigned funct, uint64_t rs1, uint64_t rs2);
  void run(uint64_t rs1, uint64_t rs2, cycle_t now, double out_bytes);   // LOOP_WS (funct 8)

  // The command that may leave LoopMatmul at `now`, if any. If none may go now but one will at a known later
  // cycle without any other event (a loop that has not started yet), *wake is that cycle, else NEVER.
  bool next(cycle_t now, loop_cmd_t *out, cycle_t *wake) const;
  void emitted(const loop_cmd_t &c);   // it was allocated in the RS
  void completed(int tag);             // an RS entry it issued completed

  bool idle() const { return active_.empty(); }
  uint64_t loops_run() const { return loops_run_; }

private:
  struct loop_t {
    uint32_t I = 0, J = 0, K = 0;
    uint64_t A = 0, B = 0, C = 0, D = 0, A_stride = 0, B_stride = 0, C_stride = 0;
    uint32_t a_sp = 0, b_end = 0, c_spad = 0, acc_base = 0;
    bool spad_only = false;
    double out_bytes = 2.0;
    cycle_t start = 0;
    uint32_t next[L_KINDS] = {0, 0, 0, 0, 0};
    uint32_t total[L_KINDS] = {0, 0, 0, 0, 0};
    bool done() const {
      for (int k = 0; k < L_KINDS; k++) if (next[k] < total[k]) return false;
      return true;
    }
  };
  // Build the next command of unroller k for loop li; false if its gates are closed now.
  bool make(const loop_t &l, int li, loop_kind_t k, cycle_t now, loop_cmd_t *o) const;

  int max_loops_;
  uint32_t mbl_, lim_ld_, lim_ex_, lim_st_, dim_, acc_half_, spad_half_;
  loop_t cfg_;                    // the loop being configured
  std::deque<loop_t> active_;     // configured loops, oldest first
  uint32_t out_[3] = {0, 0, 0};   // loop-issued RS entries not completed, per class
  uint32_t acc_cursor_ = 0;       // next alternating loop's accumulator C base
  uint64_t loops_run_ = 0;
};

}  // namespace gperf

#endif
