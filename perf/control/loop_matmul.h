#ifndef GPERF_LOOP_MATMUL_H
#define GPERF_LOOP_MATMUL_H

#include <algorithm>
#include <cstdint>
#include <deque>

#include "../params/config.h"
#include "../sim/types.h"

namespace gperf {

// LoopMatmul.scala: LOOP_WS unrolled into commands. Shared unrollers each serve the configured loops in order;
// up to loop.concurrent_loops are configured at once, so a second loop's loads and computes interleave with the
// first one's stores -- the overlap double buffering is about. Each cycle the model asks for the next command:
// among the unrollers whose command may go now, the fixed priority LdS > StC > Ex > LdA/LdB > StCSpad
// (LdA/LdB balanced on k; :1188-1195) picks one.
//
// Unrollers:
//   LdS     loop-managed scales (LOOP_WS_CONFIG_SCALES set): the A then the B scale slice as gated 2-D
//           MX_LOAD_SCALES into scale half = loop slot (to the scale loader, not the RS; :1503-1554). The A load
//           is skipped when A is resident (A = NULL) and that half still holds the same slice.
//   LdA/LdB mvins of A (I x ceil(K/4)) and B (K x ceil(J/4)) from DRAM, 4 DIM-wide blocks each.
//   Ex      [managed CONFIG_SCALE_MEM first, once its loop's scale loads have left] then preload+compute per
//           (k, j, i), k outer, i inner. Gated on the A and B mvins it reads having left.
//   StC     C to DRAM: two commands per (i, group of 4 j tiles) (numChunks = 2, single throughput).
//   StCSpad C to the scratchpad (LOOP_WS spad variant): one per (i, j).
// A store waits until the last-k compute of its tiles has left (the loop's last store: every compute). Each class
// (ld / ex / st) keeps at most loop.max_*_outstanding loop-issued commands in the RS that have not completed.
enum loop_kind_t { L_LDA, L_LDB, L_EX, L_STC, L_STSPAD, L_LDS, L_SCFG, L_KINDS };
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
  uint32_t out_mult = 1;                      // output elements per acc-tile element (multi-elem: 2 per quad operand)
  span_t out_span;                            // L_STSPAD: the scratchpad rows of the loop's output
  span_t rs_span;                             // L_STC: the RS's range for this mvout (LoopMatmul.scala:602-611)
  // L_LDS: a gated scale load; L_SCFG: the loop's managed scale config
  int sel = 0, half = 0;                      // L_LDS: 0 = A (activation) scales, 1 = B (weight); half = slot
  uint32_t len = 0;                           // L_LDS: bytes per row (rows = above)
  bool a_reuse = false;                       // L_SCFG: the A scales are reused in place (no A load issued)
};

class loop_matmul_t {
public:
  explicit loop_matmul_t(const config_t &c);

  // Loop config commands (funct 9-13, 24, 25, 31, 32) go to the slot being configured, which must be free.
  bool config_slot_free() const { return (int)active_.size() < max_loops_; }
  void config(unsigned funct, uint64_t rs1, uint64_t rs2);
  // LOOP_WS (funct 8). reads_act0: (not loop-managed) the last scale config may read act half 0.
  // multi / multi_act: mx_multi_elem / mx_multi_elem_act (ExecuteController.scala:177-187): the weight / activation
  // is quad (2 output cols / rows per lane), set by the last CONFIG_EX.
  void run(uint64_t rs1, uint64_t rs2, cycle_t now, double out_bytes, bool reads_act0 = true, bool multi = false,
           bool multi_act = false);
  // A non-loop command passed LoopMatmul: the A-scale reuse record is no longer trusted (:1554).
  void invalidate_scale_reuse() { asc_valid_[0] = asc_valid_[1] = false; }

  // The command that may leave LoopMatmul at `now`, if any. If none may go now but one will at a known later
  // cycle without any other event (a loop that has not started yet), *wake is that cycle, else NEVER.
  bool next(cycle_t now, loop_cmd_t *out, cycle_t *wake) const;
  void emitted(const loop_cmd_t &c);   // it left LoopMatmul (into the RS, or to the scale loader)
  void completed(int tag);             // an RS entry it issued completed

  bool idle() const { return active_.empty(); }

  // VPU-config bypasses (LoopMatmul.scala vec_bypass): may a raw command pass the configured loops?
  bool rows_hit(const span_t &s) const;              // s touches a loop's A, B or (spad_only) C rows
  bool resident_sr_blocked() const;                  // a loop reading act half 0 still has computes to issue
  bool only_stores_left() const;                     // every loop: loads + computes issued, acc base alternates
  bool loads_done() const;                           // every loop: all its loads issued
  bool ex_has_room() const { return out_[LC_EX] < lim_ex_; }
  void note_passed_ex() { out_[LC_EX]++; }           // a passed CONFIG_SCALE_MEM counts as a loop ex command
  uint64_t loops_run() const { return loops_run_; }

private:
  struct loop_t {
    uint32_t I = 0, J = 0, K = 0;
    uint64_t A = 0, B = 0, C = 0, D = 0, A_stride = 0, B_stride = 0, C_stride = 0;
    uint64_t A_sc = 0, B_sc = 0, A_sc_stride = 0, B_sc_stride = 0;
    uint32_t a_sp = 0, b_end = 0, c_spad = 0, acc_base = 0, slot = 0;
    bool spad_only = false, a_reuse = false, inc_acc = false, reads_act0 = true, tiled = false;
    bool multi = false, multi_act = false;   // narrow_type / narrow_act (LoopMatmul.scala:1344-1345)
    bool ex_acc = false;                     // LOOP_WS rs1[0] ex_accumulate (:1340)
    double out_bytes = 2.0;
    cycle_t start = 0;
    uint64_t serial = 0;          // which LOOP_WS this is (0, 1, ...)
    uint32_t next[L_KINDS] = {0, 0, 0, 0, 0, 0, 0};
    uint32_t total[L_KINDS] = {0, 0, 0, 0, 0, 0, 0};
    bool done() const {
      for (int k = 0; k < L_KINDS; k++) if (next[k] < total[k]) return false;
      return true;
    }
  };
  struct a_scale_slice_t { uint64_t addr, stride; uint32_t I, K; };
  // Build the next command of unroller k for loop li; false if its gates are closed now.
  bool make(const loop_t &l, int li, loop_kind_t k, cycle_t now, loop_cmd_t *o) const;
  bool ld_blocked(const loop_t &l, loop_kind_t k) const;
  uint64_t unroller_loop(loop_kind_t k) const;
  // StC chunking (LoopMatmul.scala:573-577, 714-720): one chunk = one acc chunk = mbl/2 j tiles (32 BF16 columns,
  // whole 64 B lines); a j group of gb tiles stores gb * numChunks / 4 chunks (at least 1), so a 2-tile group is ONE
  // chunk, not two halves.
  // LoopMatmulStCSpad (single throughput, LoopMatmul.scala:821-980): one store per 2-tile chunk of a 4-tile j group
  // (chunks_this_j = tiles / 2), groups outer, i inner; every chunk's acc range is its group's DIM rows
  static uint32_t sts_group_tiles(uint32_t J, uint32_t g) { return (g + 1) * 4 > J && J % 4 ? J % 4 : 4; }
  static uint32_t sts_chunks(uint32_t J, uint32_t g) { return std::max(1u, sts_group_tiles(J, g) / 2); }
  static uint32_t sts_per_i(uint32_t J) {
    uint32_t n = 0;
    for (uint32_t g = 0; g < (J + 3) / 4; g++) n += sts_chunks(J, g);
    return n;
  }
  uint32_t stc_tiles() const { return std::max(1u, mbl_ / 2); }
  uint32_t stc_chunks(uint32_t gb) const { return std::max(1u, gb / stc_tiles()); }
  uint32_t stc_per_i(uint32_t J) const {
    const uint32_t g = (J + mbl_ - 1) / mbl_;
    return g ? (g - 1) * stc_chunks(mbl_) + stc_chunks(J - (g - 1) * mbl_) : 0;
  }

  int max_loops_;
  uint32_t mbl_, lim_ld_, lim_ex_, lim_st_, dim_, acc_half_, spad_half_, tiles_per_mx_block_;
  loop_t cfg_;                    // the loop being configured
  std::deque<loop_t> active_;     // configured loops, oldest first
  uint32_t out_[3] = {0, 0, 0};   // loop-issued RS entries not completed, per class
  uint32_t acc_cursor_ = 0;       // next alternating loop's accumulator C base
  bool asc_valid_[2] = {false, false};
  a_scale_slice_t asc_[2] = {};
  uint64_t loops_run_ = 0;
  // The loop each load unroller served last. An unroller moves to the next loop as soon as it has emitted all of its
  // own (unroller_loop); the A/B arbiter forces the one on the head loop only when the two are on different loops
  // (LoopMatmul.scala:1173-1176) -- so a newer loop's loads of one kind wait while the other unroller is still busy
  // with the head loop.
  uint64_t lda_loop_ = UINT64_MAX, ldb_loop_ = UINT64_MAX;
};

}  // namespace gperf

#endif
