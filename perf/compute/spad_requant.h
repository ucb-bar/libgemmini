#ifndef GPERF_SPAD_REQUANT_H
#define GPERF_SPAD_REQUANT_H

#include <cstdint>
#include <functional>
#include <memory>

#include "../params/config.h"
#include "../sim/event_queue.h"
#include "../control/reservation_station.h"
#include "../memory/scratchpad.h"
#include "../sim/types.h"

namespace gperf {

// SPAD_REQUANT (SpadRequant.scala): a BF16 M x N tile in the scratchpad -> E4M3 codes + E8M0 scales, one at a
// time. Each 32-element block reads 4 rows and writes 2, at the lowest priority on both bank ports, about
// sreq.cycles_per_block per block; sreq.fixed_cycles covers the start wait and the scale flush.
// Not calibrated yet (perf_model_plan.md step 5).
class spad_requant_t {
public:
  spad_requant_t(const config_t &c, event_queue_t &eq, scratchpad_t &sp, reservation_station_t &rs)
      : eq_(eq), sp_(sp), rs_(rs), dim_((uint32_t)c.mesh_dim), per_block_((cycle_t)c.sreq_cycles_per_block),
        fixed_((cycle_t)c.sreq_fixed_cycles) {}

  struct cmd_t { uint32_t src, dst, M, N; bool tiled, resident, fp4; };
  static cmd_t decode(uint64_t rs1, uint64_t rs2) {
    return {(uint32_t)(rs1 & 0x3FFF), (uint32_t)((rs1 >> 14) & 0x3FFF), (uint32_t)(rs2 & 0xFFFF),
            (uint32_t)((rs2 >> 16) & 0xFFFF), (bool)((rs1 >> 28) & 1), (bool)((rs1 >> 29) & 1),
            (bool)((rs2 >> 32) & 1)};
  }
  // exact footprints as the RS computes them (LoopMatmul.scala vec_bypass): BF16 source M*N/8 rows, E4M3
  // destination M*N/16 rows (M padded to 16 when tiled); FP4: two codes a byte, M*N/32 rows (M padded to 32)
  span_t read_span(const cmd_t &c) const { return make_span(local_addr_t{c.src}, (uint64_t)c.M * c.N * 2 / dim_, false); }
  span_t write_span(const cmd_t &c) const {
    const uint32_t pad = c.fp4 ? 31 : 15;
    const uint64_t m = c.tiled ? ((c.M + pad) & ~pad) : c.M;
    return make_span(local_addr_t{c.dst}, m * c.N / (c.fp4 ? 2 * dim_ : dim_), true);
  }

  bool has_room() const { return !busy_; }
  bool fp4_active() const { return busy_ && fp4_; }
  void on_done(std::function<void()> cb) { done_cb_ = std::move(cb); }
  void accept(uint64_t rs_id, cmd_t c) {
    busy_ = true;
    fp4_ = c.fp4;
    const uint64_t blocks = (uint64_t)c.M * c.N / 32;
    const cycle_t now = eq_.now();
    // reads pace the blocks (4 rows each, per_block_ cycles); the 2 written rows per block trail them
    auto left = std::make_shared<int>(2);
    auto part = [this, rs_id, left](cycle_t t) {
      if (--*left > 0) return;
      eq_.at(t + fixed_, [this, rs_id] {
        busy_ = false;
        rs_.complete(rs_id);
        rs_.kick(Q_VEC);
        if (done_cb_) done_cb_();
      });
    };
    sp_.read_port(c.src).request(SP_SREQ, now + 1, (cycle_t)blocks * per_block_, part);
    sp_.write_port(c.dst).request(SP_SREQ, now + 1 + per_block_, (cycle_t)blocks * 2, part);
  }

private:
  event_queue_t &eq_;
  scratchpad_t &sp_;
  reservation_station_t &rs_;
  uint32_t dim_;
  cycle_t per_block_, fixed_;
  bool busy_ = false, fp4_ = false;
  std::function<void()> done_cb_;
};

}  // namespace gperf

#endif
