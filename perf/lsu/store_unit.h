#ifndef GPERF_STORE_UNIT_H
#define GPERF_STORE_UNIT_H

#include <cstdint>
#include <deque>

#include "../memory/accumulator.h"
#include "../params/config.h"
#include "dma.h"
#include "../sim/event_queue.h"
#include "../control/reservation_station.h"
#include "../memory/scratchpad.h"
#include "../sim/types.h"

namespace gperf {

// StoreController + the scratchpad write path + MxRequantizer. Commands run in order from an st.queue_length
// queue; each reads its rows one per cycle -- from a scratchpad bank's read port, or from an accumulator bank in
// the cycles the mesh is not writing it -- and its RS entry completes st.completion_lag after the last read.
// The data then reaches DRAM through the StreamWriter, or the scratchpad (requant_to_spad) through the bank write
// port at DIM bytes per cycle (priority requant); only fences wait for that.
class store_unit_t {
public:
  store_unit_t(const config_t &c, event_queue_t &eq, scratchpad_t &sp, accumulator_t &acc, dma_writer_t &dma,
               reservation_station_t &rs);

  void config(uint64_t rs1, uint64_t rs2);   // CONFIG_ST: rs2[31:0] DRAM row stride

  struct st_cmd_t {
    enum kind_t { TO_DRAM, TO_SPAD } kind = TO_DRAM;
    local_addr_t src{0xFFFFFFFFu};   // acc (bit 31) or scratchpad rows
    uint32_t rows = 0, cols = 0;
    uint64_t dram = 0, stride = 0;   // TO_DRAM; stride 0 = CONFIG_ST's
    local_addr_t dst{0xFFFFFFFFu};   // TO_SPAD
    double out_bytes = 2.0;          // per output element after the requantizer
    uint32_t out_mult = 1;           // output elements per acc element (multi-elem quad tile: 2 per quad operand)
  };
  span_t src_span(const st_cmd_t &c) const;

  bool has_room() const { return q_.size() < queue_len_; }
  void accept(uint64_t rs_id, st_cmd_t c);

  uint64_t reads() const { return reads_; }
  // Scratchpad banks a store is still writing (from its start to its last write-port beat): RTL store_pending +
  // requant_pending, Scratchpad.scala:954-977. DRAM stores never count.
  uint32_t pending_banks() const;
  cycle_t last_write() const { return last_write_; }   // requant output written into the scratchpad
  // An accumulator-source store is reading (its beats are in the requantizer): an FP4 SPAD_REQUANT may not start.
  bool acc_reading() const { return reading_ && acc_src_; }
  // While hold() is true no accumulator-source store starts (FP4 SPAD_REQUANT waiting or running, Controller
  // sr_fp4_hold, commit d3e6df6); call process() again when it may have cleared.
  void on_acc_hold(std::function<bool()> hold) { hold_ = std::move(hold); }
  void process();

private:
  struct item_t { uint64_t id; st_cmd_t c; uint64_t stride; };

  event_queue_t &eq_;
  scratchpad_t &sp_;
  accumulator_t &acc_;
  dma_writer_t &dma_;
  reservation_station_t &rs_;
  size_t queue_len_;
  uint32_t dim_;
  cycle_t lag_, rq_lat_, pipe_lat_, slack_, spad_rd_interval_, pend_delay_, spad_lead_, pend_tail_;
  double elems_per_read_;
  uint64_t stride_ = 0;
  std::deque<item_t> q_;
  bool reading_ = false, acc_src_ = false;
  std::function<bool()> hold_;
  uint64_t reads_ = 0;
  cycle_t last_write_ = 0;
  uint32_t pend_[32] = {};   // per scratchpad bank
};

}  // namespace gperf

#endif
