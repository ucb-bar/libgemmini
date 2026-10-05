#ifndef GPERF_LOAD_UNIT_H
#define GPERF_LOAD_UNIT_H

#include <cstdint>
#include <deque>

#include "../params/config.h"
#include "dma.h"
#include "../sim/event_queue.h"
#include "../control/reservation_station.h"
#include "../memory/scratchpad.h"
#include "../sim/types.h"

namespace gperf {

// LoadController.scala: mvin. Issued commands wait in a ld.queue_length queue; up to ld.cmds_in_flight run at
// once, each streaming its rows through the shared DMA reader. A zero mvin (DRAM address 0) writes zeros
// through the bank write port. The RS entry completes ld.completion_lag after the last row is written.
class load_unit_t {
public:
  load_unit_t(const config_t &c, event_queue_t &eq, dma_reader_t &dma, scratchpad_t &sp, reservation_station_t &rs);

  // CONFIG_LD: rs1[4:3] state id, rs1[31:16] block stride, rs2 DRAM row stride.
  void config(uint64_t rs1, uint64_t rs2);

  struct mvin_t { local_addr_t dst; uint64_t dram; uint32_t rows, cols; int state; uint64_t stride; };  // stride 0 = CONFIG_LD's
  span_t dst_span(const mvin_t &m) const;

  bool has_room() const { return waiting_.size() < queue_len_; }
  void accept(uint64_t rs_id, mvin_t m);   // the RS issued it

  uint64_t bytes() const { return bytes_; }

private:
  struct job_t { uint64_t id; mvin_t m; uint64_t stride; };
  void try_start();

  event_queue_t &eq_;
  dma_reader_t &dma_;
  scratchpad_t &sp_;
  reservation_station_t &rs_;
  uint32_t dim_;
  size_t queue_len_;
  uint32_t max_running_;
  cycle_t lag_;
  uint64_t stride_[3] = {0, 0, 0};
  uint32_t block_stride_[3] = {0, 0, 0};
  std::deque<job_t> waiting_;
  uint32_t running_ = 0;
  uint64_t bytes_ = 0;
};

}  // namespace gperf

#endif
