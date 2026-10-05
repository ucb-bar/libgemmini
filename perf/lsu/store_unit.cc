#include "store_unit.h"

#include <cmath>

namespace gperf {

store_unit_t::store_unit_t(const config_t &c, event_queue_t &eq, scratchpad_t &sp, accumulator_t &acc,
                           dma_writer_t &dma, reservation_station_t &rs)
    : eq_(eq), sp_(sp), acc_(acc), dma_(dma), rs_(rs), queue_len_((size_t)c.st_queue_length),
      dim_((uint32_t)c.mesh_dim), lag_((cycle_t)c.st_completion_lag), rq_lat_((cycle_t)c.st_requant_latency),
      elems_per_read_(c.st_elems_per_acc_read) {}

void store_unit_t::config(uint64_t rs1, uint64_t rs2) {
  (void)rs1;
  stride_ = rs2 & 0xFFFFFFFFu;
}

span_t store_unit_t::src_span(const st_cmd_t &c) const {
  const uint32_t blocks = (c.cols + dim_ - 1) / dim_;
  if (c.src.is_acc()) return make_span(c.src, (uint64_t)blocks * (dim_ / 4), false);   // single-throughput tiles
  return make_span(c.src, (uint64_t)(blocks - 1) * dim_ + c.rows, false);
}

void store_unit_t::accept(uint64_t rs_id, st_cmd_t c) {
  const uint64_t stride = c.stride ? c.stride : stride_;
  q_.push_back({rs_id, c, stride});
  process();
}

void store_unit_t::process() {
  if (reading_ || q_.empty()) return;
  const item_t it = q_.front();
  q_.pop_front();
  rs_.kick(Q_ST);
  reading_ = true;
  const st_cmd_t &c = it.c;
  const uint32_t blocks = (c.cols + dim_ - 1) / dim_;
  const cycle_t now = eq_.now();
  double reads, row_bytes;
  port_t *rd;
  int prio;
  cycle_t data_from;
  if (c.src.is_acc()) {
    const bool full = (c.src.raw >> 29) & 1;
    reads = std::ceil((double)c.rows * c.cols / elems_per_read_);
    row_bytes = c.cols * (full ? 4.0 : c.out_bytes);
    rd = &acc_.port(c.src.row());
    prio = ACC_STORE;
    data_from = now + 1 + rq_lat_;
  } else {
    reads = (double)c.rows * blocks;   // one DIM-byte row per cycle
    row_bytes = c.cols;
    rd = &sp_.read_port(c.src.row());
    prio = SP_STORE;
    data_from = now + 1;
  }
  reads_ += (uint64_t)reads;
  const uint64_t id = it.id;
  rd->request(prio, now + 1, (cycle_t)reads, [this, id](cycle_t t) {
    eq_.at(t + lag_, [this, id] { rs_.complete(id); });
    reading_ = false;
    process();
  });
  if (c.kind == st_cmd_t::TO_DRAM) {
    dma_.submit(c.dram, c.rows, (uint32_t)std::ceil(row_bytes), it.stride, data_from, reads / c.rows, [](cycle_t) {});
  } else {
    const cycle_t beats = (cycle_t)std::ceil((double)c.rows * c.cols * c.out_bytes / dim_);
    sp_.write_port(c.dst.row()).request(SP_REQUANT, data_from, beats, [](cycle_t) {});
  }
}

}  // namespace gperf
