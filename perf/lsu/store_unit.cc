#include "store_unit.h"

#include <cmath>
#include <memory>

namespace gperf {

store_unit_t::store_unit_t(const config_t &c, event_queue_t &eq, scratchpad_t &sp, accumulator_t &acc,
                           dma_writer_t &dma, reservation_station_t &rs)
    : eq_(eq), sp_(sp), acc_(acc), dma_(dma), rs_(rs), queue_len_((size_t)c.st_queue_length),
      dim_((uint32_t)c.mesh_dim), lag_((cycle_t)c.st_completion_lag), rq_lat_((cycle_t)c.st_requant_latency),
      pipe_lat_((cycle_t)c.st_pipe_latency), slack_((cycle_t)c.st_write_slack),
      elems_per_read_(c.st_elems_per_acc_read) {}

void store_unit_t::config(uint64_t rs1, uint64_t rs2) {
  (void)rs1;
  stride_ = rs2 & 0xFFFFFFFFu;
}

span_t store_unit_t::src_span(const st_cmd_t &c) const {
  const uint32_t blocks = (c.cols + dim_ - 1) / dim_;
  // the RS's mvout range for a raw mvout, (blocks-1)*DIM + rows from its rs2 (ReservationStation.scala:286-295).
  // Loop-issued C stores carry their own range (loop_cmd_t::rs_span). The RS's store/mesh stalls come from the
  // preloads' DIM-row C range spilling over the packed DIM/4-row tiles (13.5).
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
    data_from = now + pipe_lat_ + rq_lat_;
  } else {
    reads = (double)c.rows * blocks;   // one DIM-byte row per cycle
    row_bytes = c.cols;
    rd = &sp_.read_port(c.src.row());
    prio = SP_STORE;
    data_from = now + pipe_lat_;
  }
  reads_ += (uint64_t)reads;
  const uint64_t id = it.id;
  // The command is done (RS completion, next store may start) when its reads are done and -- for a store to DRAM,
  // whose short write queues back-pressure the reads -- its Puts have all but write_slack been issued.
  const bool to_dram = c.kind == st_cmd_t::TO_DRAM;
  auto left = std::make_shared<int>(to_dram ? 2 : 1);
  auto latest = std::make_shared<cycle_t>(now);
  auto part = [this, id, left, latest](cycle_t t) {
    *latest = cmax(*latest, t);
    if (--*left > 0) return;
    const cycle_t done = *latest;
    eq_.at(done + lag_, [this, id] { rs_.complete(id); });
    eq_.at(done, [this] { reading_ = false; process(); });
  };
  rd->request(prio, now + 1, (cycle_t)reads, part);
  if (to_dram) {
    dma_.submit(c.dram, c.rows, (uint32_t)std::ceil(row_bytes), it.stride, data_from, reads / c.rows, [](cycle_t) {},
                [part](cycle_t t) { part(t); }, (uint32_t)slack_);
  } else {
    const cycle_t beats = (cycle_t)std::ceil((double)c.rows * c.cols * c.out_bytes / dim_);
    sp_.write_port(c.dst.row()).request(SP_REQUANT, data_from, beats, [this](cycle_t t) { last_write_ = cmax(last_write_, t); });
  }
}

}  // namespace gperf
