#include "load_unit.h"

namespace gperf {

load_unit_t::load_unit_t(const config_t &c, event_queue_t &eq, dma_reader_t &dma, scratchpad_t &sp,
                         reservation_station_t &rs)
    : eq_(eq), dma_(dma), sp_(sp), rs_(rs), dim_((uint32_t)c.mesh_dim), queue_len_((size_t)c.ld_queue_length),
      max_running_((uint32_t)c.ld_cmds_in_flight), lag_((cycle_t)c.ld_completion_lag) {}

void load_unit_t::config(uint64_t rs1, uint64_t rs2) {
  const int id = (int)((rs1 >> 3) & 0x3) % 3;
  stride_[id] = rs2;
  block_stride_[id] = (uint32_t)((rs1 >> 16) & 0xFFFF);
}

span_t load_unit_t::dst_span(const mvin_t &m) const {
  const uint32_t blocks = (m.cols + dim_ - 1) / dim_;
  const uint32_t bs = block_stride_[m.state] ? block_stride_[m.state] : dim_;
  return make_span(m.dst, (uint64_t)(blocks - 1) * bs + m.rows, true);
}

void load_unit_t::accept(uint64_t rs_id, mvin_t m) {
  // the stride is the one in force when the command issues (CONFIG_LD issues in order with the mvins)
  waiting_.push_back({rs_id, m, m.stride ? m.stride : stride_[m.state]});
  try_start();
}

void load_unit_t::try_start() {
  while (!waiting_.empty() && running_ < max_running_) {
    const job_t j = waiting_.front();
    waiting_.pop_front();
    running_++;
    rs_.kick(Q_LD);   // queue room
    const uint32_t elem_bytes = j.m.dst.is_acc() ? 4 : 1;
    const uint32_t row_bytes = j.m.cols * elem_bytes;
    bytes_ += (uint64_t)j.m.rows * row_bytes;
    auto finish = [this, id = j.id](cycle_t t) {
      running_--;
      eq_.at(t + lag_, [this, id] { rs_.complete(id); });
      try_start();
    };
    if (j.m.dram == 0) {
      const uint32_t blocks = (j.m.cols + dim_ - 1) / dim_;
      sp_.write_port(j.m.dst.row()).request(SP_ZERO, eq_.now() + 1, (cycle_t)j.m.rows * blocks, finish);
    } else {
      dma_.submit(j.m.dram, j.m.rows, row_bytes, j.stride, j.m.dst.row(), finish);
    }
  }
}

}  // namespace gperf
