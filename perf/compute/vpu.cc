#include "vpu.h"

#include <memory>

namespace gperf {

namespace {
bool is_reduction(int op) { return op == 8 || op == 9 || op == 10; }   // RMAX, RSUM, RAMAX
bool uses_src2(int op) { return op <= 2 || op == 11 || op == 12; }     // ADD/SUB/MUL, MAX, EXPSUB
}  // namespace

vpu_t::vpu_t(const config_t &c, event_queue_t &eq, scratchpad_t &sp, reservation_station_t &rs)
    : eq_(eq), sp_(sp), rs_(rs), latency_((cycle_t)c.vpu_latency), busy_((size_t)(c.vpu_units > 0 ? c.vpu_units : 1)) {}

vpu_t::cmd_t vpu_t::decode(uint64_t rs1, uint64_t rs2) {
  cmd_t c;
  c.src1 = rs1 & 0x3FFF;
  c.src2 = (rs1 >> 14) & 0x3FFF;
  c.dst = (rs1 >> 28) & 0x3FFF;
  c.rows = (rs1 >> 42) & 0xFFFF;
  c.op = (int)(rs2 & 0xF);
  c.bcast = (rs2 >> 4) & 1;
  c.rlen = (rs2 >> 5) & 0x3FF;
  if (c.rlen == 0) c.rlen = 1024;
  return c;
}

span_t vpu_t::read_span(const cmd_t &c) { return make_span(local_addr_t{c.src1}, c.rows, false); }
span_t vpu_t::write_span(const cmd_t &c) {
  return make_span(local_addr_t{c.dst}, is_reduction(c.op) ? c.rows / c.rlen : c.rows, true);
}

bool vpu_t::has_room() const {
  for (bool b : busy_) if (!b) return true;
  return false;
}

void vpu_t::accept(uint64_t rs_id, cmd_t c) {
  size_t u = 0;
  while (u < busy_.size() && busy_[u]) u++;
  if (u == busy_.size()) u = 0;
  busy_[u] = true;
  cmds_++;
  const cycle_t now = eq_.now();
  const uint32_t out_rows = is_reduction(c.op) ? c.rows / c.rlen : c.rows;
  const bool src2 = uses_src2(c.op);
  auto left = std::make_shared<int>(src2 ? 3 : 2);
  auto part = [this, u, rs_id, left](cycle_t t) {
    if (--*left > 0) return;
    busy_[u] = false;
    eq_.at(t + 1, [this, rs_id] { rs_.complete(rs_id); });
    rs_.kick(Q_VEC);
  };
  sp_.read_port(c.src1).request(SP_VPU, now + 1, c.rows, part);
  if (src2) sp_.read_port(c.src2).request(SP_VPU, now + 1, c.bcast ? (c.rows + c.rlen - 1) / c.rlen : c.rows, part);
  sp_.write_port(c.dst).request(SP_VPU, now + 1 + latency_, out_rows, part);
}

}  // namespace gperf
