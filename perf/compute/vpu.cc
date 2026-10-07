#include "vpu.h"

#include <memory>

namespace gperf {

namespace {
enum { RMAX = 8, RSUM = 9, RAMAX = 10, MAXOP = 11, EXPSUB = 12, EXPSUM = 13 };
bool is_reduction(int op) { return op == RMAX || op == RSUM || op == RAMAX; }
}  // namespace

vpu_t::vpu_t(const config_t &c, event_queue_t &eq, scratchpad_t &sp, reservation_station_t &rs)
    : eq_(eq), sp_(sp), rs_(rs), latency_((cycle_t)c.vpu_latency),
      busy_((size_t)(c.vpu_units > 0 ? c.vpu_units : 1)) {}

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
  c.dst2 = (rs2 >> 16) & 0x3FFF;   // EXPSUM: row sums at imm[13:0]
  return c;
}

bool vpu_t::uses_src2(int op) { return op <= 2 || op == MAXOP || op == EXPSUB || op == EXPSUM; }
uint32_t vpu_t::groups(const cmd_t &c) { return c.rows / c.rlen; }
uint32_t vpu_t::dst_rows(const cmd_t &c) { return is_reduction(c.op) ? groups(c) : c.rows; }
uint32_t vpu_t::src2_rows(const cmd_t &c) { return c.bcast ? (c.rows + c.rlen - 1) / c.rlen : c.rows; }

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
  const bool src2 = uses_src2(c.op), sum = c.op == EXPSUM;
  auto left = std::make_shared<int>(2 + (src2 ? 1 : 0) + (sum ? 1 : 0));
  auto part = [this, u, rs_id, left](cycle_t t) {
    if (--*left > 0) return;
    eq_.at(t + 1, [this, u, rs_id] {
      busy_[u] = false;
      rs_.complete(rs_id);
      rs_.kick(Q_VEC);
    });
  };
  // issue: src1 one row per cycle (+ EXPSUM's empty slot per logical row); src2 alongside (same bank: same port)
  sp_.read_port(c.src1).request(SP_VPU, now + 1, (cycle_t)c.rows + (sum ? groups(c) : 0), part);
  if (src2) sp_.read_port(c.src2).request(SP_VPU, now + 1, src2_rows(c), part);
  sp_.write_port(c.dst).request(SP_VPU, now + 1 + latency_, dst_rows(c), part);
  if (sum) sp_.write_port(c.dst2).request(SP_VPU, now + 1 + latency_, groups(c), part);
}

}  // namespace gperf
