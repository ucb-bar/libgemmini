#ifndef GPERF_SCRATCHPAD_H
#define GPERF_SCRATCHPAD_H

#include <cstdint>
#include <deque>
#include <string>

#include "../params/config.h"
#include "../sim/event_queue.h"
#include "../sim/port.h"
#include "../sim/types.h"

namespace gperf {

// Scratchpad banking (Scratchpad.scala). Each bank has one read port and one write port (sp_singleported =
// false); one DIM-byte row per cycle each. Requesters arbitrate with the RTL's fixed priorities:
//   write: VPU > ex > mvin > zero-writer > requant > SPAD_REQUANT   (Scratchpad.scala:979-1060)
//   read:  VPU > mesh (ex) > store (mvout from spad) > SPAD_REQUANT
// Two mesh operands in the same bank therefore serialise, and a VPU stream stalls the mesh on its banks.
enum spad_prio_t { SP_VPU = 0, SP_EX = 1, SP_MVIN = 2, SP_ZERO = 3, SP_STORE = 3, SP_REQUANT = 4, SP_SREQ = 5 };

class scratchpad_t {
public:
  scratchpad_t(const config_t &c, event_queue_t &eq)
      : banks_((uint32_t)c.spad_banks), bank_rows_((uint32_t)c.spad_bank_rows) {
    for (uint32_t b = 0; b < banks_; b++) {
      rd_.emplace_back(eq, "spad_rd" + std::to_string(b));
      wr_.emplace_back(eq, "spad_wr" + std::to_string(b));
    }
  }

  uint32_t bank_of(uint32_t row) const { return (row / bank_rows_) % banks_; }
  port_t &read_port(uint32_t row) { return rd_[bank_of(row)]; }
  port_t &write_port(uint32_t row) { return wr_[bank_of(row)]; }
  uint32_t banks() const { return banks_; }
  const port_t &read_bank(uint32_t b) const { return rd_[b]; }
  const port_t &write_bank(uint32_t b) const { return wr_[b]; }

private:
  uint32_t banks_, bank_rows_;
  std::deque<port_t> rd_, wr_;   // deque: ports are never moved once events refer to them
};

}  // namespace gperf

#endif
