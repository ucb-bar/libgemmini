#ifndef GPERF_ACCUMULATOR_H
#define GPERF_ACCUMULATOR_H

#include <cstdint>
#include <deque>
#include <string>

#include "../params/config.h"
#include "../sim/event_queue.h"
#include "../sim/port.h"
#include "../sim/types.h"

namespace gperf {

// Accumulator banking: the double buffer. One port per bank; the mesh's accumulate writes always win, and a
// store's reads only use the cycles in between (AccumulatorMem.scala:667-672). So stores of loop n overlap loop
// n+1's compute exactly when the two use different banks -- what alternating the loop C bases buys.
enum acc_prio_t { ACC_MESH = 0, ACC_STORE = 1 };

class accumulator_t {
public:
  accumulator_t(const config_t &c, event_queue_t &eq)
      : banks_((uint32_t)c.acc_banks), bank_rows_((uint32_t)c.acc_bank_rows) {
    for (uint32_t b = 0; b < banks_; b++) port_.emplace_back(eq, "acc" + std::to_string(b));
  }

  uint32_t bank_of(uint32_t row) const { return (row / bank_rows_) % banks_; }
  port_t &port(uint32_t row) { return port_[bank_of(row)]; }
  uint32_t banks() const { return banks_; }
  const port_t &bank(uint32_t b) const { return port_[b]; }

private:
  uint32_t banks_, bank_rows_;
  std::deque<port_t> port_;
};

}  // namespace gperf

#endif
