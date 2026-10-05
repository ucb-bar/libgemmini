#ifndef GPERF_PORT_H
#define GPERF_PORT_H

#include <cstdint>
#include <string>
#include <vector>

#include "event_queue.h"
#include "types.h"

namespace gperf {

// A resource that serves one requester per cycle: an SRAM bank port, a bus channel, the DRAM channel.
// A requester asks for `cycles` cycles at a priority (lower wins), not before `earliest`; each cycle goes to the
// best eligible demand (oldest first on a tie), and a better one preempts a running one the cycle it becomes
// eligible. Cycles are handed out only as simulated time reaches them, so a demand that arrives later -- with an
// `earliest` in the future -- still preempts correctly: that is what makes the overlap of shared units exact.
// done(t) fires at the cycle the last granted cycle ends.
class port_t {
public:
  port_t(event_queue_t &eq, std::string name) : eq_(eq), name_(std::move(name)) {}
  port_t(port_t &&) = default;

  void request(int prio, cycle_t earliest, cycle_t cycles, done_t done);

  uint64_t busy_cycles() const { return busy_; }
  const std::string &name() const { return name_; }

private:
  struct demand_t { int prio; cycle_t earliest; cycle_t left; uint64_t seq; done_t done; };

  void advance(cycle_t t);   // hand out the cycles [cursor_, t)
  void reschedule();         // wake at the next completion
  static int pick(const std::vector<demand_t> &d, const std::vector<cycle_t> &left, cycle_t c);
  static cycle_t next_event(const std::vector<demand_t> &d, const std::vector<cycle_t> &left, cycle_t c, int prio);

  event_queue_t &eq_;
  std::string name_;
  std::vector<demand_t> pend_;
  cycle_t cursor_ = 0;
  uint64_t seq_ = 0, wake_ = 0, busy_ = 0;
};

}  // namespace gperf

#endif
