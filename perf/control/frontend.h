#ifndef GPERF_FRONTEND_H
#define GPERF_FRONTEND_H

#include <cstdint>
#include <deque>

#include "../params/config.h"
#include "../sim/types.h"

namespace gperf {

// The RoCC command path from the core to LoopMatmul: frontend.depth commands of buffering, frontend.latency
// cycles end to end. The core stalls while it is full. LoopMatmul takes the head once it has arrived.
struct rocc_cmd_t { unsigned funct; uint64_t rs1, rs2; cycle_t arrival; };

class frontend_t {
public:
  explicit frontend_t(const config_t &c) : depth_((size_t)c.frontend_depth), latency_((cycle_t)c.frontend_latency) {}

  bool full() const { return q_.size() >= depth_; }
  // Taken from the core at `t`; it reaches LoopMatmul at the returned cycle.
  cycle_t push(unsigned funct, uint64_t rs1, uint64_t rs2, cycle_t t) {
    q_.push_back({funct, rs1, rs2, t + latency_});
    return t + latency_;
  }
  bool head_ready(cycle_t now) const { return !q_.empty() && q_.front().arrival <= now; }
  const rocc_cmd_t &head() const { return q_.front(); }
  cycle_t head_arrival() const { return q_.empty() ? NEVER : q_.front().arrival; }
  void pop() { q_.pop_front(); }
  bool empty() const { return q_.empty(); }

private:
  size_t depth_;
  cycle_t latency_;
  std::deque<rocc_cmd_t> q_;
};

}  // namespace gperf

#endif
