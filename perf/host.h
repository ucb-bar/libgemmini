#ifndef GPERF_HOST_H
#define GPERF_HOST_H

#include <cstdint>

#include "params/config.h"
#include "sim/types.h"

namespace gperf {

// The core's clock, as the model sees it: retired instructions x host.cpi, plus the cycles it spent stalled on
// Gemmini (a full command queue, a fence). A placeholder for a real Rocket model (perf_model_plan.md section 9).
class host_t {
public:
  explicit host_t(const config_t &c) : cpi_(c.host_cpi) {}

  cycle_t now(uint64_t instret) const { return (cycle_t)(instret * cpi_) + stall_; }
  // The core cannot proceed before t (its next instruction retires at t).
  void wait_until(uint64_t instret, cycle_t t) {
    const cycle_t n = now(instret);
    if (t > n) stall_ += t - n;
  }
  cycle_t stall_cycles() const { return stall_; }

private:
  double cpi_;
  cycle_t stall_ = 0;
};

}  // namespace gperf

#endif
