#ifndef GPERF_HOST_H
#define GPERF_HOST_H

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

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
  void add_stall(cycle_t c) { stall_ += c; }   // the core stalled c cycles (an L1 miss, mem.host_tracking 2)

  // Replay (GEMMINI_PERF_REPLAY=<file from tools/rtl_replay.py>): the RTL's retire cycle of each Gemmini command,
  // fence and rdcycle, in program order, replaces the host estimate -- the accelerator is then compared alone.
  enum replay_kind_t { R_ROCC = 0, R_FENCE = 1, R_RDCYCLE = 2 };
  bool load_replay(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return false;
    char kind[16];
    long long c;
    uint64_t roccs = 0;
    while (fscanf(f, "%15s %lld", kind, &c) == 2) {
      const int k = !strcmp(kind, "rocc") ? R_ROCC : !strcmp(kind, "fence") ? R_FENCE : R_RDCYCLE;
      replay_[k].push_back((cycle_t)c);
      if (k == R_ROCC) roccs++;
      if (k == R_FENCE) fence_after_.push_back(roccs);
    }
    fclose(f);
    replaying_ = true;
    return true;
  }
  bool replaying() const { return replaying_; }
  // The next replayed cycle of this kind; -1 when the program has more of them than the RTL run did.
  cycle_t replay_next(replay_kind_t k) {
    return next_[k] < replay_[k].size() ? replay_[k][next_[k]++] : -1;
  }
  // Fences differ between spike and the RTL run (boot / printf paths), so a fence is matched by the number of
  // Gemmini commands retired before it: the next RTL fence that follows exactly `roccs` commands, or -1.
  cycle_t replay_fence(uint64_t roccs) {
    for (size_t i = next_[R_FENCE]; i < fence_after_.size(); i++) {
      if (fence_after_[i] > roccs) return -1;
      if (fence_after_[i] == roccs) { next_[R_FENCE] = i + 1; return replay_[R_FENCE][i]; }
    }
    return -1;
  }

private:
  double cpi_;
  cycle_t stall_ = 0;
  bool replaying_ = false;
  std::vector<cycle_t> replay_[3];
  size_t next_[3] = {0, 0, 0};
  std::vector<uint64_t> fence_after_;   // per RTL fence: Gemmini commands retired before it
};

}  // namespace gperf

#endif
