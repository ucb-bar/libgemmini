#ifndef GPERF_LUT_LOADER_H
#define GPERF_LUT_LOADER_H

#include <cstdint>

#include "../params/config.h"
#include "../sim/event_queue.h"
#include "../memory/memory_system.h"
#include "../sim/types.h"

namespace gperf {

// MX_LOAD_LUT loader (Controller.scala:723-833): one TileLink source, one 8-byte Get at a time
// (sReq -> sResp -> sReq), then lut.fixed_cycles. While it runs, the command stream behind it waits.
class lut_loader_t {
public:
  lut_loader_t(const config_t &c, event_queue_t &eq, memory_system_t &mem)
      : eq_(eq), mem_(mem), fixed_((cycle_t)c.lut_fixed_cycles) {}

  bool busy() const { return busy_; }
  // MX_LOAD_LUT rs1 = DRAM address, rs2 = num tables [31:0] | entry bits [39:34] (0 = 6).
  void load(uint64_t addr, uint32_t tables, uint32_t entry_bits, done_t done) {
    if (entry_bits == 0) entry_bits = 6;
    const uint64_t bytes = ((uint64_t)tables * 16 * entry_bits + 7) / 8;
    busy_ = true;
    next(addr, (bytes + 7) / 8, std::move(done));
  }

private:
  void next(uint64_t addr, uint64_t words_left, done_t done) {
    if (words_left == 0) {
      const cycle_t t = eq_.now() + fixed_;
      eq_.at(t, [this, done, t] { busy_ = false; done(t); });
      return;
    }
    mem_.read(addr, [this, addr, words_left, done](cycle_t t) {
      eq_.at(t + 1, [this, addr, words_left, done] { next(addr + 8, words_left - 1, done); });
    });
  }

  event_queue_t &eq_;
  memory_system_t &mem_;
  cycle_t fixed_;
  bool busy_ = false;
};

}  // namespace gperf

#endif
