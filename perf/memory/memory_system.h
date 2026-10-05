#ifndef GPERF_MEMORY_SYSTEM_H
#define GPERF_MEMORY_SYSTEM_H

#include <cstdint>
#include <list>
#include <unordered_map>
#include <vector>

#include "../params/config.h"
#include "../sim/event_queue.h"
#include "../sim/port.h"
#include "../sim/types.h"

namespace gperf {

// System bus + L2 + DRAM, one TileLink request at a time. Every requester (DMA reader and writer, scale loader,
// LUT loader) shares the bus request channel in time order: a Get takes one cycle of it, a Put mem.put_cycles
// (the L2 accepts a Put only every other cycle -- measured); an L2 miss then queues on the DRAM channel. The L2 is an LRU set of lines, cold at start; a line being filled makes later hits wait for
// the fill. The host's own accesses are not seen.
class memory_system_t {
public:
  memory_system_t(const config_t &c, event_queue_t &eq);

  void read(uint64_t addr, done_t data_back);                    // a Get issued now
  void write(uint64_t addr, uint32_t bytes, done_t acked);       // a Put issued now

  uint64_t hits() const { return hits_; }
  uint64_t misses() const { return misses_; }
  uint64_t bus_busy() const { return bus_.busy_cycles(); }
  uint64_t dram_busy() const { return dram_.busy_cycles(); }

private:
  struct line_t {
    std::list<uint64_t>::iterator pos;
    cycle_t ready;                                        // NEVER while its fill is outstanding
    std::vector<std::pair<done_t, cycle_t>> waiters;      // hits waiting on the fill: (callback, hit time)
  };
  void lookup(uint64_t line, cycle_t t, done_t back);
  void insert(uint64_t line, cycle_t ready);

  event_queue_t &eq_;
  port_t bus_, dram_;
  uint32_t line_bytes_;
  size_t capacity_lines_;
  cycle_t hit_lat_, dram_lat_, ack_lat_, put_cycles_, dram_cycles_per_line_;
  std::list<uint64_t> lru_;   // front = most recent
  std::unordered_map<uint64_t, line_t> lines_;
  uint64_t hits_ = 0, misses_ = 0;
};

}  // namespace gperf

#endif
