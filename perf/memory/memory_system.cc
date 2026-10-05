#include "memory_system.h"

#include <cmath>

namespace gperf {

memory_system_t::memory_system_t(const config_t &c, event_queue_t &eq)
    : eq_(eq), bus_(eq, "bus"), dram_(eq, "dram"),
      line_bytes_((uint32_t)c.mem_line_bytes),
      capacity_lines_((size_t)(c.mem_l2_kib * 1024 / c.mem_line_bytes)),
      hit_lat_((cycle_t)c.mem_l2_hit_latency), dram_lat_((cycle_t)c.mem_dram_latency),
      ack_lat_((cycle_t)c.mem_write_ack_latency), put_cycles_((cycle_t)c.mem_put_cycles),
      dram_cycles_per_line_((cycle_t)std::ceil(c.mem_line_bytes / c.mem_dram_bytes_per_cycle)) {}

void memory_system_t::insert(uint64_t line, cycle_t ready) {
  lru_.push_front(line);
  line_t &l = lines_[line];
  l.pos = lru_.begin();
  l.ready = ready;
  if (lines_.size() > capacity_lines_) {
    const uint64_t victim = lru_.back();
    if (lines_[victim].ready != NEVER) {   // never evict a line with a fill outstanding
      lines_.erase(victim);
      lru_.pop_back();
    }
  }
}

void memory_system_t::lookup(uint64_t line, cycle_t t, done_t back) {
  auto it = lines_.find(line);
  if (it != lines_.end()) {
    hits_++;
    lru_.splice(lru_.begin(), lru_, it->second.pos);
    if (it->second.ready == NEVER) {   // a fill is outstanding: wait for it
      it->second.waiters.push_back({std::move(back), t + hit_lat_});
      return;
    }
    const cycle_t r = cmax(t + hit_lat_, it->second.ready);
    eq_.at(r, [back, r] { back(r); });
    return;
  }
  misses_++;
  insert(line, NEVER);
  lines_[line].waiters.push_back({std::move(back), t + hit_lat_});
  dram_.request(0, t, dram_cycles_per_line_, [this, line](cycle_t d) {
    const cycle_t r = d + dram_lat_;
    auto jt = lines_.find(line);
    if (jt == lines_.end()) return;
    jt->second.ready = r;
    auto waiters = std::move(jt->second.waiters);
    jt->second.waiters.clear();
    for (auto &w : waiters) {
      const cycle_t at = cmax(r, w.second);
      done_t cb = std::move(w.first);
      eq_.at(at, [cb, at] { cb(at); });
    }
  });
}

void memory_system_t::read(uint64_t addr, done_t back) {
  const uint64_t line = addr / line_bytes_;
  bus_.request(0, eq_.now(), 1, [this, line, back](cycle_t g) { lookup(line, g, back); });
}

// The L2 takes the write (write-allocate, no stall for a partial line); the ack comes back after ack_lat_.
void memory_system_t::write(uint64_t addr, uint32_t bytes, done_t acked) {
  (void)bytes;
  const uint64_t line = addr / line_bytes_;
  bus_.request(0, eq_.now(), put_cycles_, [this, line, acked](cycle_t g) {
    auto it = lines_.find(line);
    if (it == lines_.end()) insert(line, g + ack_lat_);
    else lru_.splice(lru_.begin(), lru_, it->second.pos);
    const cycle_t a = g + ack_lat_;
    eq_.at(a, [acked, a] { acked(a); });
  });
}

}  // namespace gperf
