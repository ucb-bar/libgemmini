#ifndef GPERF_HOST_CACHE_H
#define GPERF_HOST_CACHE_H

#include <cstddef>
#include <cstdint>

#ifdef GEMMINI_NO_SPIKE
#include <cstdio>
#include <cstdlib>

namespace gperf {

// Built without spike (radiance host): there is no CPU L1 to model here (mem.host_tracking 2 needs spike).
class host_cache_t {
public:
  enum result_t { HIT = 0, MISS = 1, MISS_WRITEBACK = 2 };
  host_cache_t(size_t, size_t, size_t line, const char *) : line_(line) {
    fprintf(stderr, "gemmini perf: mem.host_tracking 2 (the CPU's L1 caches) needs the spike build\n");
    abort();
  }
  result_t touch(uint64_t, bool) { return HIT; }
  bool holds(uint64_t, bool *dirty) { if (dirty) *dirty = false; return false; }
  void drop(uint64_t) {}
  void clean(uint64_t) {}
  size_t line_bytes() const { return line_; }
  uint64_t misses() const { return 0; }
  uint64_t writebacks() const { return 0; }

private:
  size_t line_;
};

}  // namespace gperf
#else
#include <riscv/cachesim.h>   // spike's cache model, used as installed (libriscv.so exports it)

namespace gperf {

// One of the CPU's L1 caches (mem.host_tracking = 2): spike's own cache_sim_t -- set-associative with an LFSR
// random replacer, like Rocket's -- driven by the CPU's traced fetches / loads / stores. It only says hit or miss;
// memory_system_t turns a miss into stall cycles, and Gemmini's probes read and change its contents.
class host_cache_t : public cache_sim_t {
public:
  enum result_t { HIT = 0, MISS = 1, MISS_WRITEBACK = 2 };   // MISS_WRITEBACK: the victim was dirty

  host_cache_t(size_t sets, size_t ways, size_t line, const char *name)
      : cache_sim_t(sets, ways, line, name), line_(line) {}
  ~host_cache_t() override {}

  // A CPU access to the line holding addr (a store dirties it; a miss allocates, write-allocate).
  result_t touch(uint64_t addr, bool store) {
    uint64_t *way = check_tag(addr);
    if (way) {
      if (store) *way |= DIRTY;
      return HIT;
    }
    store ? misses_w_++ : misses_r_++;
    const uint64_t victim = victimize(addr);
    if (store) *check_tag(addr) |= DIRTY;
    if ((victim & (VALID | DIRTY)) == (VALID | DIRTY)) { writebacks_++; return MISS_WRITEBACK; }
    return MISS;
  }
  // Does the L1 hold the line (and is it dirty, i.e. held with write permission)?
  bool holds(uint64_t addr, bool *dirty) {
    uint64_t *way = check_tag(addr);
    if (dirty) *dirty = way && (*way & DIRTY);
    return way != nullptr;
  }
  void drop(uint64_t addr) { if (uint64_t *w = check_tag(addr)) *w = 0; }            // probed to N
  void clean(uint64_t addr) { if (uint64_t *w = check_tag(addr)) *w &= ~DIRTY; }     // probed to B (data back)

  size_t line_bytes() const { return line_; }
  uint64_t misses() const { return misses_r_ + misses_w_; }
  uint64_t writebacks() const { return writebacks_; }

private:
  size_t line_;
  uint64_t misses_r_ = 0, misses_w_ = 0, writebacks_ = 0;
};

}  // namespace gperf
#endif  // GEMMINI_NO_SPIKE

#endif
