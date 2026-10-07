#ifndef GPERF_MEMORY_SYSTEM_H
#define GPERF_MEMORY_SYSTEM_H

#include <cstdint>
#include <deque>
#include <list>
#include <unordered_map>
#include <vector>

#include "../host/host_cache.h"
#include "../params/config.h"
#include "../sim/event_queue.h"
#include "../sim/port.h"
#include "../sim/trace.h"
#include "../sim/types.h"

namespace gperf {

// System bus + L2 + DRAM + the CPU's L1 as seen by probes. Every requester (DMA reader and writer, scale loader,
// LUT loader) shares the bus request channel in time order: a Get takes one cycle of it, a Put (always a line-sized
// message, byte-masked when partial; DMA.scala:426-430) line / mem.bus_bytes cycles.
// L2 (InclusiveCache; its 42 MSHRs never limit these kernels): an LRU set of lines, cold at start. A miss queues on
// the DRAM channel (one line per line/bandwidth cycles) and later requests to that line wait for the fill. Puts to
// the same line are handled one at a time (mem.l2_put_serial_cycles); a partial Put to a line the L2 lacks waits for
// its fill (write-allocate). A line the CPU's L1 holds is probed out first: the L1 takes one probe per
// mem.probe_cycles, so a burst of Puts to CPU-written lines queues on it (dramloop FSDB).
class memory_system_t {
public:
  memory_system_t(const config_t &c, event_queue_t &eq);

  // A Get issued now. who: trace tag; own_client: the MX scale / LUT loaders' own TileLink client, which does not
  // share the DMA's crossbar port (measured: their Gets never appear on it), only the L2 and DRAM.
  // granted(g), if given: the bus accepted the request at g (a TileLink client holds a request until then).
  void read(uint64_t addr, done_t data_back, const char *who = "get", bool own_client = false,
            done_t granted = nullptr);
  void write(uint64_t addr, uint32_t bytes, done_t acked, done_t granted = nullptr);   // a Put issued now

  // The CPU stored to (write) or loaded from [addr, addr+bytes): those lines are now in its L1 (and present in the
  // inclusive L2; dirty there when written). Returns the core's stall cycles (mem.host_tracking 2), else 0.
  cycle_t host_access(uint64_t addr, uint32_t bytes, bool write);
  cycle_t host_fetch(uint64_t addr, uint32_t bytes);   // an instruction fetch (host_tracking 2 + host.icache)
  uint64_t host_l1_misses() const { return (l1d_ ? l1d_->misses() : 0) + (l1i_ ? l1i_->misses() : 0); }
  uint64_t host_l1_writebacks() const { return l1d_ ? l1d_->writebacks() : 0; }
  cycle_t host_stall() const { return host_stall_; }

  uint64_t hits() const { return hits_; }
  uint64_t probes() const { return probes_; }
  uint64_t host_stores() const { return host_stores_; }
  uint64_t writebacks() const { return writebacks_; }
  cycle_t last_ack() const { return last_ack_; }   // Gemmini-visible: the latest Put ack (fills / write-backs are not)
  uint64_t write_fills() const { return write_fills_; }
  uint64_t misses() const { return misses_; }
  uint64_t bus_busy() const { return bus_.busy_cycles(); }
  uint64_t dram_busy() const { return dram_.busy_cycles(); }

private:
  struct line_t {
    std::list<uint64_t>::iterator pos;
    cycle_t ready;                                        // NEVER while its fill is outstanding
    bool dirty = false;                                   // written (by Gemmini or the CPU): evicting it costs DRAM
    std::vector<done_t> waiters;                          // requests waiting on the fill: resumed at its landing
    cycle_t put_free = 0;                                 // the next Put to this line may start here
  };
  void lookup(uint64_t line, cycle_t t, done_t back, cycle_t hit_lat);
  void insert(uint64_t line, cycle_t ready, bool dirty = false);
  void fetch(uint64_t line, cycle_t t);   // the L2 fills `line` from DRAM; its waiters resume when it lands
  // A Put's turn on its line: not before `earliest` nor the line's previous Put; acked `post` cycles later.
  void put_op(uint64_t line, cycle_t earliest, cycle_t post, done_t acked);
  void ack(cycle_t a, const done_t &acked);
  // If the CPU's L1 holds `line`, probe it out: done(t) with t = when the L2 has it back (probe slot queueing +
  // mem.probe_latency); else done(g) at once.
  void probe(uint64_t line, cycle_t g, bool write, done_t done);
  // mem.host_tracking 2: the CPU's L1 caches (spike's cache_sim_t; never deleted -- its destructor prints stats)
  host_cache_t *l1d_ = nullptr, *l1i_ = nullptr;
  cycle_t pen_l2_ = 0, pen_dram_ = 0, pen_wb_ = 0, host_stall_ = 0;
  cycle_t host_miss(uint64_t line, host_cache_t::result_t r, bool write);   // stall of one L1 miss

  event_queue_t &eq_;
  port_t bus_, client_bus_, dram_, l1_probe_;
  uint32_t line_bytes_;
  size_t capacity_lines_;
  cycle_t hit_lat_, client_hit_lat_, dram_lat_, ack_lat_, full_ack_lat_, put_beats_, dram_cycles_per_line_,
      put_serial_, fill_secondary_;
  std::list<uint64_t> lru_;   // front = most recent
  std::unordered_map<uint64_t, line_t> lines_;
  cycle_t last_ack_ = 0;
  uint64_t hits_ = 0, misses_ = 0, probes_ = 0, host_stores_ = 0, writebacks_ = 0, write_fills_ = 0;
  // the CPU's L1 D$: lines it has written and still owns; set-associative, LRU within a set
  bool take_from_l1(uint64_t line);   // Gemmini touches `line`: true if the L1 owned it (now probed out)
  // DRAM banks (mem.dram_model = 1): open row and when the bank may activate again
  bool dram_banked_;
  uint32_t dram_max_reads_ = 0, dram_reads_ = 0;            // FASED-style cap on reads in flight
  std::deque<std::pair<uint64_t, cycle_t>> dram_waiting_;   // fills waiting for a read slot: (line, since)
  void start_fetch(uint64_t line, cycle_t t);
  uint32_t dram_banks_, dram_row_lines_;
  cycle_t row_miss_, t_rc_;
  std::vector<uint64_t> open_row_;
  std::vector<cycle_t> bank_act_free_;
  cycle_t dram_ready(uint64_t line, cycle_t d);   // when a line whose channel slot ended at d is back
  uint32_t l1_sets_, l1_ways_;
  bool l1_random_;
  uint64_t lfsr_ = 0x9E3779B97F4A7C15ull;   // the random replacer (statistically like Rocket's, not cycle-exact)
  cycle_t probe_issue_, probe_cycles_, probe_lat_, probe_put_ack_;
  std::vector<std::list<uint64_t>> l1_lru_;   // per set, front = most recent
  std::unordered_map<uint64_t, std::list<uint64_t>::iterator> l1_;
};

}  // namespace gperf

#endif
