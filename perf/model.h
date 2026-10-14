#ifndef GPERF_MODEL_H
#define GPERF_MODEL_H

#include <cstdint>
#include <memory>
#include <cstdio>
#include <map>
#include <vector>
#include <string>

#include "compute/execute_unit.h"
#include "compute/spad_requant.h"
#include "compute/vpu.h"
#include "control/frontend.h"
#include "control/loop_matmul.h"
#include "control/reservation_station.h"
#include "lsu/dma.h"
#include "lsu/load_unit.h"
#include "lsu/lut_loader.h"
#include "lsu/scale_loader.h"
#include "lsu/store_unit.h"
#include "memory/accumulator.h"
#include "memory/memory_system.h"
#include "memory/scratchpad.h"
#include "params/config.h"
#include "sim/event_queue.h"
#include "sim/trace.h"
#include "sim/types.h"
#include "host.h"
#include "host/host_core.h"

namespace gperf {

// The whole accelerator, event driven. The host side (spike) calls command() / fence() at the core's current
// cycle; before anything happens at that cycle, the simulation runs every event before it. Inside, the command
// stream works one cycle at a time: the front end delivers commands to LoopMatmul; each cycle one command leaves
// LoopMatmul (a loop-unrolled one, or the front end's head once no loop is configured) into the reservation
// station or a Controller-side unit; the RS issues to the units; the units share the scratchpad / accumulator
// bank ports and the memory system through priority-arbitrated ports.
class model_t {
public:
  explicit model_t(const config_t &c);
  ~model_t();

  // A RoCC command retires. xd: it writes a register, so the core waits for Gemmini's response.
  void command(unsigned funct, uint64_t rs1, uint64_t rs2, uint64_t instret, bool xd = false);
  void fence(uint64_t instret);                                                 // the core runs a fence
  cycle_t now(uint64_t instret) const { return host_.now(instret); }
  cycle_t rdcycle(uint64_t instret);   // the program reads the cycle counter (marked in the trace)

  // The CPU's own memory traffic (spike memtracer): its L1 misses stall the CPU clock (mem.host_tracking 2).
  void host_store(uint64_t addr, uint32_t bytes) { const cycle_t s = mem_.host_access(addr, bytes, true); host_.add_stall(s); host_data_stall(s); }
  void host_load(uint64_t addr, uint32_t bytes) { const cycle_t s = mem_.host_access(addr, bytes, false); host_.add_stall(s); host_data_stall(s); }
  // An executed instruction (fetch trace): its I$ miss, then the pipeline model (host.core_model).
  void host_fetch(uint64_t addr, uint32_t bytes, uint32_t bits) {
    const cycle_t m = mem_.host_fetch(addr, bytes);
    host_.add_stall(m);
    if (core_) {
      core_->external_stall(m);
      core_frac_ += core_->insn(addr, bits);
      const cycle_t whole = (cycle_t)core_frac_;
      if (whole > 0) { host_.add_stall(whole); core_frac_ -= (double)whole; }
    }
  }
  void host_data_stall(cycle_t c) { if (core_) core_->external_stall(c); }
  bool host_tracking() const { return cfg_.mem_host_tracking != 0; }
  int host_tracking_level() const { return (int)cfg_.mem_host_tracking; }
  bool host_icache() const { return cfg_.host_icache != 0; }

  // ---- host.kind 1 (radiance, gemmini_radiance.cc): the host's clock; commands are MMIO register writes ----
  // Every call names the host's current cycle t (non-decreasing); everything before t has happened first, so a
  // read sees the registers as the cycle begins.
  void advance_to(cycle_t t) { if (t > eq_.now()) eq_.advance_to(t); }
  // The INST register write at t: false while the command path is full (RTL io.cmd.ready low -- the store waits and
  // is offered again later); true: taken at t.
  bool offer(cycle_t t, unsigned funct, uint64_t rs1, uint64_t rs2);
  bool cmd_ready(cycle_t t) { advance_to(t); return !front_.full(); }          // READY (+0x08)
  bool busy(cycle_t t) { advance_to(t); return !idle() || busy_until() >= t; }   // BUSY (+0x20): io.busy
  uint64_t loops_pending(cycle_t t) { advance_to(t); return loops_accepted_ - loops_.loops_unrolled(); }   // +0x28
  uint64_t loops_retired(cycle_t t);   // RETIRED (+0x38): LOOP_WS whose every command completed, in issue order
  cycle_t next_event() const { return eq_.next_time(); }   // the host may skip to here while it has nothing to say

  void finish();                 // run until idle (end of program)
  cycle_t busy_until() const;    // when Gemmini's busy signal last dropped
  void check_stuck(const char *where);
  bool stuck_reported_ = false;
  bool passing_ = false;         // the raw command being issued is bypassing configured loops
  bool idle() const { return front_.empty() && loops_.idle() && rs_.idle() && scales_.idle() && !luts_.busy(); }
  void report(FILE *f) const;

private:
  void kick_at(cycle_t t);       // make sure the command stream steps at t
  void step();                   // one cycle of the command stream
  bool issue_loop(const loop_cmd_t &c);
  bool issue_raw(const rocc_cmd_t &c);
  bool may_pass(const rocc_cmd_t &c) const;
  uint64_t rs_alloc(rs_cmd_t c);
  uint32_t bank_mask(const span_t &s) const;
  span_t preload_c_span(local_addr_t c) const;

  config_t cfg_;
  event_queue_t eq_;
  memory_system_t mem_;
  scratchpad_t spad_;
  accumulator_t acc_;
  dma_reader_t reader_;
  dma_writer_t writer_;
  reservation_station_t rs_;
  load_unit_t load_;
  execute_unit_t exec_;
  store_unit_t store_;
  scale_loader_t scales_;
  lut_loader_t luts_;
  vpu_t vpu_;
  spad_requant_t sreq_;
  loop_matmul_t loops_;
  frontend_t front_;
  host_t host_;
  std::unique_ptr<host_core_t> core_;   // mem.host_tracking 2 + host.icache + host.core_model
  double core_frac_ = 0;

  uint32_t dim_;
  double out_bytes_ = 2.0;       // config_ex out_fmt, as the stream sees it
  bool mx_multi_ = false, mx_multi_act_ = false;   // config_ex: quad weight / activation (FP4/FP6/E5M2)
  bool cfg_reads_act0_ = true;   // the last raw CONFIG_SCALE_MEM may read act scale half 0
  cycle_t last_out_ = -1;        // one command per cycle leaves LoopMatmul
  cycle_t step_at_ = NEVER;
  uint64_t step_ev_ = 0;

  uint64_t cmds_ = 0, fences_ = 0;
  // LOOP_WS accounting for the radiance registers: accepted, and per loop its RS entries not yet completed
  int64_t loop_serial_ = -1;     // the loop whose command issue_loop is issuing (stamped on its RS entries)
  uint64_t loops_accepted_ = 0, loops_retired_ = 0;
  std::map<uint64_t, uint64_t> loop_out_;
  struct fence_cmp_t { cycle_t rtl, model; };   // replay: RTL fence retire vs model idle, for fences that waited
  std::vector<fence_cmp_t> fence_cmp_;
  uint64_t cmds_at_fence_ = 0;
  uint64_t taken_ = 0;           // commands LoopMatmul has taken from the front end
  cycle_t last_taken_ = 0;       // ... and when the latest one was taken
  std::map<std::string, uint64_t> unmodelled_;
  FILE *trace_ = nullptr;
};

}  // namespace gperf

#endif
