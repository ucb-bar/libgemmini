#ifndef GPERF_MODEL_H
#define GPERF_MODEL_H

#include <cstdint>
#include <cstdio>
#include <map>
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
#include "sim/types.h"
#include "host.h"

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

  void command(unsigned funct, uint64_t rs1, uint64_t rs2, uint64_t instret);   // a RoCC command retires
  void fence(uint64_t instret);                                                 // the core runs a fence
  cycle_t now(uint64_t instret) const { return host_.now(instret); }            // rdcycle

  void finish();                 // run until idle (end of program)
  void report(FILE *f) const;

private:
  void kick_at(cycle_t t);       // make sure the command stream steps at t
  void step();                   // one cycle of the command stream
  bool issue_loop(const loop_cmd_t &c);
  bool issue_raw(const rocc_cmd_t &c);
  uint64_t rs_alloc(rs_cmd_t c);

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

  uint32_t dim_;
  double out_bytes_ = 2.0;       // config_ex out_fmt, as the stream sees it
  cycle_t last_out_ = -1;        // one command per cycle leaves LoopMatmul
  cycle_t step_at_ = NEVER;
  uint64_t step_ev_ = 0;

  uint64_t cmds_ = 0, fences_ = 0;
  std::map<std::string, uint64_t> unmodelled_;
  FILE *trace_ = nullptr;
};

}  // namespace gperf

#endif
