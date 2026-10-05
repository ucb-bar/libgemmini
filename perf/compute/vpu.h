#ifndef GPERF_VPU_H
#define GPERF_VPU_H

#include <cstdint>
#include <vector>

#include "../params/config.h"
#include "../sim/event_queue.h"
#include "../control/reservation_station.h"
#include "../memory/scratchpad.h"
#include "../sim/types.h"

namespace gperf {

// VPU (vpu/Vpu.scala): vpu.units BF16 vector engines on the scratchpad, first free unit takes a command.
// A command streams `rows` src1 rows (and src2 rows, or one src2 row per logical row when broadcast) through the
// bank read ports at top priority -- src1 and src2 in one bank share a port, the +1 cycle per row the RTL pays --
// and writes its result rows (one per logical row for a reduction) vpu.latency later through the write port, also
// at top priority. So a VPU stream stalls the mesh and the DMA on the banks it touches.
// Not calibrated yet (perf_model_plan.md step 5).
class vpu_t {
public:
  vpu_t(const config_t &c, event_queue_t &eq, scratchpad_t &sp, reservation_station_t &rs);

  struct cmd_t { uint32_t src1, src2, dst, rows, rlen; int op; bool bcast; };
  static cmd_t decode(uint64_t rs1, uint64_t rs2);
  static span_t read_span(const cmd_t &c);
  static span_t write_span(const cmd_t &c);

  bool has_room() const;
  void accept(uint64_t rs_id, cmd_t c);

  uint64_t commands() const { return cmds_; }

private:
  event_queue_t &eq_;
  scratchpad_t &sp_;
  reservation_station_t &rs_;
  cycle_t latency_;
  std::vector<bool> busy_;
  uint64_t cmds_ = 0;
};

}  // namespace gperf

#endif
