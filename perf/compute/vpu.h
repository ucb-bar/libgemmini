#ifndef GPERF_VPU_H
#define GPERF_VPU_H

#include <cstdint>
#include <vector>

#include "../memory/scratchpad.h"
#include "../params/config.h"
#include "../sim/event_queue.h"
#include "../sim/types.h"
#include "../control/reservation_station.h"

namespace gperf {

// VPU (vpu/Vpu.scala): vpu.units BF16 vector engines on the scratchpad; a command goes to a free unit, and a unit
// takes the next one only once its pipeline has drained (cmd.ready := !active && !inFlight).
// Issue streams one row per cycle: src1 and (unless broadcast past the first row of a logical row) src2 are read
// through their banks' read ports at top priority -- two reads in one bank share its port, the +1 cycle the RTL
// pays on a conflict -- and EXPSUM leaves one empty slot per logical row for its row-sum write. Results are
// written vpu.latency cycles later through the write ports (top priority): one row per row, one per logical row for
// a reduction, plus EXPSUM's row sums at dst2.
class vpu_t {
public:
  vpu_t(const config_t &c, event_queue_t &eq, scratchpad_t &sp, reservation_station_t &rs);

  struct cmd_t { uint32_t src1, src2, dst, dst2, rows, rlen; int op; bool bcast; };
  static cmd_t decode(uint64_t rs1, uint64_t rs2);
  static bool uses_src2(int op);
  static uint32_t dst_rows(const cmd_t &c);
  static uint32_t src2_rows(const cmd_t &c);
  static uint32_t groups(const cmd_t &c);   // logical rows

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
