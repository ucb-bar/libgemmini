#ifndef GPERF_EXECUTE_UNIT_H
#define GPERF_EXECUTE_UNIT_H

#include <cstdint>
#include <deque>
#include <functional>
#include <vector>

#include "../memory/accumulator.h"
#include "../params/config.h"
#include "../sim/event_queue.h"
#include "../control/reservation_station.h"
#include "../memory/scratchpad.h"
#include "../sim/types.h"

namespace gperf {

// ExecuteController + MeshWithDelays, weight stationary. Commands run in order from an ex.queue_length queue.
// A compute streams its A rows (DIM, at least mesh.min_rows) through the mesh while the preload paired with it
// reads the new weights; both read the scratchpad through the bank read ports (priority ex), so two operands
// in one bank serialise and a VPU stream stalls them. Computes run back to back with no bubble. The results land
// in the accumulator bank (top priority on its port) from mesh.fill_latency, the last commit at
// mesh.commit_latency. Every PE mode costs the same cycles per request (quad modes do more MACs in them).
// The compute's RS entry completes when its rows are fed; the preload's when the results are committed.
// CONFIG_EX / CONFIG_SCALE_MEM wait until no matmul is in flight.
class execute_unit_t {
public:
  execute_unit_t(const config_t &c, event_queue_t &eq, scratchpad_t &sp, accumulator_t &acc,
                 reservation_station_t &rs);

  struct ex_cmd_t {
    enum kind_t { PRELOAD, COMPUTE, CONFIG } kind;
    local_addr_t a{0xFFFFFFFFu}, b{0xFFFFFFFFu}, c{0xFFFFFFFFu};   // compute: a; preload: b (weights), c (acc)
    uint32_t rows = 0;
    std::function<void(done_t)> wait_for;   // CONFIG: an extra condition (e.g. scale loads landed)
  };
  bool has_room() const { return q_.size() < queue_len_; }
  void accept(uint64_t rs_id, ex_cmd_t c);

  uint64_t busy_cycles() const { return busy_; }
  uint64_t tiles() const { return tiles_; }

private:
  struct item_t { uint64_t id; ex_cmd_t c; };
  void process();
  void run_tile(const item_t *pre, const item_t &comp);

  event_queue_t &eq_;
  scratchpad_t &sp_;
  accumulator_t &acc_;
  reservation_station_t &rs_;
  size_t queue_len_;
  uint32_t dim_, min_rows_;
  cycle_t issue_lat_, fill_lat_, commit_lat_, drain_extra_;
  std::deque<item_t> q_;
  bool busy_now_ = false;          // a tile is feeding or a config is draining
  int tiles_in_flight_ = 0;        // fed, not yet committed
  local_addr_t weights_{0xFFFFFFFFu};
  cycle_t last_feed_end_ = -1;     // the mesh streams with no bubble while computes keep coming
  uint64_t busy_ = 0, tiles_ = 0;
};

}  // namespace gperf

#endif
