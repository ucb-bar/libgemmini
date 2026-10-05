#ifndef GPERF_RESERVATION_STATION_H
#define GPERF_RESERVATION_STATION_H

#include <cstdint>
#include <deque>
#include <functional>
#include <unordered_map>
#include <vector>

#include "../params/config.h"
#include "../sim/event_queue.h"
#include "../sim/types.h"

namespace gperf {

// ReservationStation.scala. Four queues (ld / ex / st / vec) with fixed entries; allocation needs a free entry
// in the command's queue. ld and ex issue in order and pipeline (one issue per queue per cycle); st and vec issue
// only once every older entry of their queue has completed. Across queues an entry waits for every older live
// entry whose rows overlap its own where either side writes; that dependency clears when the older one completes.
// Configs outside the ex queue complete on issue.
struct rs_cmd_t {
  queue_t q = Q_LD;
  span_t a, b;                               // the rows it reads / writes
  bool config = false;
  std::function<bool()> unit_has_room;       // the unit's command queue can take it
  std::function<void(uint64_t id)> start;    // hand it to the unit; the unit calls complete(id)
  int tag = -1;                              // >= 0: issued by LoopMatmul (for its throttles)
  const char *what = "";
};

class reservation_station_t {
public:
  reservation_station_t(const config_t &c, event_queue_t &eq);

  bool has_room(queue_t q) const { return order_[q].size() < (size_t)cap_[q]; }
  uint64_t alloc(rs_cmd_t c);
  void complete(uint64_t id);   // at the current cycle
  void kick(queue_t q);         // something changed (a unit took a command, a dependency cleared)

  void on_room(std::function<void()> cb) { room_cb_ = std::move(cb); }
  void on_done(std::function<void(int tag)> cb) { done_cb_ = std::move(cb); }
  void on_trace(std::function<void(const rs_cmd_t &, cycle_t alloc, cycle_t issue, cycle_t done)> cb) {
    trace_cb_ = std::move(cb);
  }

  bool idle() const { return ents_.empty(); }
  uint64_t allocs() const { return allocs_; }

private:
  struct ent_t {
    rs_cmd_t c;
    bool issued = false;
    int deps = 0;
    std::vector<uint64_t> dependents;
    cycle_t alloc_t = 0, issue_t = 0;
  };
  void try_issue(queue_t q);

  event_queue_t &eq_;
  int cap_[Q_COUNT];
  std::unordered_map<uint64_t, ent_t> ents_;
  std::deque<uint64_t> order_[Q_COUNT];   // live entries, oldest first
  cycle_t last_issue_[Q_COUNT] = {-1, -1, -1, -1};
  bool pending_[Q_COUNT] = {false, false, false, false};
  uint64_t next_id_ = 1, allocs_ = 0;
  std::function<void()> room_cb_;
  std::function<void(int)> done_cb_;
  std::function<void(const rs_cmd_t &, cycle_t, cycle_t, cycle_t)> trace_cb_;
};

}  // namespace gperf

#endif
