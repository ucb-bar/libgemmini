#ifndef GPERF_SCALE_LOADER_H
#define GPERF_SCALE_LOADER_H

#include <cstdint>
#include <deque>
#include <functional>
#include <vector>

#include "../memory/memory_system.h"
#include "../params/config.h"
#include "../sim/event_queue.h"
#include "../sim/types.h"

namespace gperf {

// MX_LOAD_SCALES loader (Controller.scala:542-721), outside the reservation station. Loads wait in a
// scale.start_q queue (the command stream stalls while it is full) and run one at a time: each row is cut into
// aligned Gets of up to a line, at most scale.slots outstanding, retired in address order at
// scale.bytes_per_cycle, with scale.row_bubble cycles between rows.
//
// The scale memory has two halves per side (x = sel*2 + half; sel 0 = activation / A scales, 1 = weight / B).
// A loop-managed ("gated") load into half x starts only when x is FREE, and the queue head blocks the ones behind
// it. Half states: FREE -(gated load starts)-> LOADED -(managed CONFIG_SCALE_MEM selects it)-> INUSE -(a config
// selects the other half)-> FREE. So loop n+1's scales load into the other half while loop n computes, and loop
// n+2's wait until loop n+1's config has freed loop n's half -- the scale double buffer.
class scale_loader_t {
public:
  scale_loader_t(const config_t &c, event_queue_t &eq, memory_system_t &mem);

  struct load_t { uint64_t addr; uint32_t len, rows; uint64_t pitch; int sel, half; bool gated; };
  bool has_room() const { return queue_.size() < start_q_; }
  void submit(const load_t &l);

  bool landed(int x) const { return pending_[x] == 0; }                       // no load into x outstanding
  bool ready(int x) const { return pending_[x] == 0 && state_[x] == LOADED; } // a gated load into x has landed
  bool idle() const { return queue_.empty() && !running_; }
  cycle_t last_landed() const { return last_landed_; }
  // A CONFIG_SCALE_MEM executes: it reads activation half act_half and weight half wgt_half.
  void config(int act_half, int wgt_half, bool managed);
  // cb(t) once cond() holds; re-checked whenever the loader's state changes.
  void when(std::function<bool()> cond, done_t cb);
  void on_room(std::function<void()> cb) { room_cb_ = std::move(cb); }

  uint64_t bytes() const { return bytes_; }

private:
  enum half_state_t { FREE, LOADED, INUSE };
  struct get_t { uint64_t addr; uint32_t useful; bool row_end; cycle_t back = NEVER; };
  void try_start();
  void try_issue();
  void try_retire();
  void changed();

  event_queue_t &eq_;
  memory_system_t &mem_;
  uint32_t line_, get_bytes_;
  size_t start_q_, slots_;
  double bytes_per_cycle_;
  cycle_t row_bubble_;
  std::deque<load_t> queue_;          // submitted, not started
  bool running_ = false;
  int run_x_ = 0;
  std::vector<get_t> gets_;           // the running load's Gets
  size_t next_get_ = 0, retired_ = 0, in_flight_ = 0;
  double retire_free_ = 0;
  bool issue_pending_ = false, retiring_ = false;
  cycle_t next_issue_ = 0;
  int pending_[4] = {0, 0, 0, 0};
  half_state_t state_[4] = {FREE, FREE, FREE, FREE};
  std::vector<std::pair<std::function<bool()>, done_t>> waiters_;
  std::function<void()> room_cb_;
  uint64_t bytes_ = 0;
  cycle_t last_landed_ = 0;
};

}  // namespace gperf

#endif
