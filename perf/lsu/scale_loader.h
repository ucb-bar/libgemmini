#ifndef GPERF_SCALE_LOADER_H
#define GPERF_SCALE_LOADER_H

#include <cstdint>
#include <deque>
#include <vector>

#include "../params/config.h"
#include "../sim/event_queue.h"
#include "../memory/memory_system.h"
#include "../sim/types.h"

namespace gperf {

// MX_LOAD_SCALES loader (Controller.scala:542-721), outside the reservation station. Loads queue in a
// scale.start_q queue (the command stream stalls while it is full) and run one at a time: each row is cut into
// aligned Gets of up to a line, at most scale.slots outstanding, retired in address order at
// scale.bytes_per_cycle, with scale.row_bubble cycles between rows. Only a fence (or a CONFIG_SCALE_MEM that
// waits) orders it against the mesh.
class scale_loader_t {
public:
  scale_loader_t(const config_t &c, event_queue_t &eq, memory_system_t &mem);

  bool has_room() const { return jobs_.size() < start_q_; }
  void submit(uint64_t addr, uint32_t len, uint32_t rows, uint64_t pitch);
  void when_idle(done_t cb);    // cb(t) once every load submitted so far has landed
  void on_room(std::function<void()> cb) { room_cb_ = std::move(cb); }

  bool idle() const { return jobs_.empty(); }
  uint64_t bytes() const { return bytes_; }

private:
  struct get_t { uint64_t addr; uint32_t useful; bool row_end; cycle_t back = NEVER; };
  struct job_t { std::vector<get_t> gets; };
  void start_next();
  void try_issue();
  void try_retire();

  event_queue_t &eq_;
  memory_system_t &mem_;
  uint32_t line_;
  size_t start_q_, slots_;
  double bytes_per_cycle_;
  cycle_t row_bubble_;
  std::deque<job_t> jobs_;            // front = running
  size_t next_get_ = 0, retired_ = 0, in_flight_ = 0;
  double retire_free_ = 0;
  bool running_ = false, issue_pending_ = false, retiring_ = false;
  cycle_t next_issue_ = 0;
  std::vector<done_t> idle_waiters_;
  std::function<void()> room_cb_;
  uint64_t bytes_ = 0;
};

}  // namespace gperf

#endif
