#ifndef GPERF_EVENT_QUEUE_H
#define GPERF_EVENT_QUEUE_H

#include <cstdint>
#include <functional>
#include <queue>
#include <unordered_set>
#include <vector>

#include "types.h"

namespace gperf {

using done_t = std::function<void(cycle_t)>;

// The simulation clock. Every component is a process that schedules its own next step here, in cycle order
// (FIFO within a cycle). No component computes a future time for a resource it does not own, so every shared
// resource is arbitrated in time order -- a request made later in program order can still win an earlier cycle.
class event_queue_t {
public:
  using fn_t = std::function<void()>;

  uint64_t at(cycle_t t, fn_t f) {
    if (t < now_) t = now_;
    const uint64_t id = ++seq_;
    heap_.push(ev_t{t, id, std::move(f)});
    return id;
  }
  void cancel(uint64_t id) { if (id) cancelled_.insert(id); }

  cycle_t now() const { return now_; }
  cycle_t last_activity() const { return last_; }   // time of the last event that ran
  uint64_t events() const { return n_; }

  // Run the next event if it is before `limit`. False when there is none.
  bool step(cycle_t limit = NEVER) {
    while (!heap_.empty()) {
      if (heap_.top().t >= limit) return false;
      ev_t e = std::move(const_cast<ev_t &>(heap_.top()));
      heap_.pop();
      if (!cancelled_.empty()) {
        auto it = cancelled_.find(e.id);
        if (it != cancelled_.end()) { cancelled_.erase(it); continue; }
      }
      now_ = e.t;
      last_ = e.t;
      n_++;
      e.f();
      return true;
    }
    return false;
  }
  // Everything strictly before t has happened; the clock reads t.
  void advance_to(cycle_t t) {
    while (step(t)) {}
    if (t > now_) now_ = t;
  }
  void run_all() { while (step()) {} }

private:
  struct ev_t { cycle_t t; uint64_t id; fn_t f; };
  struct later {
    bool operator()(const ev_t &a, const ev_t &b) const { return a.t != b.t ? a.t > b.t : a.id > b.id; }
  };
  std::priority_queue<ev_t, std::vector<ev_t>, later> heap_;
  std::unordered_set<uint64_t> cancelled_;
  cycle_t now_ = 0, last_ = 0;
  uint64_t seq_ = 0, n_ = 0;
};

}  // namespace gperf

#endif
