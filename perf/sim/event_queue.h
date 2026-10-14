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
//
// Two phases per cycle, as clocked logic: at() events (evaluate: read state, decide) run before commit() events
// (register updates). A commit is invisible to every decision of its own cycle whatever the insertion order, and
// anything it schedules for the current cycle runs the next one. State that the RTL holds in a register and that
// one unit sets while another tests it in the same cycle (the store unit's pending banks vs a vector issue) is
// changed only through commit().
class event_queue_t {
public:
  using fn_t = std::function<void()>;

  uint64_t at(cycle_t t, fn_t f) { return push(t, EVAL, std::move(f)); }
  uint64_t commit(cycle_t t, fn_t f) { return push(t, COMMIT, std::move(f)); }   // phase 2 of cycle t
  uint64_t commit(fn_t f) { return push(now_, COMMIT, std::move(f)); }           // ... of this cycle
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
      phase_ = e.phase;
      last_ = e.t;
      n_++;
      e.f();
      phase_ = EVAL;   // callers outside an event (the instruction stream) schedule as evaluates
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
  enum phase_t : uint8_t { EVAL, COMMIT };
  uint64_t push(cycle_t t, phase_t ph, fn_t f) {
    if (t < now_) t = now_;
    if (t == now_ && ph < phase_) t++;   // an evaluate scheduled by a commit: next cycle
    const uint64_t id = ++seq_;
    heap_.push(ev_t{t, ph, id, std::move(f)});
    return id;
  }
  struct ev_t { cycle_t t; phase_t phase; uint64_t id; fn_t f; };
  struct later {
    bool operator()(const ev_t &a, const ev_t &b) const {
      if (a.t != b.t) return a.t > b.t;
      if (a.phase != b.phase) return a.phase > b.phase;
      return a.id > b.id;
    }
  };
  phase_t phase_ = EVAL;
  std::priority_queue<ev_t, std::vector<ev_t>, later> heap_;
  std::unordered_set<uint64_t> cancelled_;
  cycle_t now_ = 0, last_ = 0;
  uint64_t seq_ = 0, n_ = 0;
};

}  // namespace gperf

#endif
