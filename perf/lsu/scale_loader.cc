#include "scale_loader.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace gperf {

scale_loader_t::scale_loader_t(const config_t &c, event_queue_t &eq, memory_system_t &mem)
    : eq_(eq), mem_(mem), line_((uint32_t)c.mem_line_bytes), get_bytes_((uint32_t)c.scale_get_bytes),
      start_q_((size_t)c.scale_start_q),
      slots_((size_t)c.scale_slots), bytes_per_cycle_(c.scale_bytes_per_cycle),
      row_bubble_((cycle_t)c.scale_row_bubble) {}

void scale_loader_t::submit(const load_t &l) {
  if (getenv("GEMMINI_PERF_DEBUG"))
    fprintf(stderr, "scale load @%lld addr=%llx len=%u rows=%u pitch=%llu sel=%d half=%d gated=%d\n",
            (long long)eq_.now(), (unsigned long long)l.addr, l.len, l.rows, (unsigned long long)l.pitch, l.sel,
            l.half, (int)l.gated);
  queue_.push_back(l);
  if (!l.gated) pending_[l.sel * 2 + l.half]++;   // an ungated load counts as pending from the moment it is queued
  try_start();
}

void scale_loader_t::when(std::function<bool()> cond, done_t cb) {
  if (cond()) {
    const cycle_t t = eq_.now();
    eq_.at(t, [cb, t] { cb(t); });
    return;
  }
  waiters_.push_back({std::move(cond), std::move(cb)});
}

void scale_loader_t::changed() {
  auto w = std::move(waiters_);
  waiters_.clear();
  for (auto &p : w) {
    if (p.first()) p.second(eq_.now());
    else waiters_.push_back(std::move(p));
  }
}

void scale_loader_t::config(int act_half, int wgt_half, bool managed) {
  for (int x = 0; x < 4; x++) {
    const int cfg_half = x / 2 == 0 ? act_half : wgt_half;
    if (cfg_half == x % 2) {
      if (managed) state_[x] = INUSE;
    } else if (state_[x] == INUSE) {
      state_[x] = FREE;
    }
  }
  try_start();
  changed();
}

void scale_loader_t::try_start() {
  if (running_ || queue_.empty()) return;
  const load_t l = queue_.front();
  const int x = l.sel * 2 + l.half;
  if (l.gated && state_[x] != FREE) return;   // head of line: everything behind it waits too
  queue_.pop_front();
  if (room_cb_) room_cb_();
  if (l.gated) { state_[x] = LOADED; pending_[x]++; }
  running_ = true;
  run_x_ = x;
  gets_.clear();
  for (uint32_t r = 0; r < l.rows; r++) {
    uint64_t a = l.addr + (uint64_t)r * l.pitch;
    const uint64_t end = a + l.len;
    while (a < end) {
      // largest of line, line/2, ... 8 bytes that is aligned at a and fits; else one 8-byte Get
      uint32_t sz = get_bytes_;
      while (sz > 8 && ((a % sz) != 0 || a + sz > end)) sz >>= 1;
      gets_.push_back({a, (uint32_t)((a + sz < end ? a + sz : end) - a), false});
      a += sz;
    }
    if (!gets_.empty()) gets_.back().row_end = true;
  }
  next_get_ = retired_ = in_flight_ = 0;
  retire_free_ = (double)eq_.now() + 1;
  if (gets_.empty()) {   // zero-length load
    running_ = false;
    pending_[x]--;
    changed();
    try_start();
    return;
  }
  try_issue();
}

void scale_loader_t::try_issue() {
  if (!running_ || issue_pending_ || next_get_ >= gets_.size() || in_flight_ >= slots_) return;
  if (next_issue_ > eq_.now()) {
    issue_pending_ = true;
    eq_.at(next_issue_, [this] { issue_pending_ = false; try_issue(); });
    return;
  }
  const size_t i = next_get_++;
  in_flight_++;
  next_issue_ = eq_.now() + 1;
  bytes_ += gets_[i].useful;
  mem_.read(gets_[i].addr, [this, i](cycle_t t) {
    gets_[i].back = t;
    try_retire();
  }, "sget", true);
  try_issue();
}

// retire in address order, scale.bytes_per_cycle, a bubble after each row
void scale_loader_t::try_retire() {
  if (retiring_ || retired_ >= gets_.size() || gets_[retired_].back == NEVER) return;
  const get_t &g = gets_[retired_];
  double t = std::max(retire_free_, (double)g.back) + g.useful / bytes_per_cycle_;
  if (g.row_end) t += (double)row_bubble_;
  retire_free_ = t;
  retiring_ = true;
  eq_.at((cycle_t)std::ceil(t), [this] {
    retiring_ = false;
    retired_++;
    in_flight_--;
    if (retired_ == gets_.size()) {   // landed
      running_ = false;
      last_landed_ = eq_.now();
      pending_[run_x_]--;
      changed();
      try_start();
      return;
    }
    try_retire();
    try_issue();
  });
}

}  // namespace gperf
