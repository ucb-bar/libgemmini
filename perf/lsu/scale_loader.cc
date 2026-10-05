#include "scale_loader.h"

#include <cmath>

namespace gperf {

scale_loader_t::scale_loader_t(const config_t &c, event_queue_t &eq, memory_system_t &mem)
    : eq_(eq), mem_(mem), line_((uint32_t)c.mem_line_bytes), start_q_((size_t)c.scale_start_q),
      slots_((size_t)c.scale_slots), bytes_per_cycle_(c.scale_bytes_per_cycle),
      row_bubble_((cycle_t)c.scale_row_bubble) {}

void scale_loader_t::submit(uint64_t addr, uint32_t len, uint32_t rows, uint64_t pitch) {
  job_t j;
  for (uint32_t r = 0; r < rows; r++) {
    uint64_t a = addr + (uint64_t)r * pitch;
    const uint64_t end = a + len;
    while (a < end) {
      // largest of line, line/2, ... 8 bytes that is aligned at a and fits; else one 8-byte Get
      uint32_t sz = line_;
      while (sz > 8 && ((a % sz) != 0 || a + sz > end)) sz >>= 1;
      const uint32_t useful = (uint32_t)((a + sz < end ? a + sz : end) - a);
      j.gets.push_back({a, useful, false});
      a += sz;
    }
    if (!j.gets.empty()) j.gets.back().row_end = true;
  }
  jobs_.push_back(std::move(j));
  if (!running_) start_next();
}

void scale_loader_t::when_idle(done_t cb) {
  if (jobs_.empty()) {
    const cycle_t t = eq_.now();
    eq_.at(t, [cb, t] { cb(t); });
  } else {
    idle_waiters_.push_back(std::move(cb));
  }
}

void scale_loader_t::start_next() {
  if (jobs_.empty()) {
    running_ = false;
    auto w = std::move(idle_waiters_);
    idle_waiters_.clear();
    for (auto &cb : w) cb(eq_.now());
    return;
  }
  running_ = true;
  next_get_ = retired_ = in_flight_ = 0;
  retire_free_ = (double)eq_.now();
  if (jobs_.front().gets.empty()) {
    jobs_.pop_front();
    if (room_cb_) room_cb_();
    start_next();
    return;
  }
  try_issue();
}

void scale_loader_t::try_issue() {
  job_t &j = jobs_.front();
  if (issue_pending_ || next_get_ >= j.gets.size() || in_flight_ >= slots_) return;
  if (next_issue_ > eq_.now()) {
    issue_pending_ = true;
    eq_.at(next_issue_, [this] { issue_pending_ = false; if (running_) try_issue(); });
    return;
  }
  const size_t i = next_get_++;
  in_flight_++;
  next_issue_ = eq_.now() + 1;
  bytes_ += j.gets[i].useful;
  mem_.read(j.gets[i].addr, [this, i](cycle_t t) {
    jobs_.front().gets[i].back = t;
    try_retire();
  });
  try_issue();
}

// retire in address order, scale.bytes_per_cycle, a bubble after each row
void scale_loader_t::try_retire() {
  if (retiring_) return;
  job_t &j = jobs_.front();
  if (retired_ >= j.gets.size() || j.gets[retired_].back == NEVER) return;
  const get_t &g = j.gets[retired_];
  double t = std::max(retire_free_, (double)g.back) + g.useful / bytes_per_cycle_;
  if (g.row_end) t += (double)row_bubble_;
  retire_free_ = t;
  retiring_ = true;
  eq_.at((cycle_t)std::ceil(t), [this] {
    retiring_ = false;
    retired_++;
    in_flight_--;
    if (retired_ == jobs_.front().gets.size()) {
      jobs_.pop_front();
      if (room_cb_) room_cb_();
      start_next();
      return;
    }
    try_retire();
    try_issue();
  });
}

}  // namespace gperf
