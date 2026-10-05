#include "port.h"

#include <algorithm>
#include <utility>

namespace gperf {

void port_t::request(int prio, cycle_t earliest, cycle_t cycles, done_t done) {
  advance(eq_.now());
  if (earliest < eq_.now()) earliest = eq_.now();
  if (cycles <= 0) {
    eq_.at(earliest, [done, earliest] { done(earliest); });
    return;
  }
  pend_.push_back({prio, earliest, cycles, ++seq_, std::move(done)});
  reschedule();
}

// The demand that owns cycle c: eligible, best priority, oldest. -1 if none.
int port_t::pick(const std::vector<demand_t> &d, const std::vector<cycle_t> &left, cycle_t c) {
  int b = -1;
  for (size_t i = 0; i < d.size(); i++) {
    if (left[i] <= 0 || d[i].earliest > c) continue;
    if (b < 0 || d[i].prio < d[(size_t)b].prio || (d[i].prio == d[(size_t)b].prio && d[i].seq < d[(size_t)b].seq))
      b = (int)i;
  }
  return b;
}

// The first cycle after c at which a demand better than `prio` becomes eligible (any demand, if prio < 0).
cycle_t port_t::next_event(const std::vector<demand_t> &d, const std::vector<cycle_t> &left, cycle_t c, int prio) {
  cycle_t n = NEVER;
  for (size_t i = 0; i < d.size(); i++)
    if (left[i] > 0 && d[i].earliest > c && (prio < 0 || d[i].prio < prio)) n = std::min(n, d[i].earliest);
  return n;
}

void port_t::advance(cycle_t t) {
  std::vector<std::pair<done_t, cycle_t>> fired;
  std::vector<cycle_t> left(pend_.size());
  for (size_t i = 0; i < pend_.size(); i++) left[i] = pend_[i].left;
  while (cursor_ < t) {
    const int b = pick(pend_, left, cursor_);
    if (b < 0) {
      const cycle_t n = next_event(pend_, left, cursor_, -1);
      cursor_ = n >= t ? t : n;
      continue;
    }
    const cycle_t until = std::min({cursor_ + left[(size_t)b], t, next_event(pend_, left, cursor_, pend_[(size_t)b].prio)});
    left[(size_t)b] -= until - cursor_;
    busy_ += (uint64_t)(until - cursor_);
    cursor_ = until;
    if (left[(size_t)b] == 0) fired.push_back({std::move(pend_[(size_t)b].done), cursor_});
  }
  // drop finished demands, keep the rest with their remaining cycles
  std::vector<demand_t> keep;
  for (size_t i = 0; i < pend_.size(); i++)
    if (left[i] > 0) { pend_[i].left = left[i]; keep.push_back(std::move(pend_[i])); }
  pend_.swap(keep);
  // callbacks may request this port again: run them once the state is consistent
  for (auto &f : fired) f.first(f.second);
}

void port_t::reschedule() {
  eq_.cancel(wake_);
  wake_ = 0;
  if (pend_.empty()) return;
  std::vector<cycle_t> left(pend_.size());
  for (size_t i = 0; i < pend_.size(); i++) left[i] = pend_[i].left;
  cycle_t c = cursor_;
  for (;;) {
    const int b = pick(pend_, left, c);
    if (b < 0) {
      const cycle_t n = next_event(pend_, left, c, -1);
      if (n == NEVER) return;
      c = n;
      continue;
    }
    const cycle_t until = std::min(c + left[(size_t)b], next_event(pend_, left, c, pend_[(size_t)b].prio));
    left[(size_t)b] -= until - c;
    c = until;
    if (left[(size_t)b] == 0) break;
  }
  wake_ = eq_.at(c, [this] {
    wake_ = 0;
    advance(eq_.now());
    reschedule();
  });
}

}  // namespace gperf
