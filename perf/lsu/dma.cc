#include "dma.h"

#include <cmath>

namespace gperf {

dma_reader_t::dma_reader_t(const config_t &c, event_queue_t &eq, memory_system_t &mem, scratchpad_t &sp)
    : eq_(eq), mem_(mem), sp_(sp), get_bytes_((uint32_t)c.dma_get_bytes),
      max_in_flight_((uint32_t)c.dma_max_in_flight),
      row_bytes_per_cycle_((uint32_t)c.dma_spad_write_bytes_per_cycle),
      interval_((cycle_t)std::ceil(1.0 / c.dma_gets_per_cycle)), ooo_(c.dma_ooo_free != 0) {}

void dma_reader_t::submit(uint64_t addr, uint32_t rows, uint32_t row_bytes, uint64_t stride, uint32_t spad_row,
                          done_t done) {
  const uint64_t job = next_job_++;
  jobs_[job] = std::move(done);
  size_t first = to_issue_.size();
  for (uint32_t r = 0; r < rows; r++) {
    const uint64_t a0 = addr + (uint64_t)r * stride, a1 = a0 + row_bytes;
    for (uint64_t line = a0 / get_bytes_; line * get_bytes_ < a1; line++) {
      const uint64_t lo = std::max<uint64_t>(line * get_bytes_, a0), hi = std::min<uint64_t>((line + 1) * get_bytes_, a1);
      to_issue_.push_back({line * get_bytes_, (uint32_t)(hi - lo), spad_row + r, job, false});
    }
  }
  if (to_issue_.size() == first) {   // nothing to read
    done_t d = std::move(jobs_[job]);
    jobs_.erase(job);
    const cycle_t t = eq_.now();
    eq_.at(t, [d, t] { d(t); });
    return;
  }
  to_issue_.back().last = true;
  job_left_[job] = (uint32_t)(to_issue_.size() - first);
  try_issue();
}

void dma_reader_t::try_issue() {
  if (issue_pending_ || to_issue_.empty() || in_flight_ >= max_in_flight_ || next_issue_ == NEVER) return;
  if (next_issue_ > eq_.now()) {
    issue_pending_ = true;
    eq_.at(next_issue_, [this] { issue_pending_ = false; try_issue(); });
    return;
  }
  get_t g = to_issue_.front();
  to_issue_.pop_front();
  in_flight_++;
  next_issue_ = NEVER;   // until the bus accepts this Get
  const uint64_t seq = base_seq_ + issued_.size();
  issued_.push_back(g);
  gets_++;
  bytes_ += g.useful;
  mem_.read(g.addr, [this, seq](cycle_t t) {
    get_t &e = issued_[(size_t)(seq - base_seq_)];
    e.back = t;
    lat_sum_ += (uint64_t)(t - e.sent);
    if (ooo_) { pack(seq); return; }
    try_merge();
  }, "get", false, [this, seq](cycle_t gr) {
    issued_[(size_t)(seq - base_seq_)].sent = gr;
    next_issue_ = gr + interval_ - 1;   // gr = the end of the accepted cycle
    try_issue();
  });
}

// The beat packer takes responses as they arrive: write the row, free the tracker slot, and complete the job with
// its last written Get.
void dma_reader_t::pack(uint64_t seq) {
  const get_t &g = issued_[(size_t)(seq - base_seq_)];
  const cycle_t cycles = (g.useful + row_bytes_per_cycle_ - 1) / row_bytes_per_cycle_;
  const uint64_t job = g.job;
  sp_.write_port(g.spad_row).request(SP_MVIN, eq_.now(), cycles, [this, job, seq](cycle_t t) {
    in_flight_--;
    issued_[(size_t)(seq - base_seq_)].packed = true;
    while (!issued_.empty() && issued_.front().packed) { issued_.pop_front(); base_seq_++; }
    if (--job_left_[job] == 0) {
      job_left_.erase(job);
      done_t d = std::move(jobs_[job]);
      jobs_.erase(job);
      d(t);
    }
    try_issue();
  });
}

void dma_reader_t::try_merge() {
  if (merging_ || issued_.empty() || issued_.front().back == NEVER) return;
  merging_ = true;
  const get_t &g = issued_.front();
  const cycle_t cycles = (g.useful + row_bytes_per_cycle_ - 1) / row_bytes_per_cycle_;
  sp_.write_port(g.spad_row).request(SP_MVIN, eq_.now(), cycles, [this](cycle_t t) {
    const get_t g = issued_.front();
    issued_.pop_front();
    base_seq_++;
    in_flight_--;
    merging_ = false;
    if (g.last) {
      done_t d = std::move(jobs_[g.job]);
      jobs_.erase(g.job);
      d(t);
    }
    try_merge();
    try_issue();
  });
}

dma_writer_t::dma_writer_t(const config_t &c, event_queue_t &eq, memory_system_t &mem)
    : eq_(eq), mem_(mem), put_bytes_((uint32_t)c.dma_put_bytes), max_in_flight_((uint32_t)c.dma_max_puts_in_flight) {}

void dma_writer_t::submit(uint64_t addr, uint32_t rows, uint32_t row_bytes, uint64_t stride, cycle_t earliest,
                          double cycles_per_row, done_t done, done_t issued, uint32_t slack) {
  const uint64_t job = next_job_++;
  uint64_t n = 0;
  for (uint32_t r = 0; r < rows; r++) {
    const cycle_t ready = earliest + (cycle_t)std::ceil((r + 1) * cycles_per_row);
    const uint64_t a0 = addr + (uint64_t)r * stride, a1 = a0 + row_bytes;
    for (uint64_t ch = a0 / put_bytes_; ch * put_bytes_ < a1; ch++) {
      const uint64_t lo = std::max<uint64_t>(ch * put_bytes_, a0), hi = std::min<uint64_t>((ch + 1) * put_bytes_, a1);
      to_issue_.push_back({lo, (uint32_t)(hi - lo), ready, job, false});
      n++;
    }
  }
  if (n == 0) {
    const cycle_t t = cmax(earliest, eq_.now());
    eq_.at(t, [done, issued, t] { if (issued) issued(t); done(t); });
    return;
  }
  if (slack >= n) {   // all of it fits in the write queues: the store is released now
    if (issued) { const cycle_t t = cmax(earliest, eq_.now()); eq_.at(t, [issued, t] { issued(t); }); }
    issued = nullptr;
  } else {
    to_issue_[to_issue_.size() - 1 - slack].fire = true;
  }
  jobs_[job] = {n, std::move(done), 0, std::move(issued)};
  try_issue();
}

void dma_writer_t::try_issue() {
  if (issue_pending_ || to_issue_.empty() || in_flight_ >= max_in_flight_ || next_issue_ == NEVER) return;
  const cycle_t when = cmax(next_issue_, to_issue_.front().ready);
  if (when > eq_.now()) {
    issue_pending_ = true;
    eq_.at(when, [this] { issue_pending_ = false; try_issue(); });
    return;
  }
  const put_t p = to_issue_.front();
  to_issue_.pop_front();
  in_flight_++;
  next_issue_ = NEVER;   // until the bus accepts this Put
  puts_++;
  mem_.write(p.addr, p.bytes, [this, job = p.job](cycle_t t) {
    in_flight_--;
    auto it = jobs_.find(job);
    if (it != jobs_.end() && --it->second.left == 0) {
      done_t d = std::move(it->second.done);
      jobs_.erase(it);
      d(t);
    }
    try_issue();
  }, [this, p](cycle_t gr) {
    next_issue_ = gr;   // gr = the end of the accepted bus slot
    if (p.fire) {
      auto it = jobs_.find(p.job);
      if (it != jobs_.end() && it->second.issued) { done_t cb = std::move(it->second.issued); cb(gr); }
    }
    try_issue();
  });
}

}  // namespace gperf
