#include "store_unit.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>

namespace gperf {

store_unit_t::store_unit_t(const config_t &c, event_queue_t &eq, scratchpad_t &sp, accumulator_t &acc,
                           dma_writer_t &dma, reservation_station_t &rs)
    : eq_(eq), sp_(sp), acc_(acc), dma_(dma), rs_(rs), queue_len_((size_t)c.st_queue_length),
      dim_((uint32_t)c.mesh_dim), lag_((cycle_t)c.st_completion_lag), rq_lat_((cycle_t)c.st_requant_latency),
      pipe_lat_((cycle_t)c.st_pipe_latency), slack_((cycle_t)c.st_write_slack),
      spad_rd_interval_((cycle_t)c.st_spad_read_interval), pend_delay_((cycle_t)c.st_pending_delay),
      spad_lead_((cycle_t)c.st_spad_write_lead), pend_tail_((cycle_t)c.st_pending_tail),
      elems_per_read_(c.st_elems_per_acc_read) {}

void store_unit_t::config(uint64_t rs1, uint64_t rs2) {
  (void)rs1;
  stride_ = rs2 & 0xFFFFFFFFu;
}

span_t store_unit_t::src_span(const st_cmd_t &c) const {
  const uint32_t blocks = (c.cols + dim_ - 1) / dim_;
  // the RS's mvout range for a raw mvout, (blocks-1)*DIM + rows from its rs2 (ReservationStation.scala:286-295).
  // Loop-issued C stores carry their own range (loop_cmd_t::rs_span). The RS's store/mesh stalls come from the
  // preloads' DIM-row C range spilling over the packed DIM/4-row tiles (13.5).
  return make_span(c.src, (uint64_t)(blocks - 1) * dim_ + c.rows, false);
}

uint32_t store_unit_t::pending_banks() const {
  uint32_t m = 0;
  for (uint32_t b = 0; b < 32; b++) if (pend_[b]) m |= 1u << b;
  return m;
}

void store_unit_t::accept(uint64_t rs_id, st_cmd_t c) {
  const uint64_t stride = c.stride ? c.stride : stride_;
  q_.push_back({rs_id, c, stride});
  process();
}

void store_unit_t::process() {
  if (reading_ || q_.empty()) return;
  if (q_.front().c.src.is_acc() && hold_ && hold_()) return;   // re-run when the FP4 SPAD_REQUANT is done
  const item_t it = q_.front();
  q_.pop_front();
  rs_.kick(Q_ST);
  reading_ = true;
  acc_src_ = it.c.src.is_acc();
  const st_cmd_t &c = it.c;
  const uint32_t blocks = (c.cols + dim_ - 1) / dim_;
  const cycle_t now = eq_.now();
  double reads, row_bytes;
  port_t *rd;
  int prio;
  cycle_t data_from;
  if (c.src.is_acc()) {
    const bool full = (c.src.raw >> 29) & 1;
    reads = std::ceil((double)c.rows * c.cols / elems_per_read_);
    row_bytes = c.cols * (full ? 4.0 : c.out_bytes) * c.out_mult;
    rd = &acc_.port(c.src.row());
    prio = ACC_STORE;
    data_from = now + pipe_lat_ + rq_lat_;
  } else {
    reads = (double)c.rows * blocks;   // one DIM-byte row per cycle
    row_bytes = c.cols;
    rd = &sp_.read_port(c.src.row());
    prio = SP_STORE;
    data_from = now + pipe_lat_;
  }
  reads_ += (uint64_t)reads;
  const uint64_t id = it.id;
  // The command is done (RS completion, next store may start) when its reads are done and -- for a store to DRAM,
  // whose short write queues back-pressure the reads -- its Puts have all but write_slack been issued.
  const bool to_dram = c.kind == st_cmd_t::TO_DRAM;
  // acc -> scratchpad reads are paced by their own writes: a starved write port (VPU writes win) stalls the reads
  const bool paced_spad = !to_dram && c.src.is_acc() && spad_rd_interval_ > 1;
  auto left = std::make_shared<int>(to_dram || paced_spad ? 2 : 1);
  auto latest = std::make_shared<cycle_t>(now);
  auto part = [this, id, left, latest](cycle_t t) {
    *latest = cmax(*latest, t);
    if (--*left > 0) return;
    const cycle_t done = *latest;
    eq_.at(done + lag_, [this, id] { rs_.complete(id); });
    eq_.at(done, [this] {
      reading_ = false;
      if (acc_src_) rs_.kick(Q_VEC);   // an FP4 SPAD_REQUANT may have waited for the requantizer to drain
      process();
    });
  };
  // scratchpad rows this store writes (a multi-elem quad tile: out_mult outputs per acc element)
  const cycle_t beats = (cycle_t)std::ceil((double)c.rows * c.cols * c.out_bytes * c.out_mult / dim_);
  cycle_t paced = 0;
  if (paced_spad) {
    // acc -> scratchpad: each read waits for the previous result's scratchpad write (one read per interval, or the
    // result's write beats when it has more -- the FP4 32x32 BF16 tile writes 128 rows from 8 reads); each read's
    // rows reach the bank spad_lead_ later
    paced = now + 1 + std::max((cycle_t)std::ceil(reads) * spad_rd_interval_, beats);
    // the reads go out one at a time across the paced window (each waits for an earlier result's write), so a late
    // read meets whatever holds the bank's read port then (an accumulating mesh write: AccumulatorMem.scala:667)
    const uint32_t n = (uint32_t)std::max(1.0, std::ceil(reads));
    const double step = (double)(paced - now - 1) / n;
    auto k = std::make_shared<uint32_t>(0);
    auto issue = std::make_shared<std::function<void(cycle_t)>>();
    *issue = [this, rd, prio, k, n, step, now, paced, part, issue](cycle_t at) {
      rd->request(prio, at, 1, [this, k, n, step, now, paced, part, issue](cycle_t t) {
        if (++*k == n) { part(cmax(t, paced)); *issue = nullptr; return; }
        (*issue)(cmax(t, now + 1 + (cycle_t)(step * *k)));
      });
    };
    (*issue)(now + 1);
    data_from = cmax(now + spad_lead_, paced + spad_lead_ - beats);   // unstarved: all but `lead` rows by `paced`
  } else {
    rd->request(prio, now + 1, (cycle_t)reads, part);
  }
  if (to_dram) {
    dma_.submit(c.dram, c.rows, (uint32_t)std::ceil(row_bytes), it.stride, data_from, reads / c.rows, [](cycle_t) {},
                [part](cycle_t t) { part(t); }, (uint32_t)slack_);
  } else {
    const uint32_t b0 = sp_.bank_of(c.dst.row()), b1 = sp_.bank_of(c.dst.row() + (uint32_t)(beats ? beats - 1 : 0));
    // the banks count as pending once the first rows reach write_norm_q (Scratchpad.scala st_in), pend_delay_ later
    auto claim = [this, b0, b1] { for (uint32_t b = b0; b <= b1 && b < 32; b++) pend_[b]++; };
    if (pend_delay_ > 0) eq_.at(now + pend_delay_, claim);
    else claim();
    auto released = std::make_shared<bool>(false);
    auto release = [this, b0, b1, released] {
      if (*released) return;
      *released = true;
      for (uint32_t b = b0; b <= b1 && b < 32; b++) pend_[b]--;
      rs_.kick(Q_VEC);   // a vector entry may have been waiting on these banks
    };
    port_t &wr = sp_.write_port(c.dst.row());
    const cycle_t lead = spad_lead_, tail = pend_tail_;
    if (paced_spad && beats > lead) {
      // The reads are paced by the writes. All but the last request (`lead` rows) written = the store's rows have
      // left write_issue_q: the store is done reading (completion) and its banks stop counting in vpu_pending_banks
      // (st_out; the last rows wait in the output stage uncounted) `tail` later, after a back-to-back next store has
      // claimed its own. A VPU op that then issues wins the bank's write port and starves those last rows
      // (RTL: 655 cycles behind a 1,031-cycle op); a store starved earlier stalls its completion.
      wr.request(SP_REQUANT, data_from, beats - lead, [this, part, release, tail](cycle_t t) {
        last_write_ = cmax(last_write_, t);
        part(t);
        eq_.at(t + tail, release);
      });
      wr.request(SP_REQUANT, data_from, lead, [this](cycle_t t) { last_write_ = cmax(last_write_, t); });
    } else {
      if (paced_spad) part(now);   // nothing to wait for on the write side
      wr.request(SP_REQUANT, data_from, beats, [this, release](cycle_t t) {
        last_write_ = cmax(last_write_, t);
        release();
      });
    }
  }
}

}  // namespace gperf
