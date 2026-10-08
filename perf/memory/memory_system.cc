#include "memory_system.h"

#include <algorithm>
#include <cmath>

namespace gperf {

memory_system_t::memory_system_t(const config_t &c, event_queue_t &eq)
    : eq_(eq), bus_(eq, "bus"), client_bus_(eq, "client_bus"), dram_(eq, "dram"), l1_probe_(eq, "l1_probe"),
      line_bytes_((uint32_t)c.mem_line_bytes),
      capacity_lines_((size_t)(c.mem_l2_kib * 1024 / c.mem_line_bytes)),
      hit_lat_((cycle_t)c.mem_l2_hit_latency), client_hit_lat_((cycle_t)c.mem_client_hit_latency),
      dram_lat_((cycle_t)c.mem_dram_latency),
      ack_lat_((cycle_t)c.mem_write_ack_latency), full_ack_lat_((cycle_t)c.mem_full_write_ack_latency),
      put_beats_((cycle_t)std::ceil((double)c.mem_line_bytes / c.mem_bus_bytes)),
      dram_cycles_per_line_((cycle_t)std::ceil(c.mem_line_bytes / c.mem_dram_bytes_per_cycle)),
      put_serial_((cycle_t)c.mem_l2_put_serial_cycles),
      fill_secondary_first_((cycle_t)c.mem_l2_fill_secondary_first),
      fill_secondary_next_((cycle_t)c.mem_l2_fill_secondary_next),
      l1_ways_((uint32_t)c.mem_host_l1_ways), l1_random_(c.mem_host_l1_random != 0),
      probe_issue_((cycle_t)c.mem_probe_issue_latency), probe_cycles_((cycle_t)c.mem_probe_cycles),
      probe_lat_((cycle_t)c.mem_probe_latency), probe_put_ack_((cycle_t)c.mem_probe_put_ack_latency) {
  l1_sets_ = std::max<uint32_t>(1, (uint32_t)(c.mem_host_l1_kib * 1024 / c.mem_line_bytes / l1_ways_));
  l1_lru_.resize(l1_sets_);
  miss_detect_ = (cycle_t)c.mem_l2_miss_detect;
  fill_to_data_ = (cycle_t)c.mem_l2_fill_to_data;
  if (c.mem_host_tracking >= 2) {
    l1d_ = new host_cache_t((size_t)c.host_l1d_sets, (size_t)c.host_l1d_ways, line_bytes_, "host L1 D$");
    if (c.host_icache) l1i_ = new host_cache_t((size_t)c.host_l1i_sets, (size_t)c.host_l1i_ways, line_bytes_, "host L1 I$");
    pen_l2_ = (cycle_t)c.host_l1_miss_l2_hit;
    pen_dram_ = (cycle_t)c.host_l1_miss_dram;
    pen_wb_ = (cycle_t)c.host_l1_writeback;
  }
  dram_banked_ = c.mem_dram_model != 0;
  dram_max_reads_ = (uint32_t)std::max(0.0, c.mem_dram_max_reads);
  dram_banks_ = std::max<uint32_t>(1, (uint32_t)c.mem_dram_banks);
  dram_row_lines_ = std::max<uint32_t>(1, (uint32_t)c.mem_dram_lines_per_row);
  row_miss_ = (cycle_t)c.mem_dram_row_miss_cycles;
  t_rc_ = (cycle_t)c.mem_dram_rc_cycles;
  open_row_.assign(dram_banks_, UINT64_MAX);
  bank_act_free_.assign(dram_banks_, 0);
  // Approximation (user: the cache may be approximate): the L1 starts full of lines the model never saw (boot,
  // code, data), so random replacement evicts some CPU-written lines just as in the RTL -- a full L1 keeps ~66% of a
  // 32 KB memset vs ~60% measured (dramloop FSDB: 305 of 512 C lines probed). Placeholder ids are never addresses.
  if (c.mem_host_l1_start_full && !l1d_)
    for (uint32_t s = 0; s < l1_sets_; s++)
      for (uint32_t w = 0; w < l1_ways_; w++) {
        const uint64_t ph = ~(uint64_t)0 - ((uint64_t)w * l1_sets_ + s);   // placeholder, maps to set s
        l1_lru_[s].push_back(ph - (ph % l1_sets_) + s);
        l1_[l1_lru_[s].back()] = std::prev(l1_lru_[s].end());
      }
}

// An L1 miss of the (blocking) core: the L2 has the line (hit) or fetches it (DRAM); the inclusive L2 holds it from
// now on. Fixed stalls -- the core's misses do not take the model's bus / DRAM slots (approximation).
cycle_t memory_system_t::host_miss(uint64_t line, host_cache_t::result_t r, bool write) {
  if (r == host_cache_t::HIT) return 0;
  cycle_t s = r == host_cache_t::MISS_WRITEBACK ? pen_wb_ : 0;
  auto jt = lines_.find(line);
  if (jt != lines_.end() && jt->second.ready != NEVER && jt->second.ready <= eq_.now()) {
    s += pen_l2_;
    if (write) jt->second.dirty = true;
  } else {
    s += pen_dram_;
    if (jt == lines_.end()) insert(line, eq_.now(), write);
  }
  return s;
}

cycle_t memory_system_t::host_fetch(uint64_t addr, uint32_t bytes) {
  if (!l1i_) return 0;
  cycle_t s = 0;
  for (uint64_t line = addr / line_bytes_; line * line_bytes_ < addr + (bytes ? bytes : 1); line++)
    s += host_miss(line, l1i_->touch(line * line_bytes_, false), false);
  host_stall_ += s;
  return s;
}

cycle_t memory_system_t::host_access(uint64_t addr, uint32_t bytes, bool write) {
  if (l1d_) {
    cycle_t s = 0;
    for (uint64_t line = addr / line_bytes_; line * line_bytes_ < addr + (bytes ? bytes : 1); line++) {
      if (write) host_stores_++;
      s += host_miss(line, l1d_->touch(line * line_bytes_, write), write);
    }
    host_stall_ += s;
    return s;
  }
  if (!write) return 0;
  for (uint64_t line = addr / line_bytes_; line * line_bytes_ < addr + (bytes ? bytes : 1); line++) {
    host_stores_++;
    auto &set = l1_lru_[line % l1_sets_];
    auto it = l1_.find(line);
    if (it != l1_.end()) {
      if (!l1_random_) set.splice(set.begin(), set, it->second);
    } else {
      if (set.size() >= l1_ways_) {   // the set evicts a line (written back into the L2): random way, or the LRU one
        auto victim = std::prev(set.end());
        if (l1_random_) {
          lfsr_ ^= lfsr_ << 13; lfsr_ ^= lfsr_ >> 7; lfsr_ ^= lfsr_ << 17;
          victim = std::next(set.begin(), (long)(lfsr_ % set.size()));
        }
        l1_.erase(*victim);
        set.erase(victim);
      }
      set.push_front(line);
      l1_[line] = set.begin();
    }
    // inclusive L2: the line is present (its data is in the L1 until probed)
    auto jt = lines_.find(line);
    if (jt == lines_.end()) insert(line, eq_.now(), write);
    else {
      if (write) jt->second.dirty = true;
      if (jt->second.ready != NEVER) lru_.splice(lru_.begin(), lru_, jt->second.pos);
    }
  }
  return 0;
}

bool memory_system_t::take_from_l1(uint64_t line) {
  auto it = l1_.find(line);
  if (it == l1_.end()) return false;
  l1_lru_[line % l1_sets_].erase(it->second);
  l1_.erase(it);
  probes_++;
  return true;
}

// The L2 probes the L1 (mem.probe_issue_latency after taking the request), the L1 serves one probe per
// mem.probe_cycles, and its ProbeAckData lands mem.probe_latency after the probe went out.
// Gemmini writing a line the L1 holds probes it out (toN); reading one probes only a dirty (write-permission) copy,
// which stays as a clean one (toB). host_tracking 1 has no clean/dirty: any held line is probed out.
void memory_system_t::probe(uint64_t line, cycle_t g, bool write, done_t done) {
  if (l1d_) {
    bool dirty = false;
    const uint64_t a = line * line_bytes_;
    if (!l1d_->holds(a, &dirty) || (!write && !dirty)) { done(g); return; }
    probes_++;
    if (write) l1d_->drop(a); else l1d_->clean(a);
  } else if (!take_from_l1(line)) {
    done(g);
    return;
  }
  l1_probe_.request(0, g + probe_issue_, probe_cycles_, [this, done](cycle_t e) {
    const cycle_t t = e - probe_cycles_ + probe_lat_;
    eq_.at(t, [done, t] { done(t); });
  });
}

void memory_system_t::insert(uint64_t line, cycle_t ready, bool dirty) {
  lru_.push_front(line);
  line_t &l = lines_[line];
  l.pos = lru_.begin();
  l.ready = ready;
  l.dirty = dirty;
  if (lines_.size() > capacity_lines_) {
    const uint64_t victim = lru_.back();
    const line_t &v = lines_[victim];
    if (v.ready != NEVER) {   // never evict a line with a fill outstanding
      if (v.dirty) {          // write-back: a DRAM line slot nobody waits for
        writebacks_++;
        dram_.request(0, eq_.now(), dram_cycles_per_line_, [](cycle_t) {});
      }
      lines_.erase(victim);
      lru_.pop_back();
    }
  }
}

void memory_system_t::lookup(uint64_t line, cycle_t t, done_t back, cycle_t hit_lat) {
  auto it = lines_.find(line);
  if (it != lines_.end()) {
    hits_++;
    lru_.splice(lru_.begin(), lru_, it->second.pos);
    if (it->second.ready == NEVER) {   // a fill is outstanding: its own L2 pass after the fill
      pending_hits_++;
      it->second.waiters.push_back([back](cycle_t r) { back(r); });   // released with its data (fetch())
      return;
    }
    const cycle_t r = cmax(t + hit_lat, it->second.ready);
    eq_.at(r, [back, r] { back(r); });
    return;
  }
  misses_++;
  insert(line, NEVER);
  lines_[line].waiters.push_back([this, back, t, hit_lat](cycle_t r) {
    const cycle_t at = cmax(r + fill_to_data_, t + hit_lat);   // the L2 writes the fill, then responds
    eq_.at(at, [back, at] { back(at); });
  });
  fetch(line, t + miss_detect_);   // the L2 detects the miss before it asks DRAM
}

// Open-page DDR3, approximately (no refresh, no scheduler reordering): a row hit costs the unloaded latency; a row
// miss also precharges + activates (row_miss_), not before the bank's previous activate + tRC.
cycle_t memory_system_t::dram_ready(uint64_t line, cycle_t d) {
  if (!dram_banked_) return d + dram_lat_;
  const uint32_t b = (uint32_t)(line % dram_banks_);
  const uint64_t row = line / ((uint64_t)dram_banks_ * dram_row_lines_);
  if (open_row_[b] == row) return d + dram_lat_;
  const cycle_t act = cmax(d, bank_act_free_[b]);
  open_row_[b] = row;
  bank_act_free_[b] = act + t_rc_;
  return act + row_miss_ + dram_lat_;
}

void memory_system_t::fetch(uint64_t line, cycle_t t) {
  if (dram_max_reads_ && dram_reads_ >= dram_max_reads_) { dram_waiting_.push_back({line, t}); return; }
  start_fetch(line, t);
}

void memory_system_t::start_fetch(uint64_t line, cycle_t t) {
  dram_reads_++;
  dram_.request(0, t, dram_cycles_per_line_, [this, line](cycle_t d) {
    const cycle_t r = dram_ready(line, d);
    eq_.at(r, [this, r] {   // the read's slot frees when its data is back
      dram_reads_--;
      if (!dram_waiting_.empty()) {
        const auto w = dram_waiting_.front();
        dram_waiting_.pop_front();
        start_fetch(w.first, cmax(w.second, r));
      }
    });
    auto jt = lines_.find(line);
    if (jt == lines_.end()) return;
    jt->second.ready = r;
    auto waiters = std::move(jt->second.waiters);
    jt->second.waiters.clear();
    // the primary first, then the requests queued behind the fill: one L2 pass later, then closely spaced
    for (size_t k = 0; k < waiters.size(); k++) {
      const cycle_t at = k == 0 ? r : r + fill_secondary_first_ + (cycle_t)(k - 1) * fill_secondary_next_;
      done_t w = waiters[k];
      if (k == 0) w(r); else eq_.at(at, [w, at] { w(at); });
    }
  });
}

void memory_system_t::read(uint64_t addr, done_t back, const char *who, bool own_client, done_t granted) {
  const uint64_t line = addr / line_bytes_;
  (own_client ? client_bus_ : bus_).request(0, eq_.now(), 1, [this, line, back, who, granted, own_client](cycle_t g) {
    trace_ev(who, g);
    if (granted) granted(g);
    const cycle_t lat = own_client ? client_hit_lat_ : hit_lat_;
    probe(line, g, false, [this, line, back, lat](cycle_t t) { lookup(line, t, back, lat); });
  });
}

void memory_system_t::ack(cycle_t a, const done_t &acked) {
  last_ack_ = cmax(last_ack_, a);
  eq_.at(a, [acked, a] { acked(a); });
}

void memory_system_t::put_op(uint64_t line, cycle_t earliest, cycle_t post, done_t acked) {
  cycle_t start = earliest;
  auto it = lines_.find(line);
  if (it != lines_.end()) {
    start = cmax(start, it->second.put_free);
    it->second.put_free = start + put_serial_;
  }
  ack(start + post, acked);
}

// The L2 takes the write (write-allocate). Unloaded acks (512-bit build): a full-line Put mem.full_write_ack_latency
// after the bus took it; a partial one on a line the L2 holds mem.write_ack_latency; same-line Puts then follow one
// per mem.l2_put_serial_cycles. A partial Put to a line the L2 lacks fetches it, and every partial Put to that line
// waits for the fill. A line the CPU's L1 holds is probed out first; the Put acks mem.probe_put_ack_latency after
// the ProbeAckData.
void memory_system_t::write(uint64_t addr, uint32_t bytes, done_t acked, done_t granted) {
  const uint64_t line = addr / line_bytes_;
  const bool full = bytes >= line_bytes_ && addr % line_bytes_ == 0;   // PutFullData vs PutPartialData
  bus_.request(0, eq_.now(), put_beats_, [this, line, full, acked, granted](cycle_t g) {
    trace_ev("put", g);
    if (granted) granted(g);
    probe(line, g, true, [this, line, full, acked, g](cycle_t t) {
      const bool probed = t != g;
      auto it = lines_.find(line);
      if (it == lines_.end()) {
        insert(line, full ? t : NEVER, true);
        if (!full) {   // write-allocate: fetch the line, the Put waits for it
          write_fills_++;
          lines_[line].waiters.push_back([this, line, g, acked](cycle_t r) {
            put_op(line, cmax(r - 1, g + ack_lat_ - put_serial_), put_serial_, acked);
          });
          fetch(line, g + miss_detect_);
          return;
        }
      } else {
        it->second.dirty = true;
        if (it->second.ready == NEVER && !full) {   // its fill is outstanding
          it->second.waiters.push_back([this, line, g, acked](cycle_t r) {
            put_op(line, cmax(r - 1, g + ack_lat_ - put_serial_), put_serial_, acked);
          });
          return;
        }
        if (it->second.ready != NEVER) lru_.splice(lru_.begin(), lru_, it->second.pos);
      }
      if (probed) put_op(line, t, probe_put_ack_, acked);
      else if (full) put_op(line, g, full_ack_lat_, acked);
      else put_op(line, g + ack_lat_ - put_serial_, put_serial_, acked);
    });
  });
}

}  // namespace gperf
