#include "reservation_station.h"

#include <algorithm>

namespace gperf {

reservation_station_t::reservation_station_t(const config_t &c, event_queue_t &eq) : eq_(eq) {
  cap_[Q_LD] = (int)c.rs_ld_entries;
  cap_[Q_EX] = (int)c.rs_ex_entries;
  cap_[Q_ST] = (int)c.rs_st_entries;
  cap_[Q_VEC] = (int)c.rs_vec_entries;
}

uint64_t reservation_station_t::alloc(rs_cmd_t c) {
  const uint64_t id = next_id_++;
  ent_t e;
  e.c = std::move(c);
  e.alloc_t = eq_.now();
  for (int q = 0; q < Q_COUNT; q++) {
    if (q == e.c.q && q != Q_VEC) continue;   // ld / ex / st: ordered within their queue by the issue rules
    for (uint64_t oid : order_[q]) {
      ent_t &o = ents_[oid];
      const span_t *mine[4] = {&e.c.a, &e.c.b, &e.c.c, &e.c.d}, *theirs[4] = {&o.c.a, &o.c.b, &o.c.c, &o.c.d};
      bool dep = false;
      for (auto m : mine)
        for (auto t : theirs)
          if (overlaps(*m, *t) && (m->write || t->write)) dep = true;
      if (e.c.q == Q_VEC && q == Q_VEC)   // across vector units: shared read banks, SPAD_REQUANT in order
        dep = dep || (e.c.read_banks & o.c.read_banks) || (e.c.vec == 2 && o.c.vec == 2);
      if (e.c.q == Q_VEC && e.c.vec == 2 && q == Q_ST && o.c.quantized_store) dep = true;
      if (dep) { e.deps++; o.dependents.push_back(id); }
    }
  }
  const queue_t q = e.c.q;
  ents_.emplace(id, std::move(e));
  order_[q].push_back(id);
  allocs_++;
  kick(q);
  return id;
}

void reservation_station_t::kick(queue_t q) {
  if (pending_[q]) return;
  pending_[q] = true;
  const cycle_t t = cmax(eq_.now(), last_issue_[q] + 1);   // one issue per queue per cycle
  eq_.at(t, [this, q] { pending_[q] = false; try_issue(q); });
}

void reservation_station_t::try_issue(queue_t q) {
  if (last_issue_[q] >= eq_.now()) { kick(q); return; }
  uint64_t cand = 0;
  if (q == Q_VEC) {   // out of order: the oldest entry that is ready and whose unit is free
    for (uint64_t id : order_[q]) {
      ent_t &e = ents_[id];
      if (e.issued || e.deps > 0 || (e.c.unit_has_room && !e.c.unit_has_room())) continue;
      cand = id;
      break;
    }
    if (!cand) return;
  } else {
    const bool serial = q == Q_ST;
    for (uint64_t id : order_[q]) {
      ent_t &e = ents_[id];
      if (e.issued) {
        if (serial) return;   // an older one has not completed
        continue;
      }
      cand = id;
      break;
    }
    if (!cand) return;
    ent_t &e = ents_[cand];
    if (e.deps > 0) return;   // re-kicked when a dependency clears
    if (e.c.unit_has_room && !e.c.unit_has_room()) return;   // re-kicked by the unit
  }
  ent_t &e = ents_[cand];
  e.issued = true;
  e.issue_t = eq_.now();
  last_issue_[q] = eq_.now();
  if (e.c.start) e.c.start(cand);
  if (e.c.config && q != Q_EX) complete(cand);
  kick(q);
}

void reservation_station_t::complete(uint64_t id) {
  auto it = ents_.find(id);
  if (it == ents_.end()) return;
  ent_t e = std::move(it->second);
  ents_.erase(it);
  last_complete_ = cmax(last_complete_, eq_.now());
  auto &ord = order_[e.c.q];
  ord.erase(std::find(ord.begin(), ord.end(), id));
  if (trace_cb_) trace_cb_(e.c, e.alloc_t, e.issue_t, eq_.now());
  bool kicked[Q_COUNT] = {false, false, false, false};
  for (uint64_t d : e.dependents) {
    auto jt = ents_.find(d);
    if (jt == ents_.end()) continue;
    if (--jt->second.deps == 0 && !kicked[jt->second.c.q]) { kick(jt->second.c.q); kicked[jt->second.c.q] = true; }
  }
  if (!kicked[e.c.q]) kick(e.c.q);
  if (done_cb_ && e.c.tag >= 0) done_cb_(e.c.tag);
  if (room_cb_) room_cb_();
}

}  // namespace gperf

namespace gperf {
std::string reservation_station_t::describe() const {
  static const char *qn[Q_COUNT] = {"ld", "ex", "st", "vec"};
  std::string out;
  for (int q = 0; q < Q_COUNT; q++) {
    out += std::string(qn[q]) + "=" + std::to_string(order_[q].size());
    if (!order_[q].empty()) {
      const auto &e = ents_.at(order_[q].front());
      out += "(head " + std::string(e.c.what) + (e.issued ? " issued" : "") + " deps " + std::to_string(e.deps) + ")";
    }
    out += " ";
  }
  return out;
}
}  // namespace gperf
