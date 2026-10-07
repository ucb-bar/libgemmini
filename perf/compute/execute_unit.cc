#include "execute_unit.h"

#include <memory>

namespace gperf {

execute_unit_t::execute_unit_t(const config_t &c, event_queue_t &eq, scratchpad_t &sp, accumulator_t &acc,
                               reservation_station_t &rs)
    : eq_(eq), sp_(sp), acc_(acc), rs_(rs), queue_len_((size_t)c.ex_queue_length), dim_((uint32_t)c.mesh_dim),
      min_rows_((uint32_t)c.mesh_min_rows), issue_lat_((cycle_t)c.mesh_issue_latency),
      fill_lat_((cycle_t)c.mesh_fill_latency), commit_lat_((cycle_t)c.mesh_commit_latency),
      drain_extra_((cycle_t)c.mesh_drain_extra), lone_preload_((cycle_t)c.mesh_lone_preload_cycles) {}

void execute_unit_t::accept(uint64_t rs_id, ex_cmd_t c) {
  q_.push_back({rs_id, std::move(c)});
  process();
}

void execute_unit_t::process() {
  if (busy_now_ || q_.empty()) return;
  item_t head = q_.front();
  switch (head.c.kind) {
    case ex_cmd_t::PRELOAD:
      if (q_.size() < 2) return;   // wait for the compute it pairs with
      if (q_[1].c.kind == ex_cmd_t::COMPUTE) {
        const item_t comp = q_[1];
        q_.pop_front();
        q_.pop_front();
        run_tile(&head, comp);
      } else {   // a preload with no compute after it: just loads the weights
        q_.pop_front();
        rs_.kick(Q_EX);
        weights_ = head.c.b;
        eq_.at(eq_.now() + dim_, [this, id = head.id] { rs_.complete(id); });
        process();
      }
      return;
    case ex_cmd_t::COMPUTE:
      q_.pop_front();
      run_tile(nullptr, head);
      return;
    case ex_cmd_t::CONFIG: {
      if (tiles_in_flight_ > 0) return;   // drain first: re-run on the last commit
      q_.pop_front();
      rs_.kick(Q_EX);
      busy_now_ = true;
      auto finish = [this, id = head.id, exec = head.c.on_execute](cycle_t t) {
        if (exec) exec();
        eq_.at(t + drain_extra_, [this, id] {
          rs_.complete(id);
          busy_now_ = false;
          process();
        });
      };
      if (head.c.wait_for) head.c.wait_for(finish);
      else finish(eq_.now());
      return;
    }
  }
}

void execute_unit_t::run_tile(const item_t *pre, const item_t &comp) {
  rs_.kick(Q_EX);   // queue room
  busy_now_ = true;
  uint32_t rows = comp.c.rows ? comp.c.rows : dim_;
  if (rows < min_rows_) rows = min_rows_;
  // back to back with the previous compute: no bubble; from idle: the spad request -> a_buf latency
  const bool new_weights = pre && !pre->c.b.garbage() && !pre->c.b.is_acc();
  // back to back with the previous compute: no bubble, the preload rides under it; from idle: the spad request ->
  // a_buf latency, and new weights go in as their own short mesh request first
  const bool idle = eq_.now() > last_feed_end_;
  const cycle_t start = !idle ? eq_.now() : eq_.now() + issue_lat_ + (new_weights ? lone_preload_ : 0);
  if (pre) weights_ = pre->c.b;
  const uint64_t comp_id = comp.id, pre_id = pre ? pre->id : 0;
  const local_addr_t c = pre ? pre->c.c : local_addr_t{0xFFFFFFFFu};
  auto left = std::make_shared<int>(new_weights ? 2 : 1);
  auto fed = [this, rows, start, comp_id, pre_id, c, left](cycle_t t) {
    if (--*left > 0) return;
    busy_ += (uint64_t)(t - start);
    trace_ev("tile", t - rows);
    last_feed_end_ = t;
    tiles_++;
    rs_.complete(comp_id);
    const cycle_t first_out = t - rows + fill_lat_, commit = t - rows + commit_lat_;
    if (!c.garbage()) acc_.port(c.row()).request(ACC_MESH, first_out, rows, [](cycle_t) {});
    tiles_in_flight_++;
    eq_.at(commit, [this, pre_id] {
      if (pre_id) rs_.complete(pre_id);
      tiles_in_flight_--;
      process();
    });
    busy_now_ = false;
    process();
  };
  sp_.read_port(comp.c.a.row()).request(SP_EX, start, rows, fed);
  if (new_weights) sp_.read_port(pre->c.b.row()).request(SP_EX, start, rows, fed);
}

}  // namespace gperf
