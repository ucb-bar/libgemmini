#ifndef GPERF_DMA_H
#define GPERF_DMA_H

#include <cstdint>
#include <deque>
#include <unordered_map>

#include "../params/config.h"
#include "../sim/event_queue.h"
#include "../memory/memory_system.h"
#include "../memory/scratchpad.h"
#include "../sim/types.h"

namespace gperf {

// StreamReader + BeatMerger (DMA.scala:125-337, BeatMerger.scala). Rows are read in submission order: one Get per
// line a row touches, at most dma.gets_per_cycle, at most dma.max_in_flight outstanding. Responses are merged in
// request order; each writes its useful bytes into the target bank's write port (priority mvin) at one DIM-byte
// row per cycle, and frees its tracker slot when written.
class dma_reader_t {
public:
  dma_reader_t(const config_t &c, event_queue_t &eq, memory_system_t &mem, scratchpad_t &sp);

  // `rows` rows of `row_bytes` at addr, addr+stride, ... into the scratchpad from `spad_row`.
  // done(t): the last byte is in the scratchpad.
  void submit(uint64_t addr, uint32_t rows, uint32_t row_bytes, uint64_t stride, uint32_t spad_row, done_t done);

  uint64_t gets() const { return gets_; }
  uint64_t bytes() const { return bytes_; }
  double mean_latency() const { return gets_ ? (double)lat_sum_ / (double)gets_ : 0; }   // Get accepted -> data

private:
  struct get_t { uint64_t addr; uint32_t useful, spad_row; uint64_t job; bool last; cycle_t back = NEVER; cycle_t sent = 0; };
  void try_issue();
  void try_merge();

  event_queue_t &eq_;
  memory_system_t &mem_;
  scratchpad_t &sp_;
  uint32_t get_bytes_, max_in_flight_, row_bytes_per_cycle_;
  uint64_t lat_sum_ = 0;
  cycle_t interval_;
  std::deque<get_t> to_issue_, issued_;   // issued_: in request order, merged from the front
  uint64_t base_seq_ = 0, next_job_ = 0;
  std::unordered_map<uint64_t, done_t> jobs_;
  uint32_t in_flight_ = 0;
  cycle_t next_issue_ = 0;
  bool issue_pending_ = false, merging_ = false;
  uint64_t gets_ = 0, bytes_ = 0;
};

// StreamWriter: Puts in submission order, one per cycle at most, at most dma.max_puts_in_flight outstanding.
class dma_writer_t {
public:
  dma_writer_t(const config_t &c, event_queue_t &eq, memory_system_t &mem);

  // Rows whose data is ready from `earliest`, one row per `cycles_per_row` cycles. done(t): every Put acked.
  // issued(t), if given: all but `slack` of the job's Puts have been accepted by the bus (the store controller is
  // held until then: its write queues hold `slack` Puts).
  void submit(uint64_t addr, uint32_t rows, uint32_t row_bytes, uint64_t stride, cycle_t earliest,
              double cycles_per_row, done_t done, done_t issued = nullptr, uint32_t slack = 0);

  uint64_t puts() const { return puts_; }

private:
  struct put_t { uint64_t addr; uint32_t bytes; cycle_t ready; uint64_t job; bool fire; };   // fire: the job's issued()
  struct job_t { uint64_t left; done_t done; cycle_t last_ack; done_t issued; };
  void try_issue();

  event_queue_t &eq_;
  memory_system_t &mem_;
  uint32_t put_bytes_, max_in_flight_;
  std::deque<put_t> to_issue_;
  std::unordered_map<uint64_t, job_t> jobs_;
  uint64_t next_job_ = 0;
  uint32_t in_flight_ = 0;
  cycle_t next_issue_ = 0;
  bool issue_pending_ = false;
  uint64_t puts_ = 0;
};

}  // namespace gperf

#endif
