#ifndef GPERF_TYPES_H
#define GPERF_TYPES_H

#include <cstdint>

namespace gperf {

using cycle_t = int64_t;
constexpr cycle_t NEVER = INT64_MAX / 4;   // "not known yet" (e.g. a preload before its compute is scheduled)

inline cycle_t cmax(cycle_t a, cycle_t b) { return a > b ? a : b; }

// A range of local rows [lo, hi) that a command reads or writes. Scratchpad and accumulator are separate
// address spaces (LocalAddr bit 31), kept apart by the top bit of the 64-bit key.
struct span_t {
  uint64_t lo = 0, hi = 0;
  bool valid = false;
  bool write = false;
};

inline bool overlaps(const span_t &a, const span_t &b) {
  return a.valid && b.valid && a.lo < b.hi && b.lo < a.hi;
}

// LocalAddr (LocalAddr.scala): bit 31 = accumulator, 30 = accumulate, 29 = read full row; all ones = garbage.
struct local_addr_t {
  uint32_t raw;
  bool garbage() const { return raw == 0xFFFFFFFFu; }
  bool is_acc() const { return (raw >> 31) & 1; }
  uint32_t row() const { return raw & 0x1FFFFFFFu; }
  uint64_t key() const { return ((uint64_t)is_acc() << 40) | row(); }
};

inline span_t make_span(local_addr_t a, uint64_t rows, bool write) {
  span_t s;
  if (a.garbage() || rows == 0) return s;
  s.lo = a.key();
  s.hi = a.key() + rows;
  s.valid = true;
  s.write = write;
  return s;
}

enum queue_t { Q_LD = 0, Q_EX = 1, Q_ST = 2, Q_VEC = 3, Q_COUNT = 4 };

}  // namespace gperf

#endif
