#ifndef GPERF_TRACE_H
#define GPERF_TRACE_H

#include <cstdio>

#include "types.h"

namespace gperf {

// Unit-level event trace for comparing against RTL waveforms event by event ("the Nth Get", "the Nth tile").
// Enabled by $GEMMINI_PERF_TRACE (model.cc opens the file); one line per event: ev,<kind>,<cycle>.
//   get / sget / lget   a DMA-reader / scale-loader / LUT-loader Get granted on the bus
//   put                 a StreamWriter Put granted on the bus
//   tile                a mesh request starts feeding rows
//   rdcycle             the program read the cycle counter (phase marks)
inline FILE *&trace_file() {
  static FILE *f = nullptr;
  return f;
}
inline void trace_ev(const char *kind, cycle_t t) {
  if (FILE *f = trace_file()) fprintf(f, "ev,%s,%lld\n", kind, (long long)t);
}
//   l2fill / l2wb       the L2 fetches a line from DRAM / writes a dirty line back (with the line's byte address)
inline void trace_ev_addr(const char *kind, cycle_t t, unsigned long long addr) {
  if (FILE *f = trace_file()) fprintf(f, "ev,%s,%lld,%llx\n", kind, (long long)t, addr);
}

}  // namespace gperf

#endif
