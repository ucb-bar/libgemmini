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

}  // namespace gperf

#endif
