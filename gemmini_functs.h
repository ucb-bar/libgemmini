#ifndef _GEMMINI_FUNCTS_H
#define _GEMMINI_FUNCTS_H

// Gemmini command functs the hosts' front ends need (gemmini_perf.cc for rocket, gemmini_radiance.cc), and which of
// them perf mode skips functionally.

// Command functs, mirroring the *_funct constants in gemmini.h (private members there, so not
// reachable from here). Keep the two in step when a command is added.
namespace funct {
  enum : unsigned {
    MVIN2 = 1, MVIN = 2, MVOUT = 3, COMPUTE_PRELOADED = 4, COMPUTE_ACCUMULATED = 5, PRELOAD = 6,
    LOOP_WS = 8, MVIN3 = 14, LOOP_CONV_WS = 15,
    MX_LOAD_SCALES = 27, MX_READ_SMEM = 28, MX_LOAD_LUT = 29,
    VPU_EXEC = 33, SPAD_REQUANT = 34,
  };
}

// The commands whose functional work is skipped in perf mode: everything that touches data (DRAM,
// scratchpad, accumulator, scale/LUT memories) or computes. Config commands are cheap and keep running,
// so the functional state they set stays as it would be.
inline bool moves_or_computes_data(unsigned f) {
  switch (f) {
    case funct::MVIN: case funct::MVIN2: case funct::MVIN3: case funct::MVOUT:
    case funct::PRELOAD: case funct::COMPUTE_PRELOADED: case funct::COMPUTE_ACCUMULATED:
    case funct::LOOP_WS: case funct::LOOP_CONV_WS:
    case funct::MX_LOAD_SCALES: case funct::MX_READ_SMEM: case funct::MX_LOAD_LUT:
    case funct::VPU_EXEC: case funct::SPAD_REQUANT:
      return true;
    default:
      return false;
  }
}

#endif
