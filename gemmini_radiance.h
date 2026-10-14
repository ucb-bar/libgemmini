#ifndef _GEMMINI_RADIANCE_H
#define _GEMMINI_RADIANCE_H

// MX-Gemmini in a Radiance cluster (host.kind 1), for a simulator of the cluster (cyclotron): the functional model
// (gemmini.cc) and the timing model (perf/) behind the RTL's MMIO registers (radiance GemminiTile.scala:360-390).
// The host owns the clock: every call names its current cycle (non-decreasing), and the model has run everything
// before it. Built without spike: make libgemmini_radiance.a.
//
// Registers (offset from the cluster's Gemmini block, GPU-local 0x84000):
//   +0x00 INST     W  a RoCC instruction word, funct = [31:25]: issues {funct, RS1, RS2}. The store is not taken
//                     while the command path is full (io.cmd.ready low): the host retries it later, its warp waits.
//   +0x08 READY    R  1 when an INST store would be taken now
//   +0x10 RS1      W  64 bits (two 32-bit stores: lo at +0x10, hi at +0x14)
//   +0x18 RS2      W  64 bits (lo +0x18, hi +0x1c)
//   +0x20 BUSY     R  io.busy: commands, loop unrollers, scratchpad traffic, scale / LUT loaders
//   +0x28 OCCUPANCY R LOOP_WS accepted and not yet fully unrolled
//   +0x38 RETIRED  R  LOOP_WS whose every command has completed, in issue order
// Scales, LUTs and the requantizer are loaded by Gemmini's own DMAs (MX_LOAD_SCALES, MX_LOAD_LUT, SPAD_REQUANT);
// the RTL's GPU-written scale / requantizer windows are not modelled.

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gemmini_radiance gemmini_radiance_t;

// The SoC around Gemmini, from the host
typedef struct {
  void *ctx;
  // Global memory as Gemmini's DMA sees it: the address the kernel passed (0x1_0000_0000 | GPU address), little
  // endian, bytes = 1, 2, 4 or 8
  uint64_t (*load)(void *ctx, uint64_t addr, unsigned bytes);
  void (*store)(void *ctx, uint64_t addr, uint64_t value, unsigned bytes);
  // The cluster's shared memory = the scratchpad: row r (16 B) at smem + 16 r. Must stay valid while g lives.
  uint8_t *smem;
  uint64_t smem_bytes;
} gemmini_radiance_soc_t;

enum gemmini_radiance_mode {
  GEMMINI_RADIANCE_PERF = 0,   // timing only: commands that move or compute data skip their functional work
  GEMMINI_RADIANCE_BOTH = 1,   // functional (bit-exact) + timing
};

// preset / overrides: the timing parameters (perf/params/config.h): a preset name (nullptr: radiance) and
// "section.name=value,..." (nullptr: none). Returns nullptr if the shared memory is smaller than the scratchpad.
gemmini_radiance_t *gemmini_radiance_new(int mode, const char *preset, const char *overrides,
                                         const gemmini_radiance_soc_t *soc);
void gemmini_radiance_free(gemmini_radiance_t *g);

// An MMIO store of `bytes` at `offset`: 1 = taken, 0 = not now (INST while the command path is full; retry later)
int gemmini_radiance_write(gemmini_radiance_t *g, uint64_t cycle, uint32_t offset, uint64_t value, unsigned bytes);
uint64_t gemmini_radiance_read(gemmini_radiance_t *g, uint64_t cycle, uint32_t offset, unsigned bytes);

void gemmini_radiance_advance(gemmini_radiance_t *g, uint64_t cycle);   // the host's cycle `cycle` has begun
int gemmini_radiance_busy(gemmini_radiance_t *g, uint64_t cycle);       // BUSY
uint64_t gemmini_radiance_next_event(const gemmini_radiance_t *g);     // the model's next event (UINT64_MAX: none)
// End of the run: let Gemmini go idle and write the timing report to `report_path` (nullptr: stderr). Returns the
// cycle its busy signal last dropped.
uint64_t gemmini_radiance_finish(gemmini_radiance_t *g, const char *report_path);

#ifdef __cplusplus
}
#endif

#endif
