#ifndef _GEMMINI_HOST_H
#define _GEMMINI_HOST_H

// Where Gemmini lives: the SoC around it. The functional model (gemmini.cc) reaches the system's memory only
// through the host, and the timing model (perf/) takes its clock and command path from it (host.kind).
//   rocket    a RoCC accelerator of a Rocket core, in spike: commands are retired custom instructions, the DMA
//             reaches the core's physical memory (spike's mmu), the scratchpad is Gemmini's own SRAM
//   radiance  an MMIO device in a Radiance cluster, in cyclotron: commands are stores to the cluster's Gemmini
//             registers, the DMA reaches global memory, the scratchpad is the cluster's shared memory

#include <cstdint>

enum class gemmini_host_kind_t { ROCKET = 0, RADIANCE = 1 };

class gemmini_host_t {
public:
  virtual ~gemmini_host_t() = default;
  virtual gemmini_host_kind_t kind() const = 0;

  // Memory as Gemmini's DMA sees it (byte address, little endian, bytes = 1, 2, 4 or 8)
  virtual uint64_t load(uint64_t addr, unsigned bytes) = 0;
  virtual void store(uint64_t addr, uint64_t value, unsigned bytes) = 0;
  void store8(uint64_t addr, uint8_t value) { store(addr, value, 1); }
  void store16(uint64_t addr, uint16_t value) { store(addr, value, 2); }

  virtual bool log_commits() const { return false; }   // gemmini.cc's dprintf
};

#endif
