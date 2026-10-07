#ifndef _GEMMINI_PERF_H
#define _GEMMINI_PERF_H

// Timing model of MX-Gemmini, driven by the same RoCC command stream the functional model executes.
// It knows time only, never data: gemmini.cc owns the bit-exact math and the model (perf/) never reads the
// scratchpad, accumulator or DRAM contents. So `both` and `perf` must report identical cycles.
//
// Mode is chosen at run time from $GEMMINI_MODE:
//   func  (default)  bit-exact math only, no timing -- the model as it was
//   perf             timing only; data-moving / compute commands skip their functional work
//   both             bit-exact math and timing
// In perf / both the program's own `fence` and `rdcycle` see modelled time, so a kernel's printed cycle counts
// are the model's. Hardware parameters: perf/config.h. Summary at exit to stderr, or $GEMMINI_PERF_OUT.

#include <cstdint>
#include <memory>
#include <vector>

class processor_t;
struct insn_desc_t;
namespace gperf { class model_t; }

enum class gemmini_mode_t { FUNC, PERF, BOTH };

class gemmini_perf_t {
public:
  gemmini_perf_t();
  ~gemmini_perf_t();
  void reset();
  gemmini_mode_t mode() const { return mode_; }

  // Called for every Gemmini command before the functional dispatch. Returns true when the
  // functional work must be skipped (perf mode, a command that moves or computes data).
  bool on_cmd(processor_t *p, unsigned funct, uint64_t rs1, uint64_t rs2);

  // Start observing the CPU's stores (perf / both, unless mem.host_tracking = 0).
  void attach(processor_t *p);

  // In perf / both: the fence and rdcycle overrides, appended to the extension's instructions.
  static void add_instructions(std::vector<insn_desc_t> &insns);
  // The extension is being registered with processor p (gemmini_t::get_instructions): add_instructions, and in
  // perf / both create the model and start tracing the CPU's stores.
  void on_register(const processor_t &p, std::vector<insn_desc_t> &insns);

private:
  gemmini_mode_t mode_ = gemmini_mode_t::FUNC;
  std::unique_ptr<gperf::model_t> model_;
};

#endif
