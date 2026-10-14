// MX-Gemmini as an MMIO device in a Radiance cluster (gemmini_radiance.h): the radiance host of the functional
// model and the MMIO front of the timing model.

#include "gemmini_radiance.h"

#include <cstdio>
#include <cstdlib>
#include <memory>

#include "gemmini.h"
#include "gemmini_functs.h"
#include "perf/model.h"

namespace {

// gemmini_host_t for Gemmini in a Radiance cluster: global memory through the host's callbacks, the scratchpad in
// its shared memory
class radiance_host_t : public gemmini_host_t {
public:
  explicit radiance_host_t(const gemmini_radiance_soc_t &soc) : soc_(soc) {}
  gemmini_host_kind_t kind() const override { return gemmini_host_kind_t::RADIANCE; }
  uint64_t load(uint64_t addr, unsigned bytes) override { return soc_.load(soc_.ctx, addr, bytes); }
  void store(uint64_t addr, uint64_t value, unsigned bytes) override { soc_.store(soc_.ctx, addr, value, bytes); }
  uint8_t *shared_scratchpad(size_t *rows) override {
    *rows = (size_t)BANK_NUM * BANK_ROWS;
    return soc_.smem;
  }

private:
  gemmini_radiance_soc_t soc_;
};

// Register offsets (GemminiTile.scala:360-390)
enum : uint32_t {
  R_INST = 0x00, R_READY = 0x08, R_RS1 = 0x10, R_RS2 = 0x18, R_BUSY = 0x20, R_OCCUPANCY = 0x28, R_RETIRED = 0x38,
};

// Write `bytes` of `value` at byte `off` of a 64-bit register
void put(uint64_t *reg, uint32_t off, uint64_t value, unsigned bytes) {
  const uint64_t mask = bytes >= 8 ? ~0ull : ((1ull << (8 * bytes)) - 1);
  *reg = (*reg & ~(mask << (8 * off))) | ((value & mask) << (8 * off));
}

}  // namespace

struct gemmini_radiance {
  int mode;
  std::unique_ptr<gemmini_t> func;          // BOTH and PERF: config commands always run (as spike's perf mode)
  std::unique_ptr<gperf::model_t> model;
  uint64_t rs1 = 0, rs2 = 0;
  uint64_t commands = 0, rejected = 0;
  bool warned[0x1000 / 8] = {};

  void warn(uint32_t offset, const char *what) {
    bool &w = warned[(offset & 0xFFF) / 8];
    if (!w) fprintf(stderr, "gemmini radiance: %s of unmodelled register +0x%x (ignored)\n", what, offset);
    w = true;
  }
};

extern "C" {

gemmini_radiance_t *gemmini_radiance_new(int mode, const char *preset, const char *overrides,
                                         const gemmini_radiance_soc_t *soc) {
  if (!soc || !soc->smem || soc->smem_bytes < (uint64_t)BANK_NUM * BANK_ROWS * DIM * sizeof(elem_t)) return nullptr;
  gperf::config_t c = gperf::config_t::make(preset && *preset ? preset : "radiance", overrides);
  c.host_kind = 1;
  auto *g = new gemmini_radiance;
  g->mode = mode;
  g->func.reset(new gemmini_t);
  g->func->set_host(std::unique_ptr<gemmini_host_t>(new radiance_host_t(*soc)));
  g->func->reset();
  g->model.reset(new gperf::model_t(c));
  return g;
}

void gemmini_radiance_free(gemmini_radiance_t *g) { delete g; }

int gemmini_radiance_write(gemmini_radiance_t *g, uint64_t cycle, uint32_t offset, uint64_t value, unsigned bytes) {
  const gperf::cycle_t t = (gperf::cycle_t)cycle;
  offset &= 0xFFF;
  if (offset >= R_RS1 && offset < R_RS1 + 8) { put(&g->rs1, offset - R_RS1, value, bytes); return 1; }
  if (offset >= R_RS2 && offset < R_RS2 + 8) { put(&g->rs2, offset - R_RS2, value, bytes); return 1; }
  if (offset != R_INST) { g->warn(offset, "write"); return 1; }
  const unsigned funct = (unsigned)(value >> 25) & 0x7F;
  if (!g->model->offer(t, funct, g->rs1, g->rs2)) { g->rejected++; return 0; }
  g->commands++;
  // the functional effect when the command is taken (as spike's `both`: at issue)
  if (g->mode == GEMMINI_RADIANCE_BOTH || !moves_or_computes_data(funct)) g->func->execute(funct, g->rs1, g->rs2);
  return 1;
}

uint64_t gemmini_radiance_read(gemmini_radiance_t *g, uint64_t cycle, uint32_t offset, unsigned bytes) {
  (void)bytes;
  const gperf::cycle_t t = (gperf::cycle_t)cycle;
  switch (offset & 0xFFF) {
    case R_READY: return g->model->cmd_ready(t) ? 1 : 0;
    case R_BUSY: return g->model->busy(t) ? 1 : 0;
    case R_OCCUPANCY: return g->model->loops_pending(t);
    case R_RETIRED: return g->model->loops_retired(t) & 0xFFFFFFFFu;
    default: g->warn(offset, "read"); return 0;
  }
}

void gemmini_radiance_advance(gemmini_radiance_t *g, uint64_t cycle) { g->model->advance_to((gperf::cycle_t)cycle); }

int gemmini_radiance_busy(gemmini_radiance_t *g, uint64_t cycle) { return g->model->busy((gperf::cycle_t)cycle); }

uint64_t gemmini_radiance_next_event(const gemmini_radiance_t *g) {
  const gperf::cycle_t t = g->model->next_event();
  return t == gperf::NEVER ? UINT64_MAX : (uint64_t)t;
}

uint64_t gemmini_radiance_finish(gemmini_radiance_t *g, const char *report_path) {
  g->model->finish();
  FILE *f = report_path ? fopen(report_path, "w") : nullptr;
  if (!f) f = stderr;
  fprintf(f, "gemmini radiance: %llu commands (%llu INST stores retried)\n", (unsigned long long)g->commands,
          (unsigned long long)g->rejected);
  g->model->report(f);
  if (f != stderr) fclose(f);
  return (uint64_t)g->model->busy_until();
}

}  // extern "C"
