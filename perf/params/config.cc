#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "config.h"

namespace gperf {

config_t preset_mx_rocket() {
  config_t c;
  c.preset = "mx_rocket";
  return c;
}

// MxE3M2PerfGemminiRocketConfig (FP6 E3M2 LUT, standalone MX, current RTL; GemminiConfigs.scala:148-156)
config_t preset_e3m2_lut() {
  config_t c;
  c.preset = "e3m2_lut";
  c.rs_packed_preload_align = 1;
  c.rs_scale_cfg_waits_loads = 0;
  return c;
}

config_t preset_e4m3_vpu() {
  config_t c;
  c.preset = "e4m3_vpu";
  c.rs_ld_entries = 32;   // ConfigsFP.scala:523
  c.vpu_units = 2;        // :520-522
  c.frontend_depth = 8;   // no LoopConv stage (Controller.scala:1115-1124)
  c.rs_packed_preload_align = 1;   // VPU-config RTL is after b0dd6cb
  c.rs_scale_cfg_waits_loads = 0;  // ... and after 42c9871
  return c;
}

// FireSim: chipyard MxGemminiFiresimRocketConfig on BaseXilinxAlveoU250Config (firesim-cy, built 2026-10-05). Every
// clock 1 GHz (one domain, so cycle counts carry over from VCS's 500 MHz); MxE4M3VpuGemminiRocketConfig there has
// WithSystemBusWidth(256) (local change; TLWidthWidget32 in the scratchpad); memory = FASED LatencyPipe (default
// runtime read latency 30, maxReads 16), no LLC model, 64-bit memory bus.
config_t preset_firesim() {
  config_t c = preset_e4m3_vpu();
  c.preset = "firesim";
  c.mem_bus_bytes = 32;
  c.mem_dram_latency = 30;
  c.mem_dram_max_reads = 16;
  // FASED: calibrated on the fixed pipe (13.9), no refresh stalls measured there
  c.mem_dram_model = 0;
  c.mem_dram_refresh_interval = 0;
  return c;
}

// The DIM 4 / 8 / 32 and multi-format DIM 16 VPU builds (GemminiConfigs.scala:198-284, ConfigsFP.scala:395-560): the
// e4m3_vpu RS / VPU / SPAD_REQUANT, but untrimmed -- LoopConv in the command path (frontend depth 10), Im2Col /
// transposer / MMIO requant present -- and the MxE3M2Vpu memory system (4-bank L2, outerLatencyCycles 128).
// Not modelled: the L2 bank count and outer MSHRs (memory_system.h: the model's L2 never limits reads in flight).
static config_t vpu_build(const char *name, int dim) {
  config_t c = preset_e4m3_vpu();
  c.preset = name;
  c.preset_dim = dim;
  c.frontend_depth = 10;                    // LoopConv present (G:109; trimmed only in e4m3Trimmed, C:663)
  if (dim == 4) c.spad_capacity_kib = 128;  // C:467
  if (dim == 32) c.acc_banks = 1;           // C:351
  return c;
}

// name -> preset. The DIM 4 / 8 / 32 single-format builds time the same as their multi-format sibling (formats only
// change which PE modes exist), so each RTL config name maps to one fragment.
static bool preset_by_name(const std::string &n, config_t &c) {
  if (n.empty() || n == "mx_rocket") c = preset_mx_rocket();
  else if (n == "e4m3_vpu") { c = preset_e4m3_vpu(); c.preset_dim = 16; }
  else if (n == "firesim") { c = preset_firesim(); c.preset_dim = 16; }
  else if (n == "e3m2_lut") { c = preset_e3m2_lut(); c.preset_dim = 16; }
  else if (n == "fp4_e3m2_e4m3_vpu") c = vpu_build("fp4_e3m2_e4m3_vpu", 16);     // MxFp4E3M2E4M3Vpu
  else if (n == "dim4_fp4_e3m2_vpu" || n == "dim4_fp4_vpu" || n == "dim4_e3m2_vpu") c = vpu_build(n.c_str(), 4);
  else if (n == "dim8_fp4_e3m2_e4m3_vpu" || n == "dim8_e3m2_vpu" || n == "dim8_fp4_vpu") c = vpu_build(n.c_str(), 8);
  else if (n == "dim32_e4m3_vpu" || n == "dim32_e3m2_vpu" || n == "dim32_fp4_vpu") c = vpu_build(n.c_str(), 32);
  else return false;
  return true;
}

void config_t::finalize() {
  if (preset_dim && preset_dim != (int)mesh_dim) {
    fprintf(stderr, "gemmini perf: preset %s is a DIM %d config but mesh.dim is %g (this library is built for DIM %d)\n",
            preset.c_str(), preset_dim, mesh_dim, GPERF_DEFAULT_DIM);
    abort();
  }
  const double d = mesh_dim, row = spad_row_bytes();
  auto fill = [this](double &p, const char *name, double v) {
    if (p == 0) { p = v; derived.push_back(name); }
  };
  fill(loop_max_block_len, "loop.max_block_len", std::max(1.0, std::floor(dma_get_bytes / row)));
  fill(spad_bank_rows, "spad.bank_rows", spad_capacity_kib * 1024 / (spad_banks * row));
  fill(acc_total_rows, "acc.total_rows", acc_capacity_kib * 1024 * 8 / (64 * d));
  fill(acc_bank_rows, "acc.bank_rows", acc_total_rows / acc_banks);
  fill(mesh_fill_latency, "mesh.fill_latency", 2 * d + 1);
  fill(mesh_commit_latency, "mesh.commit_latency", 3 * d + 3);
  fill(dma_spad_write_bytes_per_cycle, "dma.spad_write_bytes_per_cycle", row);
  fill(st_elems_per_acc_read, "st.elems_per_acc_read", 2 * std::max(d, 16.0));
  fill(sreq_cycles_per_block, "sreq.cycles_per_block", std::max(1.0, 64 / row));
}

bool config_t::set(const std::string &name, double v) {
#define GPERF_SET(sec, n, def, doc) if (name == #sec "." #n) { sec##_##n = v; return true; }
  GPERF_PARAMS(GPERF_SET)
#undef GPERF_SET
  return false;
}

void config_t::dump(FILE *f) const {
  fprintf(f, "gemmini perf config (preset %s, DIM %g; * = changed from the default, a = derived from DIM):\n",
          preset.c_str(), mesh_dim);
  auto mark = [this](const char *name, double v, double def) {
    for (const std::string &s : derived) if (s == name) return " a";
    return v != def ? " *" : "  ";
  };
#define GPERF_DUMP(sec, n, def, doc) \
  fprintf(f, "  %-32s %10g%s   %s\n", #sec "." #n, sec##_##n, mark(#sec "." #n, sec##_##n, (double)(def)), doc);
  GPERF_PARAMS(GPERF_DUMP)
#undef GPERF_DUMP
}

config_t config_t::from_env() {
  config_t c;
  const char *p = getenv("GEMMINI_PERF_CONFIG");
  if (!preset_by_name(p ? p : "", c)) {
    fprintf(stderr, "gemmini perf: unknown GEMMINI_PERF_CONFIG=%s (mx_rocket|e4m3_vpu|firesim|e3m2_lut|"
                    "fp4_e3m2_e4m3_vpu|dim4_{fp4_e3m2,fp4,e3m2}_vpu|dim8_{fp4_e3m2_e4m3,e3m2,fp4}_vpu|"
                    "dim32_{e4m3,e3m2,fp4}_vpu)\n", p);
    abort();
  }

  if (const char *s = getenv("GEMMINI_PERF_SET")) {
    std::string all(s);
    size_t pos = 0;
    while (pos < all.size()) {
      size_t end = all.find(',', pos);
      if (end == std::string::npos) end = all.size();
      std::string kv = all.substr(pos, end - pos);
      size_t eq = kv.find('=');
      if (eq == std::string::npos || !c.set(kv.substr(0, eq), atof(kv.c_str() + eq + 1))) {
        fprintf(stderr, "gemmini perf: bad GEMMINI_PERF_SET entry '%s' (want section.name=value)\n", kv.c_str());
        abort();
      }
      pos = end + 1;
    }
  }
  c.finalize();
  if (getenv("GEMMINI_PERF_DUMP_CONFIG")) c.dump(stderr);
  return c;
}

}  // namespace gperf
