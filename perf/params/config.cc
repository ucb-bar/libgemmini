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

// Radiance: WithRadianceE4M3MxGemmini in a Radiance cluster (RadianceE4M3MxGemminiSingleClusterTraceConfig, VCS
// simv of 2026-10-07 11:37: after 42c9871 and b0dd6cb). Gemmini is e4m3SingleNoLutMxFPConfig + spad requant + loop
// retire counter: the same standaloneMxFPConfig pipeline as MxGemminiRocketConfig (spad_read_delay 4, acc_latency 3,
// mesh_output_delay 1, tile_latency 0, 2 acc banks, 32 Gets in flight), so the mx_rocket latencies stand until the
// Radiance waveforms say otherwise. What the SoC changes: an MMIO command path, the scratchpad in the cluster's shared
// memory (128 KiB), a 32 KiB accumulator, 32 B cache blocks on a 256-bit system bus.
config_t preset_radiance() {
  config_t c = preset_mx_rocket();
  c.preset = "radiance";
  c.host_kind = 1;
  c.mem_host_tracking = 0;          // Gemmini's DMA probes no CPU L1 (the Muon caches are not coherent with it)
  c.frontend_depth = 6;             // GemminiTile's INST register drives io.cmd: raw_cmd_q, LoopConv, LoopMatmul (3 x 2)
  c.frontend_latency = 3;           // [estimate] one cycle per queue (the RoCC path's 5 included the core side)
  c.spad_bank_rows = 2048;          // sp_capacity = smem 128 KiB over 4 banks (TapeoutSmemConfig)
  c.acc_bank_rows = 128;            // acc_capacity 32 KiB = 256 rows (radiance-kernels nightly/mx.h ACC_ROWS)
  c.acc_total_rows = 256;           //   ... alternating halves of 128 (mx.h ACC_HALF)
  c.mem_line_bytes = 32;            // WithCacheBlockBytes(32) (RadianceBaseConfig)
  c.mem_bus_bytes = 32;             // WithSystemBusWidth(256)
  c.dma_get_bytes = 32;             // dma_maxbytes = CacheBlockBytes
  c.dma_put_bytes = 32;
  c.scale_get_bytes = 32;           // the scale loader's Gets are at most a line
  c.rs_packed_preload_align = 1;    // b0dd6cb is in the simv
  c.rs_scale_cfg_waits_loads = 0;   // 42c9871 is in the simv
  return c;
}

bool config_t::set(const std::string &name, double v) {
#define GPERF_SET(sec, n, def, doc) if (name == #sec "." #n) { sec##_##n = v; return true; }
  GPERF_PARAMS(GPERF_SET)
#undef GPERF_SET
  return false;
}

void config_t::dump(FILE *f) const {
  fprintf(f, "gemmini perf config (preset %s):\n", preset.c_str());
#define GPERF_DUMP(sec, n, def, doc) \
  fprintf(f, "  %-32s %10g%s   %s\n", #sec "." #n, sec##_##n, (sec##_##n != (double)(def)) ? " *" : "  ", doc);
  GPERF_PARAMS(GPERF_DUMP)
#undef GPERF_DUMP
}

config_t config_t::make(const char *p, const char *s) {
  config_t c;
  if (!p || !*p || !strcmp(p, "mx_rocket")) c = preset_mx_rocket();
  else if (!strcmp(p, "e4m3_vpu")) c = preset_e4m3_vpu();
  else if (!strcmp(p, "firesim")) c = preset_firesim();
  else if (!strcmp(p, "e3m2_lut")) c = preset_e3m2_lut();
  else if (!strcmp(p, "radiance")) c = preset_radiance();
  else { fprintf(stderr, "gemmini perf: unknown GEMMINI_PERF_CONFIG=%s (mx_rocket|e4m3_vpu|firesim|e3m2_lut|radiance)\n", p); abort(); }

  if (s) {
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
  return c;
}

config_t config_t::from_env() {
  config_t c = make(getenv("GEMMINI_PERF_CONFIG"), getenv("GEMMINI_PERF_SET"));
  if (getenv("GEMMINI_PERF_DUMP_CONFIG")) c.dump(stderr);
  return c;
}

}  // namespace gperf
