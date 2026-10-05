#ifndef GPERF_CONFIG_H
#define GPERF_CONFIG_H

// Hardware parameters of the MX-Gemmini timing model -- the one file to edit, like a Chisel config.
//
// Every parameter is one line of GPERF_PARAMS: section, name, default, and where the number comes from.
// The defaults are MxGemminiRocketConfig (RTL refs are relative to generators/gemmini/src/main/scala/gemmini/).
// A preset (config.cc) changes a few of them, the way a config fragment does. At run time, without rebuilding:
//   GEMMINI_PERF_CONFIG=mx_rocket|e4m3_vpu                         pick a preset (default mx_rocket)
//   GEMMINI_PERF_SET="rs.ex_entries=8,mem.dram_bytes_per_cycle=16"   override any parameter by section.name
//   GEMMINI_PERF_DUMP_CONFIG=1                                     print the resolved parameters at start
//
// Mesh size: the kernel is compiled for one DIM (tile counts and spad addresses are in the instruction stream),
// so mesh.dim must match the DIM the program was built for; changing it means recompiling the kernel.
// Everything else (queues, banks, bandwidths, latencies, unit counts) can change freely under the same program.

#include <cstdio>
#include <string>

// X(section, name, default, meaning / source)
#define GPERF_PARAMS(X) \
  /* --- host (Rocket). Placeholder until the host model (perf_model_plan.md section 9) --- */ \
  X(host, cpi,                      1.0,  "host cycles per retired instruction") \
  X(host, fence_cycles,             2,    "cycles for a fence to retire once Gemmini is idle") \
  /* --- RoCC front end: router -> raw_cmd_q -> LoopConv -> LoopMatmul -> unrolled_cmd (Controller.scala:1102-1196) --- */ \
  X(frontend, depth,                10,   "commands buffered between the core and LoopMatmul (5 x 2-entry queues)") \
  X(frontend, latency,              5,    "cycles from RoCC issue to LoopMatmul input") \
  /* --- LoopMatmul (LoopMatmul.scala) --- */ \
  X(loop, concurrent_loops,         2,    "loop slots; a 3rd LOOP_WS waits for the head slot (:1133)") \
  X(loop, max_block_len,            4,    "DIM-wide blocks per loop-issued mvin (:1101)") \
  X(loop, max_ld_outstanding,       8,    "loop-issued loads in the RS, not completed (rob_overloaded :1198)") \
  X(loop, max_ex_outstanding,       16,   "loop-issued ex commands in the RS, not completed") \
  X(loop, max_st_outstanding,       4,    "loop-issued stores in the RS, not completed") \
  /* --- reservation station (ReservationStation.scala; entries ConfigsFP.scala:232-234) --- */ \
  X(rs, ld_entries,                 8,    "load queue entries") \
  X(rs, ex_entries,                 16,   "execute queue entries") \
  X(rs, st_entries,                 4,    "store queue entries") \
  X(rs, vec_entries,                16,   "vector (VPU / SPAD_REQUANT) queue entries; only with a VPU") \
  /* --- mesh / execute (ExecuteController.scala, MeshWithDelays.scala) --- */ \
  X(mesh, dim,                      16,   "mesh rows = cols = DIM; must match the kernel build") \
  X(ex, queue_length,               8,    "issued ex commands waiting in the ExecuteController queue (:229)") \
  X(mesh, issue_latency,            3,    "compute issue -> first A row into the mesh (spad req -> a_buf)") \
  X(mesh, fill_latency,             33,   "first A row in -> first output row") \
  X(mesh, commit_latency,           51,   "first A row in -> last accumulator commit") \
  X(mesh, min_rows,                 4,    "minimum rows per request (same-address acc write spacing, :430-438)") \
  X(mesh, drain_extra,              2,    "cycles a CONFIG_EX / CONFIG_SCALE_MEM adds after the mesh drains") \
  /* --- scratchpad (Scratchpad.scala; banks x rows ConfigsFP.scala:236,242) --- */ \
  X(spad, banks,                    4,    "scratchpad banks") \
  X(spad, bank_rows,                4096, "rows per bank (one row = DIM bytes)") \
  /* --- accumulator (AccumulatorMem.scala; ConfigsFP.scala:245,343) --- */ \
  X(acc, banks,                     2,    "accumulator banks: stores read one bank while the mesh writes the other") \
  X(acc, bank_rows,                 256,  "accumulator rows per bank") \
  X(acc, total_rows,                512,  "max_acc_addr: loop C bases alternate by total_rows / concurrent_loops") \
  /* --- load path (LoadController.scala, DMA.scala StreamReader, BeatMerger.scala) --- */ \
  X(ld, queue_length,               8,    "issued mvins waiting in the LoadController queue (ConfigsFP.scala:228)") \
  X(ld, cmds_in_flight,             3,    "mvin commands the load tracker overlaps (:84,94)") \
  X(ld, completion_lag,             2,    "last spad row written -> RS completion") \
  X(dma, get_bytes,                 64,   "bytes per TileLink Get (dma_maxbytes; 64-aligned)") \
  X(dma, gets_per_cycle,            1,    "Gets the StreamReader issues per cycle") \
  X(dma, max_in_flight,             32,   "outstanding Gets (max_in_flight_mem_reqs)") \
  X(dma, spad_write_bytes_per_cycle,16,   "BeatMerger: one DIM-byte spad row per cycle") \
  X(dma, put_bytes,                 64,   "bytes per TileLink Put") \
  X(dma, max_puts_in_flight,        32,   "outstanding Puts (StreamWriter)") \
  /* --- store path (StoreController.scala, Scratchpad.scala write path, MxRequantizer.scala) --- */ \
  X(st, queue_length,               2,    "issued stores waiting in the StoreController queue (:230)") \
  X(st, completion_lag,             2,    "last acc/spad read issued -> RS completion (stores complete early)") \
  X(st, requant_latency,            4,    "acc read -> requantizer output -> writer") \
  X(st, elems_per_acc_read,         32,   "output elements per accumulator read / requantizer beat") \
  /* --- memory system: L2 (InclusiveCache) + DRAM. Calibrated, not derivable from the RTL --- */ \
  X(mem, line_bytes,                64,   "cache line") \
  X(mem, l2_kib,                    512,  "L2 capacity (lines start cold)") \
  X(mem, l2_hit_latency,            40,   "Get issue -> data, L2 hit") \
  X(mem, dram_latency,              200,  "Get issue -> data, L2 miss, unloaded") \
  X(mem, dram_bytes_per_cycle,      8,    "DRAM channel bandwidth") \
  X(mem, write_ack_latency,         44,   "Put accepted -> ack; median of 2321 Puts, 128x128 mvout FSDB (p10 21, p90 73)") \
  X(mem, put_cycles,                2,    "bus cycles one Put occupies: the L2 takes a Put every 2nd cycle (128x128 FSDB)") \
  /* --- scale loader, outside the RS (Controller.scala:542-721) --- */ \
  X(scale, start_q,                 4,    "MX_LOAD_SCALES commands queued in the loader") \
  X(scale, slots,                   8,    "outstanding Gets") \
  X(scale, bytes_per_cycle,         8,    "retire rate: one 8-byte word per cycle") \
  X(scale, row_bubble,              1,    "cycles between rows of a 2-D load") \
  /* --- LUT loader, outside the RS (Controller.scala:723-833): one 8-byte Get at a time --- */ \
  X(lut, fixed_cycles,              2,    "per MX_LOAD_LUT besides the word Gets") \
  /* --- VPU (vpu/Vpu.scala) and SPAD_REQUANT (SpadRequant.scala). Not calibrated yet (plan step 5) --- */ \
  X(vpu, units,                     0,    "VPU units (0 = none; e4m3_vpu preset: 2)") \
  X(vpu, rows_per_cycle,            1,    "scratchpad rows per cycle per unit") \
  X(vpu, latency,                   6,    "issue -> last write, beyond the row stream") \
  X(vpu, bank_conflict_cycles,      1,    "extra cycle when src2 is in the same bank as src1") \
  X(sreq, cycles_per_block,         4,    "SPAD_REQUANT cycles per 32-element block") \
  X(sreq, fixed_cycles,             9,    "SPAD_REQUANT start wait + tail")

namespace gperf {

struct config_t {
#define GPERF_FIELD(sec, name, def, doc) double sec##_##name = def;
  GPERF_PARAMS(GPERF_FIELD)
#undef GPERF_FIELD
  std::string preset = "mx_rocket";

  // GEMMINI_PERF_CONFIG / GEMMINI_PERF_SET / GEMMINI_PERF_DUMP_CONFIG, as above. Aborts on an unknown name.
  static config_t from_env();
  bool set(const std::string &dotted_name, double value);   // "rs.ex_entries"; false if unknown
  void dump(FILE *f) const;
};

// Presets: the config fragments. Each starts from the defaults above.
config_t preset_mx_rocket();   // MxGemminiRocketConfig
config_t preset_e4m3_vpu();    // MxE4M3VpuGemminiRocketConfig: VPU x2, SPAD_REQUANT, ld RS 32, no LUT

}  // namespace gperf

#endif
