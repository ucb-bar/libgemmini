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
  X(host, l1d_sets,                 64,   "[knob] CPU L1 D$ sets (mem.host_tracking 2; spike --dc S:W:B terms). WithNHugeCores: 64 x 8 x 64 B") \
  X(host, l1d_ways,                 8,    "[knob] CPU L1 D$ ways (random replacement, spike cache_sim_t's LFSR)") \
  X(host, icache,                   1,    "[knob] with mem.host_tracking 2: also simulate the L1 I$ (traces every fetch: slower spike)") \
  X(host, l1i_sets,                 64,   "[knob] CPU L1 I$ sets (WithNHugeCores: 64 x 8 x 64 B)") \
  X(host, l1i_ways,                 8,    "[knob] CPU L1 I$ ways") \
  X(host, l1_miss_l2_hit,           10,   "[measured] core stall of an L1 miss that hits in the L2 (blocking D$): load/store retire gaps 10-12 in the VCS commit traces (tools/commit_stalls.py: chain_pipelined, attn_vpu_fa, llama_mlp_tiny_db)") \
  X(host, l1_miss_dram,             43,   "[measured] core stall of an L1 miss that also misses in the L2: retire gaps 43-45, same traces") \
  X(host, l1_writeback,             4,    "[to measure] extra stall when the miss evicts a dirty line") \
  X(host, core_model,               1,    "[knob] with mem.host_tracking 2 + host.icache: the in-order pipeline model (perf/host/host_core: register scoreboard + branch penalties)") \
  X(host, lat_load,                 2,    "[measured] int load -> dependent instruction: retire gap 2 (32750 of 33476, llama_mlp_tiny_db; tools/commit_latency.py)") \
  X(host, lat_fp_load,              4,    "[measured] FP load -> dependent: 4 (34781 of 34781)") \
  X(host, lat_fma,                  4,    "[measured] fadd/fmul/fmadd -> dependent: 4 (32641 of 32904)") \
  X(host, lat_fp_misc,              3,    "[measured] fmv / fcvt / fsgnj / compare -> dependent: 3 (49290) or 4 (16543)") \
  X(host, lat_fsqrt,                28,   "[measured] fsqrt -> dependent: 27-28 (unpipelined)") \
  X(host, lat_fdiv,                 28,   "[assumed = fsqrt] fdiv -> dependent (no fdiv in the traces)") \
  X(host, lat_mul,                  4,    "[assumed] int mul -> dependent (no dependent muls in the traces)") \
  X(host, lat_div,                  30,   "[measured, mean] int div/rem -> dependent: 66-67 full, 5-12 early-out (unpipelined)") \
  X(host, br_taken,                 0.7,  "[measured, mean] extra cycles after a taken conditional branch (0.67-0.74; BHT/BTB not modelled)") \
  X(host, br_not_taken,             1.3,  "[measured, mean] after a not-taken one (1.05-1.74: 0 or a 3-cycle mispredict)") \
  X(host, jal,                      0.7,  "[measured, mean] after jal / c.j (0.41-0.97)") \
  X(host, jalr,                     0.4,  "[measured, mean] after jalr / ret (0.25-0.56)") \
  X(host, rocc_resp_cycles,         4,    "RoCC with rd (counter read): command taken -> core resumes (~10/op incl. path, commit trace)") \
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
  X(rs, packed_exact,               0,    "[what-if, not the RTL] packed MX acc: a loop preload's C range and a loop spad store's source range are the tile's own DIM/4 rows (RTL: DIM rows, so a store blocks the next tiles' preloads)") \
  /* --- mesh / execute (ExecuteController.scala, MeshWithDelays.scala) --- */ \
  X(mesh, dim,                      16,   "mesh rows = cols = DIM; must match the kernel build") \
  X(ex, queue_length,               8,    "issued ex commands waiting in the ExecuteController queue (:229)") \
  X(mesh, issue_latency,            3,    "compute issue -> first A row into the mesh (spad req -> a_buf)") \
  X(mesh, fill_latency,             33,   "first A row in -> first output row") \
  X(mesh, commit_latency,           51,   "first A row in -> last accumulator commit") \
  X(mesh, min_rows,                 4,    "minimum rows per request (same-address acc write spacing, :430-438)") \
  X(mesh, drain_extra,              2,    "cycles a CONFIG_EX / CONFIG_SCALE_MEM adds after the mesh drains") \
  X(mesh, lone_preload_cycles,      5,    "a preload with new weights reaching an idle mesh is its own request (dramloop FSDB: +5 then the compute)") \
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
  X(st, pipe_latency,               12,   "store command issued -> its first Put (128x128 FSDB: mvout -> first Put 13)") \
  X(st, write_slack,                8,    "Puts of a store still waiting for the bus when the next store may start (write queues)") \
  X(st, elems_per_acc_read,         32,   "output elements per accumulator read / requantizer beat") \
  /* --- memory system: L2 (InclusiveCache) + DRAM. Calibrated, not derivable from the RTL --- */ \
  X(mem, line_bytes,                64,   "cache line") \
  X(mem, host_tracking,             1,    "[knob] the CPU's memory traffic: 0 none; 1 stores only (approximate L1 below, for probes; fast); 2 spike's cache_sim_t as the L1 D$ (+ I$, host.icache) over every load/store/fetch: probes from its contents and miss stalls on the CPU clock (host.l1_*)") \
  X(mem, host_l1_kib,               32,   "[knob] (host_tracking 1) the CPU's L1 D$: WithNHugeCores = 64 sets x 8 ways x 64 B") \
  X(mem, host_l1_ways,              8,    "[knob] its associativity") \
  X(mem, host_l1_random,            1,    "[knob] replacement: 1 = random (Rocket DCacheParams default, an LFSR), 0 = LRU") \
  X(mem, host_l1_start_full,        1,    "[approximation] the L1 starts full of lines the model never saw (boot/code/data)") \
  X(mem, probe_issue_latency,       5,    "[measured] Put/Get accepted -> the L2's probe to the L1 (dramloop FSDB: min 5)") \
  X(mem, probe_cycles,              7,    "[measured] the L1 (blocking, nMSHRs = 0) takes one probe per 7 cycles: dramloop FSDB, 292 of 304 probe gaps") \
  X(mem, probe_latency,             19,   "[measured] probe -> ProbeAckData back at the L2: 19 for 301 of 301 (dramloop FSDB)") \
  X(mem, probe_put_ack_latency,     7,    "[measured] ProbeAckData -> the probed Put's ack (dramloop FSDB: median 7)") \
  X(mem, l2_kib,                    512,  "L2 capacity (lines start cold)") \
  X(mem, l2_hit_latency,            12,   "Get accepted -> data, L2 hit: mx_mem_bw B_warm_16B lat_req 12") \
  X(mem, client_hit_latency,        10,   "same, for the MX scale / LUT loaders' own client: mx_mem_bw scale_warm (8 slots, 64 Gets, 85 cyc)") \
  X(mem, dram_latency,              20,   "DRAM, unloaded, after its line slot: L2 DRAM-side median 23 (MLP down-loop FSDB); loaded latency is queueing") \
  X(mem, dram_model,                0,    "[knob, optional] 0 = fixed pipe (latency + line slot); 1 = + open-row banks (DRAMSim2 DDR3, testchipip dramsim2_ini). Changes the regression < 1% (13.8)") \
  X(mem, dram_banks,                8,    "[knob] DDR3 NUM_BANKS; scheme2 mapping: bank = line % banks") \
  X(mem, dram_lines_per_row,        256,  "[knob] lines per bank row: NUM_COLS 2048 / BL 8 (row = line / (banks * this))") \
  X(mem, dram_row_miss_cycles,      15,   "[knob] tRP + tRCD = 20 tCK x 1.5 ns at 500 MHz: extra latency of a row miss") \
  X(mem, dram_rc_cycles,            26,   "[knob] tRC = tRAS + tRP = 34 tCK: a bank's next activate after the last") \
  X(mem, dram_max_reads,            0,    "[knob] DRAM reads outstanding at once (FASED maxReads = 16 on FireSim); 0 = unlimited (VCS DRAMSim2: queue 32, never binding)") \
  X(mem, dram_bytes_per_cycle,      8,    "DRAM channel: one 64 B line per 8 cycles at the L2's DRAM side (dramloop FSDB)") \
  X(mem, bus_bytes,                 64,   "[knob] system-bus beat (WithSystemBusWidth/8): a line-sized Put takes line/bus_bytes cycles. 512-bit in every current build; the pre-09-28 MX build was 256-bit (a TLWidthWidget split each Put in two)") \
  X(mem, write_ack_latency,         22,   "[measured] PutPartial accepted -> ack, first Put to an idle line the L2 holds (chain_pipelined FSDB: +18..28)") \
  X(mem, full_write_ack_latency,    10,   "[measured] PutFull accepted -> ack, line not in the CPU's L1: 190 unprobed dramloop Puts, median 10") \
  X(mem, l2_put_serial_cycles,      8,    "[measured] Puts to the same line are handled one at a time, one per 8 cycles (chain FSDB: same-line acks +22/+30/+38/+46)") \
  X(mem, l2_fill_secondary_cycles,  16,   "[approximation] requests that wait on a line's fill resume one per 16 cycles after it lands (Puts: mx_mem_bw mvout_16B ~16; Gets: MLP G,U second touches +43 vs first)") \
  X(mx, block,                      32,   "elements per E8M0 scale block (scaleSize, ConfigsFP.scala:278)") \
  /* --- scale loader, outside the RS (Controller.scala:542-721) --- */ \
  X(scale, start_q,                 4,    "MX_LOAD_SCALES commands queued in the loader") \
  X(scale, slots,                   8,    "outstanding Gets") \
  X(scale, get_bytes,               64,   "largest scale-loader Get (aligned, fits): mx_mem_bw scale_cold = 8 requests for 512 B (FSDB)") \
  X(scale, bytes_per_cycle,         8,    "retire rate: one 8-byte word per cycle") \
  X(scale, row_bubble,              1,    "cycles between rows of a 2-D load") \
  /* --- LUT loader, outside the RS (Controller.scala:723-833): one 8-byte Get at a time --- */ \
  X(lut, fixed_cycles,              2,    "per MX_LOAD_LUT besides the word Gets") \
  /* --- VPU (vpu/Vpu.scala) and SPAD_REQUANT (SpadRequant.scala). Not calibrated yet (plan step 5) --- */ \
  X(vpu, units,                     0,    "VPU units (0 = none; e4m3_vpu preset: 2)") \
  X(vpu, rows_per_cycle,            1,    "scratchpad rows per cycle per unit") \
  X(vpu, latency,                   5,    "row issue -> its write: s0, s1, s2, s3 (EXPSUM build), write reg (vpu/Vpu.scala)") \
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
