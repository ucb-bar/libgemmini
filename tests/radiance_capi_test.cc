// gemmini_radiance.h end to end, without cyclotron: one MXFP8 GEMM tile issued as the nightly gemm_mx kernel does
// (radiance-kernels lib/include/nightly/mx.h), through the MMIO registers, with the scratchpad in a host-owned
// shared memory and DRAM behind callbacks. Checks: C (BF16, written by the loop into the scratchpad) against a float
// reference (to the mesh's accumulation precision); INST stores are refused while the command path is full and BUSY / OCCUPANCY / RETIRED behave.
// make radiance_capi_test && ./radiance_capi_test [perf]

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>

#include "../gemmini_radiance.h"

namespace {

constexpr uint64_t HOST = 0x100000000ull;   // mx::host_addr
constexpr uint32_t DIM = 16, I = 4, J = 4, KT = 4, M = I * DIM, N = J * DIM, K = KT * DIM;
constexpr uint64_t A_ADDR = 0x10000, B_ADDR = 0x20000, SC_ADDR = 0x30000, SINK = 0x40000;
constexpr uint32_t A_SP = 0, B_END = 512, C_SP = 1024;   // A, B: I*KT / KT*J tiles of 16 rows; C: M rows x 2N B

std::map<uint64_t, uint8_t> dram;
std::vector<uint8_t> smem(256 << 10);

uint64_t mem_load(void *, uint64_t addr, unsigned bytes) {
  uint64_t v = 0;
  for (unsigned i = 0; i < bytes; i++) {
    auto it = dram.find(addr + i);
    v |= (uint64_t)(it == dram.end() ? 0 : it->second) << (8 * i);
  }
  return v;
}
void mem_store(void *, uint64_t addr, uint64_t value, unsigned bytes) {
  for (unsigned i = 0; i < bytes; i++) dram[addr + i] = (uint8_t)(value >> (8 * i));
}

float e4m3(uint8_t c) {
  const int s = c >> 7, e = (c >> 3) & 15, m = c & 7;
  const float v = e ? std::ldexp(1.0f + m / 8.0f, e - 7) : std::ldexp(m / 8.0f, -6);
  return s ? -v : v;
}
float bf16(uint16_t b) {
  uint32_t u = (uint32_t)b << 16;
  float f;
  memcpy(&f, &u, 4);
  return f;
}

gemmini_radiance_t *g;
uint64_t cycle = 0, refused = 0;

void cmd(uint32_t funct, uint64_t rs1, uint64_t rs2) {   // mx::cmd: five 32-bit stores, one per cycle
  gemmini_radiance_write(g, cycle++, 0x10, (uint32_t)rs1, 4);
  gemmini_radiance_write(g, cycle++, 0x14, (uint32_t)(rs1 >> 32), 4);
  gemmini_radiance_write(g, cycle++, 0x18, (uint32_t)rs2, 4);
  gemmini_radiance_write(g, cycle++, 0x1c, (uint32_t)(rs2 >> 32), 4);
  const uint32_t inst = 0x7Bu | (3u << 12) | (1u << 15) | (2u << 20) | (funct << 25);
  while (!gemmini_radiance_write(g, cycle, 0x00, inst, 4)) { refused++; cycle++; }   // the store waits
  cycle++;
}
uint64_t rd(uint32_t off) { return gemmini_radiance_read(g, cycle, off, 4); }

}  // namespace

int main(int argc, char **argv) {
  const bool perf = argc > 1 && !strcmp(argv[1], "perf");
  srand(1);
  std::vector<uint8_t> A(M * K), B(K * N);
  auto code = [] { return (uint8_t)((rand() & 0x80) | ((5 + rand() % 4) << 3) | (rand() & 7)); };   // |x| 0.25..3.75
  for (auto &a : A) a = code();
  for (auto &b : B) b = code();
  for (uint32_t i = 0; i < M * K; i++) dram[HOST | (A_ADDR + i)] = A[i];
  for (uint32_t i = 0; i < K * N; i++) dram[HOST | (B_ADDR + i)] = B[i];
  for (uint32_t i = 0; i < 1024; i++) dram[HOST | (SC_ADDR + i)] = 127;   // E8M0 1.0
  // A and B stay in DRAM: the scratchpad-mode LOOP_WS loads them itself (RTL LdA / LdB), as gemm_mx relies on

  gemmini_radiance_soc_t soc = {nullptr, mem_load, mem_store, smem.data(), smem.size()};
  g = gemmini_radiance_new(perf ? GEMMINI_RADIANCE_PERF : GEMMINI_RADIANCE_BOTH, nullptr, nullptr, &soc);
  if (!g) { fprintf(stderr, "gemmini_radiance_new failed\n"); return 1; }

  // gemm_mx's setup + one step: CONFIG (FLUSH, LUT off, EX fp8 x fp8 -> bf16, LD A/B strides), scale loads,
  // CONFIG_SCALE_MEM, loop addresses, LOOP_WS (spad mode, loads A and B, computes, stores C to the scratchpad)
  cmd(7, 0, 0);                                                                      // FLUSH
  cmd(30, 0, 0);                                                                     // MX_LUT_DISABLE
  cmd(0, (3ull << 14) | (0ull << 12) | (0ull << 10) | (1ull << 2) | (1ull << 16), 1ull << 48);   // CONFIG_EX
  cmd(0, ((uint64_t)DIM << 16) | (1ull << 8) | (0 << 3) | 1, K);                     // CONFIG_LD A: stride K B
  cmd(0, ((uint64_t)DIM << 16) | (1ull << 8) | (1 << 3) | 1, N);                     // CONFIG_LD B: stride N B
  cmd(27, (HOST | SC_ADDR) & 0xFFFFFFFFFFull, 512);                                  // MX_LOAD_SCALES A
  cmd(27, (HOST | SC_ADDR) & 0xFFFFFFFFFFull, (1ull << 32) | 512);                   // MX_LOAD_SCALES B
  cmd(26, ((uint64_t)KT << 51) | ((uint64_t)J << 42) | ((uint64_t)I << 33) | ((HOST | SINK) & 0x1FFFFFFFFull),
      (1ull << 16) | 1);                                                             // CONFIG_SCALE_MEM, wait_loads
  cmd(10, HOST | A_ADDR, HOST | B_ADDR);                                             // LOOP_WS_CONFIG_ADDRS_AB
  cmd(12, K, N);                                                                     // LOOP_WS_CONFIG_STRIDES_AB
  cmd(9, 0, ((uint64_t)KT << 32) | ((uint64_t)J << 16) | I);                         // LOOP_WS_CONFIG_BOUNDS
  cmd(24, A_SP, B_END);                                                              // LOOP_WS_CONFIG_SPAD_AB
  const uint64_t occ_after_issue = rd(0x28);
  cmd(8, 0, ((uint64_t)C_SP << 32) | (1ull << 9) | (1ull << 5));                    // LOOP_WS: skip ld D only
  const uint64_t occ_after_loop = rd(0x28);
  // back-pressure: CONFIG_LD (B, same stride) eight times, one INST store per cycle (RS1/RS2 latched once). A
  // non-loop command waits in the command path until the loop ahead is unrolled, so the path fills and refuses
  gemmini_radiance_write(g, cycle++, 0x10, ((uint64_t)DIM << 16) | (1ull << 8) | (1 << 3) | 1, 8);
  gemmini_radiance_write(g, cycle++, 0x18, N, 8);
  const uint64_t refused_before = refused;
  bool ready_low = false;
  for (int n = 0; n < 8; n++) {
    while (!gemmini_radiance_write(g, cycle, 0x00, 0x7Bu | (3u << 12) | (1u << 15) | (2u << 20), 4)) {
      ready_low |= rd(0x08) == 0;
      refused++;
      cycle++;
    }
    cycle++;
  }
  if (refused == refused_before || !ready_low) { printf("FAIL: a burst behind a LOOP_WS was never refused\n"); }

  const uint64_t issued = cycle;
  uint64_t busy_polls = 0;
  while (rd(0x20)) { busy_polls++; cycle++; }   // mx::fence
  const uint64_t retired = rd(0x38), occ = rd(0x28);
  const uint64_t done = gemmini_radiance_finish(g, nullptr);
  printf("issue done at %llu, fence done at %llu (%llu INST stores refused, %llu BUSY polls); busy until %llu\n",
         (unsigned long long)issued, (unsigned long long)cycle, (unsigned long long)refused,
         (unsigned long long)busy_polls, (unsigned long long)done);
  printf("OCCUPANCY before LOOP_WS %llu, right after %llu, at fence %llu; RETIRED %llu\n",
         (unsigned long long)occ_after_issue, (unsigned long long)occ_after_loop, (unsigned long long)occ,
         (unsigned long long)retired);
  int bad = (refused == 0);
  if (retired != 1 || occ != 0) { printf("FAIL: RETIRED should be 1 and OCCUPANCY 0 after the fence\n"); bad++; }

  if (!perf) {
    double worst = 0;
    for (uint32_t m = 0; m < M; m++)
      for (uint32_t n = 0; n < N; n++) {
        double ref = 0, mag = 0;
        for (uint32_t k = 0; k < K; k++) {
          const double t = (double)e4m3(A[m * K + k]) * e4m3(B[k * N + n]);
          ref += t;
          mag += std::fabs(t);
        }
        const uint8_t *p = &smem[(size_t)C_SP * DIM + ((size_t)m * N + n) * 2];
        const double got = bf16((uint16_t)(p[0] | (p[1] << 8)));
        // the mesh accumulates in reduced precision (E4M4 .. BF16 by lane, gemmini.cc acc_e / acc_m): the error is
        // relative to the terms' magnitude, not the result's; bit-exactness is tests/radiance_replay.cc's job
        const double err = std::fabs(got - ref) / mag;
        if (err > worst) worst = err;
        if (err > 0.05 && bad < 10) { printf("C[%u][%u] = %g, reference %g (sum |terms| %g)\n", m, n, got, ref, mag); bad++; }
      }
    printf("C vs float reference: worst error / sum|terms| %.3g\n", worst);
  }
  gemmini_radiance_free(g);
  printf(bad ? "FAILED\n" : "PASSED\n");
  return bad ? 1 : 0;
}
