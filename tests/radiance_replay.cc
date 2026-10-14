// Replays a spike functional run (GEMMINI_FUNC_LOG, gemmini_host_log.h) through gemmini_radiance.h: the same
// commands as MMIO writes, every load answered with the value spike's run read, every store compared with the one
// it made. The scratchpad is a host buffer (the radiance host's shared-memory scratchpad), the geometry the spike
// build's -- so a byte-identical store stream shows the radiance host is the rocket one, bit for bit.
// make radiance_replay && GEMMINI_FUNC_LOG=x.log spike ... && ./radiance_replay x.log

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "../gemmini_radiance.h"
#include "../gemmini_params.h"

namespace {

struct rec_t { char kind; uint64_t a, b, c; };   // C: funct rs1 rs2; L / S: addr bytes value
std::vector<rec_t> recs;
size_t next_mem = 0;   // the next L / S record the model should produce
uint64_t loads = 0, stores = 0, mismatches = 0;

void next_access(char kind, uint64_t addr, unsigned bytes) {
  while (next_mem < recs.size() && recs[next_mem].kind == 'C') next_mem++;
  if (next_mem >= recs.size() || recs[next_mem].kind != kind || recs[next_mem].a != addr || recs[next_mem].b != bytes) {
    if (mismatches++ < 10)
      fprintf(stderr, "access %c %llx/%u, log has %c %llx/%llu (record %zu)\n", kind, (unsigned long long)addr, bytes,
              next_mem < recs.size() ? recs[next_mem].kind : '-',
              next_mem < recs.size() ? (unsigned long long)recs[next_mem].a : 0ull,
              next_mem < recs.size() ? (unsigned long long)recs[next_mem].b : 0ull, next_mem);
  }
}
uint64_t mem_load(void *, uint64_t addr, unsigned bytes) {
  loads++;
  next_access('L', addr, bytes);
  return next_mem < recs.size() ? recs[next_mem++].c : 0;
}
void mem_store(void *, uint64_t addr, uint64_t value, unsigned bytes) {
  stores++;
  next_access('S', addr, bytes);
  if (next_mem < recs.size()) {
    if (recs[next_mem].c != value && mismatches++ < 10)
      fprintf(stderr, "store %llx: %llx, spike stored %llx\n", (unsigned long long)addr, (unsigned long long)value,
              (unsigned long long)recs[next_mem].c);
    next_mem++;
  }
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) { fprintf(stderr, "usage: %s <GEMMINI_FUNC_LOG file>\n", argv[0]); return 2; }
  FILE *f = fopen(argv[1], "r");
  if (!f) { perror(argv[1]); return 2; }
  char k;
  unsigned long long a, b, c;
  while (fscanf(f, " %c %llx %llx %llx", &k, &a, &b, &c) == 4) recs.push_back({k, a, b, c});
  fclose(f);

  std::vector<uint8_t> smem((size_t)BANK_NUM * BANK_ROWS * DIM);
  gemmini_radiance_soc_t soc = {nullptr, mem_load, mem_store, smem.data(), smem.size()};
  gemmini_radiance_t *g = gemmini_radiance_new(GEMMINI_RADIANCE_BOTH, nullptr, nullptr, &soc);
  if (!g) { fprintf(stderr, "gemmini_radiance_new failed\n"); return 2; }
  uint64_t cycle = 0, cmds = 0;
  for (const rec_t &r : recs) {
    if (r.kind != 'C') continue;
    gemmini_radiance_write(g, cycle++, 0x10, r.b, 8);
    gemmini_radiance_write(g, cycle++, 0x18, r.c, 8);
    while (!gemmini_radiance_write(g, cycle, 0x00, (uint64_t)r.a << 25, 4)) cycle++;
    cycle++;
    cmds++;
  }
  gemmini_radiance_finish(g, "/dev/null");
  gemmini_radiance_free(g);
  size_t left = 0;
  for (size_t i = next_mem; i < recs.size(); i++) left += recs[i].kind != 'C';
  printf("%llu commands, %llu loads, %llu stores replayed; %llu mismatches, %zu logged accesses not reached\n",
         (unsigned long long)cmds, (unsigned long long)loads, (unsigned long long)stores,
         (unsigned long long)mismatches, left);
  const bool ok = !mismatches && !left && stores;
  printf(ok ? "IDENTICAL\n" : "DIFFERENT\n");
  return ok ? 0 : 1;
}
