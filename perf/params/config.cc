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

config_t preset_e4m3_vpu() {
  config_t c;
  c.preset = "e4m3_vpu";
  c.rs_ld_entries = 32;   // ConfigsFP.scala:523
  c.vpu_units = 2;        // :520-522
  c.frontend_depth = 8;   // no LoopConv stage (Controller.scala:1115-1124)
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

config_t config_t::from_env() {
  config_t c;
  const char *p = getenv("GEMMINI_PERF_CONFIG");
  if (!p || !*p || !strcmp(p, "mx_rocket")) c = preset_mx_rocket();
  else if (!strcmp(p, "e4m3_vpu")) c = preset_e4m3_vpu();
  else { fprintf(stderr, "gemmini perf: unknown GEMMINI_PERF_CONFIG=%s (mx_rocket|e4m3_vpu)\n", p); abort(); }

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
  if (getenv("GEMMINI_PERF_DUMP_CONFIG")) c.dump(stderr);
  return c;
}

}  // namespace gperf
