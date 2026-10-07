#include "host_core.h"

#include <algorithm>

namespace gperf {

host_core_t::host_core_t(const config_t &c) {
  for (double &l : lat_) l = 1;
  lat_[K_LOAD] = c.host_lat_load;
  lat_[K_FLOAD] = c.host_lat_fp_load;
  lat_[K_FMA] = c.host_lat_fma;
  lat_[K_FMISC] = c.host_lat_fp_misc;
  lat_[K_FDIV] = c.host_lat_fdiv;
  lat_[K_FSQRT] = c.host_lat_fsqrt;
  lat_[K_MUL] = c.host_lat_mul;
  lat_[K_DIV] = c.host_lat_div;
  br_taken_ = c.host_br_taken;
  br_not_ = c.host_br_not_taken;
  jal_ = c.host_jal;
  jalr_ = c.host_jalr;
}

// RV64GC, enough to know each instruction's kind, destination and sources (x0..31 -> 0..31, f0..31 -> 32..63).
host_core_t::dec_t host_core_t::decode(uint32_t b) {
  dec_t d{K_ALU, 4, -1, {-1, -1, -1}};
  auto X = [](uint32_t r) { return (int8_t)r; };
  auto F = [](uint32_t r) { return (int8_t)(32 + r); };
  if ((b & 3) != 3) {   // compressed
    d.len = 2;
    const uint32_t q = b & 3, f3 = (b >> 13) & 7, rdp = 8 + ((b >> 2) & 7), rs1p = 8 + ((b >> 7) & 7);
    const uint32_t r = (b >> 7) & 31, r2 = (b >> 2) & 31;
    if (q == 0) {
      if (f3 == 1) { d.kind = K_FLOAD; d.rd = F(rdp); d.rs[0] = X(rs1p); }
      else if (f3 == 2 || f3 == 3) { d.kind = K_LOAD; d.rd = X(rdp); d.rs[0] = X(rs1p); }
      else if (f3 == 5) { d.kind = K_STORE; d.rs[0] = X(rs1p); d.rs[1] = F(rdp); }
      else if (f3 == 6 || f3 == 7) { d.kind = K_STORE; d.rs[0] = X(rs1p); d.rs[1] = X(rdp); }
      else { d.rd = X(rdp); d.rs[0] = X(2); }   // c.addi4spn
    } else if (q == 1) {
      if (f3 == 5) d.kind = K_JAL;
      else if (f3 == 6 || f3 == 7) { d.kind = K_BR; d.rs[0] = X(rs1p); }
      else if (f3 == 4) { d.rd = X(rs1p); d.rs[0] = X(rs1p); if (((b >> 10) & 3) == 3) d.rs[1] = X(rdp); }
      else { d.rd = X(r); if (f3 != 2 && f3 != 3) d.rs[0] = X(r); }   // c.addi/addiw/li/lui/addi16sp
    } else {
      if (f3 == 1) { d.kind = K_FLOAD; d.rd = F(r); d.rs[0] = X(2); }
      else if (f3 == 2 || f3 == 3) { d.kind = K_LOAD; d.rd = X(r); d.rs[0] = X(2); }
      else if (f3 == 5) { d.kind = K_STORE; d.rs[0] = X(2); d.rs[1] = F(r2); }
      else if (f3 == 6 || f3 == 7) { d.kind = K_STORE; d.rs[0] = X(2); d.rs[1] = X(r2); }
      else if (f3 == 4) {
        if (r2 == 0) { d.kind = K_JALR; d.rs[0] = X(r); if ((b >> 12) & 1) d.rd = X(1); }   // c.jr / c.jalr
        else { d.rd = X(r); d.rs[0] = X(r2); if ((b >> 12) & 1) d.rs[1] = X(r); }          // c.mv / c.add
      } else { d.rd = X(r); d.rs[0] = X(r); }   // c.slli
    }
    if (d.rd == 0) d.rd = -1;
    return d;
  }
  const uint32_t op = b & 0x7f, rd = (b >> 7) & 31, rs1 = (b >> 15) & 31, rs2 = (b >> 20) & 31, rs3 = b >> 27;
  switch (op) {
    case 0x03: d.kind = K_LOAD; d.rd = X(rd); d.rs[0] = X(rs1); break;
    case 0x07: d.kind = K_FLOAD; d.rd = F(rd); d.rs[0] = X(rs1); break;
    case 0x23: d.kind = K_STORE; d.rs[0] = X(rs1); d.rs[1] = X(rs2); break;
    case 0x27: d.kind = K_STORE; d.rs[0] = X(rs1); d.rs[1] = F(rs2); break;
    case 0x2f: d.kind = K_LOAD; d.rd = X(rd); d.rs[0] = X(rs1); d.rs[1] = X(rs2); break;   // AMO
    case 0x13: case 0x1b: d.rd = X(rd); d.rs[0] = X(rs1); break;
    case 0x33: case 0x3b:
      d.rd = X(rd); d.rs[0] = X(rs1); d.rs[1] = X(rs2);
      if ((b >> 25) == 1) d.kind = ((b >> 12) & 7) < 4 ? K_MUL : K_DIV;
      break;
    case 0x37: case 0x17: d.rd = X(rd); break;
    case 0x6f: d.kind = K_JAL; d.rd = X(rd); break;
    case 0x67: d.kind = K_JALR; d.rd = X(rd); d.rs[0] = X(rs1); break;
    case 0x63: d.kind = K_BR; d.rs[0] = X(rs1); d.rs[1] = X(rs2); break;
    case 0x43: case 0x47: case 0x4b: case 0x4f:
      d.kind = K_FMA; d.rd = F(rd); d.rs[0] = F(rs1); d.rs[1] = F(rs2); d.rs[2] = F(rs3); break;
    case 0x53: {
      const uint32_t f5 = b >> 27;
      d.rd = F(rd); d.rs[0] = F(rs1); d.rs[1] = F(rs2); d.kind = K_FMISC;
      if (f5 <= 2) d.kind = K_FMA;
      else if (f5 == 3) d.kind = K_FDIV;
      else if (f5 == 0x0b) { d.kind = K_FSQRT; d.rs[1] = -1; }
      else if (f5 == 0x08) d.rs[1] = -1;                                     // fcvt.s.d / fcvt.d.s
      else if (f5 == 0x14) d.rd = X(rd);                                     // feq / flt / fle
      else if (f5 == 0x18 || f5 == 0x1c) { d.rd = X(rd); d.rs[1] = -1; }     // fcvt.w.* / fmv.x.* / fclass
      else if (f5 == 0x1a || f5 == 0x1e) { d.rs[0] = X(rs1); d.rs[1] = -1; } // fcvt.*.w / fmv.*.x
      break;
    }
    case 0x73: d.rd = X(rd); d.rs[0] = X(rs1); break;   // csr
    default: break;                                      // fence, custom (RoCC: timed by the model itself)
  }
  if (d.rd == 0) d.rd = -1;   // x0
  return d;
}

double host_core_t::insn(uint64_t pc, uint32_t bits) {
  static thread_local std::unordered_map<uint64_t, dec_t> cache;   // the program's code does not change
  auto it = cache.find(pc);
  if (it == cache.end()) it = cache.emplace(pc, decode(bits)).first;
  const dec_t &d = it->second;
  n_[d.kind]++;

  double t = cur_ + 1, pen = 0;
  if (prev_branch_) {   // the previous instruction was a branch / jump: its penalty, by outcome
    if (prev_kind_ == K_BR) pen = pc != prev_next_pc_ ? br_taken_ : br_not_;
    else pen = prev_kind_ == K_JAL ? jal_ : jalr_;
    t += pen;
  }
  for (int8_t s : d.rs)
    if (s >= 0 && s != 0) t = std::max(t, ready_[s]);
  if (d.kind == K_FDIV || d.kind == K_FSQRT) { t = std::max(t, fdiv_free_); fdiv_free_ = t + lat_[d.kind]; }
  if (d.kind == K_DIV) { t = std::max(t, div_free_); div_free_ = t + lat_[d.kind]; }
  const double s = t - (cur_ + 1);
  stall_ += s;
  cur_ = t;
  if (d.rd > 0) ready_[d.rd] = t + lat_[d.kind];
  prev_branch_ = d.kind == K_BR || d.kind == K_JAL || d.kind == K_JALR;
  prev_kind_ = d.kind;
  prev_next_pc_ = pc + d.len;
  return s;
}

}  // namespace gperf
