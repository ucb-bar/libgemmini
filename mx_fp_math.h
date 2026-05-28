#ifndef _GEMMINI_MX_FP_MATH_H
#define _GEMMINI_MX_FP_MATH_H

#include <cstdint>
#include <cmath>

namespace mx {

inline uint16_t f32_to_bf16_rne(float x) {
  union { float f; uint32_t u; } v; v.f = x;
  uint32_t bits = v.u;
  if (((bits >> 23) & 0xFF) == 0xFF) {
    uint32_t bf = bits >> 16;
    if ((bits & 0x7FFFFFu) && !(bf & 0x40)) bf |= 0x40;
    return (uint16_t)(bf & 0xFFFF);
  }
  uint32_t lsb = (bits >> 16) & 1;
  uint32_t rounded = bits + 0x7FFF + lsb;
  return (uint16_t)((rounded >> 16) & 0xFFFF);
}

inline float bf16_to_f32(uint16_t bf) {
  union { uint32_t u; float f; } v; v.u = ((uint32_t)bf) << 16;
  return v.f;
}

inline float bf16_round(float x) { return bf16_to_f32(f32_to_bf16_rne(x)); }

inline int fp_emax(int e_bits, int m_bits) {
  int bias = (1 << (e_bits - 1)) - 1;
  if (e_bits == 4 && m_bits == 3) return bias + 1;
  if (e_bits == 2 && m_bits == 1) return (1 << e_bits) - 1 - bias;
  return bias;
}

inline float mx_product_saturate(float x, int e_bits, int m_bits) {
  int bias = (1 << (e_bits - 1)) - 1;
  bool is_mx_fp8 = (e_bits == 4 && m_bits == 3);
  int emax = is_mx_fp8 ? bias + 1 : bias;
  float scale = (float)(1 << m_bits);
  int max_mant = is_mx_fp8 ? ((1 << m_bits) - 2) : ((1 << m_bits) - 1);
  float max_normal = ldexpf(1.0f + max_mant / scale, emax);
  int sat_man = (1 << m_bits) - 2;
  float sat_val = ldexpf(1.0f + sat_man / scale, bias + 1);
  float ax = fabsf(x);
  if (ax > max_normal) return std::signbit(x) ? -sat_val : sat_val;
  return x;
}

inline float mx_product_quantize_trunc(float x, int e_bits, int frac_bits) {
  if (!std::isfinite(x) || x == 0.0f) return mx_product_saturate(x, e_bits, frac_bits);
  int e;
  float m = frexpf(fabsf(x), &e);
  int E = e - 1;
  float scale = (float)(1 << frac_bits);
  float frac = 2.0f * m - 1.0f;
  float frac_q = floorf(frac * scale) / scale;
  float val = ldexpf(1.0f + frac_q, E);
  float out = std::signbit(x) ? -val : val;
  return mx_product_saturate(out, e_bits, frac_bits);
}

inline float fp_quantize_trunc(float x, int e_bits, int m_bits) {
  if (x == 0.0f || !std::isfinite(x)) return 0.0f;
  bool neg = std::signbit(x);
  float ax = fabsf(x);
  int E;
  frexpf(ax, &E); E -= 1;
  int bias = (1 << (e_bits - 1)) - 1;
  int emin = 1 - bias;
  bool is_mx_fp8 = (e_bits == 4 && m_bits == 3);
  bool is_fp4 = (e_bits == 2 && m_bits == 1);
  int emax = fp_emax(e_bits, m_bits);
  float subnorm_delta = ldexpf(1.0f, emin - m_bits);
  float scale = (float)(1 << m_bits);
  float v;
  if (E < emin) {
    int k = (int)floorf(ax / subnorm_delta);
    if (k > (1 << m_bits) - 1) k = (1 << m_bits) - 1;
    v = k * subnorm_delta;
  } else if (E > emax) {
    float base_max = ldexpf(1.0f, emax);
    float delta_max = base_max / scale;
    int max_mant = is_mx_fp8 ? ((1 << m_bits) - 2) : ((1 << m_bits) - 1);
    v = base_max + max_mant * delta_max;
  } else {
    float base = ldexpf(1.0f, E);
    float delta = base / scale;
    float t = (ax - base) / delta;
    int k = (int)floorf(t);
    int hi = (1 << m_bits) - 1;
    if (is_mx_fp8 && E == emax) hi = (1 << m_bits) - 2;
    if (k > hi) k = hi;
    if (k < 0) k = 0;
    v = base + k * delta;
  }
  (void)is_fp4;
  return neg ? -v : v;
}

inline uint64_t _round_div_pow2_rne_u64(uint64_t n, int shift) {
  if (shift <= 0) return n << (-shift);
  uint64_t q = n >> shift;
  uint64_t rem = n & ((1ull << shift) - 1);
  uint64_t half = 1ull << (shift - 1);
  if (rem > half || (rem == half && (q & 1))) q += 1;
  return q;
}

inline float fp_quantize_rne_scalar(float x, int e_bits, int m_bits) {
  if (std::isnan(x)) return std::nanf("");
  if (std::isinf(x)) return x;
  if (x == 0.0f) return 0.0f;
  bool neg = std::signbit(x);
  float ax = fabsf(x);

  union { float f; uint32_t u; } v32; v32.f = ax;
  uint32_t bits = v32.u;
  uint32_t mant = bits & 0x7FFFFF;
  int e_raw    = (bits >> 23) & 0xFF;
  uint64_t num;
  int exp2;
  if (e_raw == 0) {
    if (mant == 0) return neg ? -0.0f : 0.0f;
    num  = mant;
    exp2 = 1 - 127 - 23;
  } else {
    num  = (1ull << 23) | mant;
    exp2 = e_raw - 127 - 23;
  }
  int p = 63 - __builtin_clzll(num);
  int E = exp2 + p;

  int bias = (1 << (e_bits - 1)) - 1;
  int emin = 1 - bias;
  int emax = bias;

  if (E < emin) {
    int shift = (emin - m_bits) - exp2;
    uint64_t sub_sig = _round_div_pow2_rne_u64(num, shift);
    if (sub_sig == 0) return neg ? -0.0f : 0.0f;
    if (sub_sig >= (1ull << m_bits)) {
      uint64_t total_sig = sub_sig;
      int E_fin = emin;
      if (total_sig >= (1ull << (m_bits + 1))) { total_sig >>= 1; E_fin += 1; }
      if (E_fin > emax) return neg ? -INFINITY : INFINITY;
      float r = ldexpf((float)total_sig, E_fin - m_bits);
      return neg ? -r : r;
    }
    float r = ldexpf((float)sub_sig, emin - m_bits);
    return neg ? -r : r;
  }

  uint64_t total_sig = _round_div_pow2_rne_u64(num, p - m_bits);
  if (total_sig >= (1ull << (m_bits + 1))) { total_sig >>= 1; E += 1; }
  if (E > emax) return neg ? -INFINITY : INFINITY;
  float r = ldexpf((float)total_sig, E - m_bits);
  return neg ? -r : r;
}

inline float fp_quantize_rne(float x, int e_bits, int m_bits) {
  if (e_bits == 8 && m_bits == 7) return bf16_round(x);
  return fp_quantize_rne_scalar(x, e_bits, m_bits);
}

inline float fp_add_exact(float x, float y, int e_bits, int m_bits) {
  if (std::isnan(x) || std::isnan(y)) return std::nanf("");
  if (std::isinf(x) || std::isinf(y)) {
    if (std::isinf(x) && std::isinf(y) && std::signbit(x) != std::signbit(y)) return std::nanf("");
    return std::isinf(x) ? x : y;
  }
  if (x == 0.0f) return y;
  if (y == 0.0f) return x;
  float s = x + y;
  return fp_quantize_rne_scalar(s, e_bits, m_bits);
}

inline float bf16_accum_add(float x, float y) {
  return fp_add_exact(bf16_round(x), bf16_round(y), 8, 7);
}

// Banker's rounding (round-half-to-even) matching Python 3's int(round(x)).
inline int round_half_to_even(float x) {
  float fl = floorf(x);
  float frac = x - fl;
  int fli = (int)fl;
  if (frac < 0.5f) return fli;
  if (frac > 0.5f) return fli + 1;
  return (fli & 1) ? fli + 1 : fli;
}

inline uint8_t fp8_e4m3_to_code(float v) {
  if (v == 0.0f || !std::isfinite(v)) return 0;
  int s = std::signbit(v) ? 1 : 0;
  float av = fabsf(v);
  int E = (int)floorf(log2f(av));
  int bias = 7;
  int emin = -6;
  int emax = 8;
  if (E < emin) return 0;
  int E_used = E, mant;
  if (E > emax) { E_used = emax; mant = 6; }
  else {
    float base = ldexpf(1.0f, E_used);
    float delta = base / 8.0f;
    int k = round_half_to_even((av - base) / delta);
    if (k >= 8) { E_used += 1; k = 0; if (E_used > emax) { E_used = emax; k = 6; } }
    else { int hi = (E_used == emax) ? 6 : 7; if (k > hi) k = hi; if (k < 0) k = 0; }
    mant = k;
  }
  return (uint8_t)((s << 7) | (((E_used + bias) & 0xF) << 3) | (mant & 0x7));
}

inline float fp8_e4m3_decode(uint8_t code) {
  int s = (code >> 7) & 1;
  int e = (code >> 3) & 0xF;
  int m = code & 0x7;
  int bias = 7;
  float val;
  if (e == 0) val = (m / 8.0f) * ldexpf(1.0f, 1 - bias);
  else val = (1.0f + m / 8.0f) * ldexpf(1.0f, e - bias);
  return s ? -val : val;
}

inline float fp4_e2m1_decode(uint8_t code) {
  int s = (code >> 3) & 1;
  int e = (code >> 1) & 0x3;
  int m = code & 0x1;
  float val;
  if (e == 0) val = (m / 2.0f);
  else val = (1.0f + m / 2.0f) * ldexpf(1.0f, e - 1);
  return s ? -val : val;
}

inline uint8_t fpe8m0_decode_exp(uint8_t code) { return code; }
inline float fpe8m0_decode(uint8_t code) {
  if (code == 0xFF) return std::nanf("");
  return ldexpf(1.0f, (int)code - 127);
}

}

#endif
