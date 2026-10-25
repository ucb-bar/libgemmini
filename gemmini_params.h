#ifndef GEMMINI_PARAMS_H
#define GEMMINI_PARAMS_H

#include <stdint.h>
#include <limits.h>

#define XCUSTOM_ACC 3
#ifndef GEMMINI_DIM
#define GEMMINI_DIM 16
#endif
#define DIM GEMMINI_DIM
#define ADDR_LEN 32
#define BANK_NUM 4
#if GEMMINI_DIM <= 8
// DIM=8 (and DIM=4: 128 KB of 4-B rows) test-side BANK_ROWS = 65536/DIM = 8192, so b_base = BANK_NUM*BANK_ROWS = 32768 -- the model
// scratchpad (sized BANK_NUM*BANK_ROWS/DIM matrices) must be at least that big.
#define BANK_ROWS 8192
#else
#define BANK_ROWS 4096
#endif
#define MAX_BYTES 64
#ifdef GEMMINI_BF16
// Plain BF16 Gemmini (BF16Dim*BaselineConfig, gemmini_params_bf16_dim*_vpu.h): 256 KB spad, 64 KB FP32 accumulator.
// The model holds elements as float values that are exactly BF16 and stores them as BF16 bits (elem_dram_t) in DRAM.
#define ACC_ROWS ((64 * 1024) / (DIM * 4))
#define MAX_BLOCK_LEN (MAX_BYTES/(DIM*2))
#define MAX_BLOCK_LEN_ACC (MAX_BYTES/(DIM*4))

typedef float elem_t;
typedef uint16_t elem_t_bits;
typedef elem_t_bits elem_dram_t;
static const elem_t elem_t_max = 3.3895313892515355E38;
static const elem_t elem_t_min = -3.3895313892515355E38;
typedef float acc_t;
typedef uint32_t acc_t_bits;
typedef double full_t;

#define ELEM_T_IS_FLOAT
#define ELEM_T_EXP_BITS 8
#define ELEM_T_SIG_BITS 8
#define ACC_T_EXP_BITS 8
#define ACC_T_SIG_BITS 24

#define HAS_MVIN_SCALE
typedef float scale_t;
typedef uint32_t scale_t_bits;

#define HAS_MVIN_ACC_SCALE
typedef float scale_acc_t;
typedef uint32_t scale_acc_t_bits;
#else
#define ACC_ROWS 1024
#ifdef GEMMINI_E4M3_NATIVE_QUAD
// native E4M3 quad (MxDim32E4M3Quad): one spad element = one 16-bit lane = two E4M3 codes (64-B rows)
#define MAX_BLOCK_LEN (MAX_BYTES/(DIM*2))
#define MAX_BLOCK_LEN_ACC (MAX_BYTES/(DIM*4))

typedef int16_t elem_t;
static const elem_t elem_t_max = 32767;
static const elem_t elem_t_min = -32768;
#else
#define MAX_BLOCK_LEN (MAX_BYTES/(DIM*1))
#define MAX_BLOCK_LEN_ACC (MAX_BYTES/(DIM*4))

typedef int8_t elem_t;
static const elem_t elem_t_max = 127;
static const elem_t elem_t_min = -128;
#endif
typedef elem_t elem_dram_t;   // element as stored in DRAM
typedef int32_t acc_t;
typedef int64_t full_t;

#define HAS_MVIN_SCALE
typedef float scale_t;
typedef uint32_t scale_t_bits;

typedef int32_t scale_acc_t;
typedef uint32_t scale_acc_t_bits;
#endif

typedef float acc_scale_t;
typedef uint32_t acc_scale_t_bits;

#define row_align(blocks) __attribute__((aligned(blocks*DIM*sizeof(elem_t))))
#define row_align_acc(blocks) __attribute__((aligned(blocks*DIM*sizeof(acc_t))))

#define MVIN_SCALE_IDENTITY 1.0

#define ACC_SCALE_IDENTITY 1.0

#ifdef GEMMINI_BF16
#define ROUNDING_RIGHT_SHIFT(x, shift) \
    ((x) / (1 << (shift)))
#else
// Rounding right shift equation: https://riscv.github.io/documents/riscv-v-spec/#_vector_fixed_point_rounding_mode_register_vxrm
#define ROUNDING_RIGHT_SHIFT(x, shift) \
    ((shift) > 0 ? (((x) >> (shift)) + \
        (((shift) == 0 ? 0 : (((x) >> ((shift)-1)) & 1)) & \
             ((((shift) <= 1 ? 0 : ((x) & ((1 << ((shift)-1)) - 1))) != 0) | (((x) >> (shift)) & 1)))) : ((x) << (-(shift))))
#endif

#ifdef __cplusplus
#define SAME_TYPE(x) decltype(x)
#else
#define SAME_TYPE(x) typeof(x)
#endif

#define ROUND_NEAR_EVEN(x) \
    ({ const SAME_TYPE(x) x_ = (x); \
         const long long i = x_; \
         const long long next = x_ < 0 ? x_ - 1 : x_ + 1; \
         SAME_TYPE(x) rem = x_ - i; \
         rem = rem < 0 ? -rem : rem; \
         SAME_TYPE(x) result = rem < 0.5 ? i : (rem > 0.5 ? next : ( \
                     i % 2 == 0 ? i : next)); \
         result; })

// Rounding right shift equation: https://riscv.github.io/documents/riscv-v-spec/#_vector_fixed_point_rounding_mode_register_vxrm
#define ROUNDING_RIGHT_SHIFT_BITS(x, shift) \
((shift) > 0 ? (((x) >> (shift)) + \
    (((shift) == 0 ? 0 : (((x) >> ((shift)-1)) & 1)) & \
         ((((shift) <= 1 ? 0 : ((x) & ((1 << ((shift)-1)) - 1))) != 0) | (((x) >> (shift)) & 1)))) : ((x) << (-(shift))))

#ifdef GEMMINI_BF16
#define ACC_SCALE(x, scale) \
    ((x) * (scale))

#define MVIN_SCALE(x, scale) \
    ((x) * (scale))

#define MVIN_SCALE_ACC(x, scale) \
    ((x) * (scale))
#else
#define ACC_SCALE(x, scale) \
    ({float y = ROUND_NEAR_EVEN((x) * (scale)); y > INT8_MAX ? INT8_MAX : (y < INT8_MIN ? INT8_MIN : (acc_t)y);})

#define MVIN_SCALE(x, scale) \
    ({float y = ROUND_NEAR_EVEN((x) * (scale)); y > elem_t_max ? elem_t_max : (y < elem_t_min ? elem_t_min : (elem_t)y);})

#define MVIN_SCALE_ACC(x, scale) (x)
#endif

#define ACC_SCALE_T_IS_FLOAT
#define ACC_SCALE_EXP_BITS 8
#define ACC_SCALE_SIG_BITS 24

#define ACC_READ_SMALL_WIDTH
#define ACC_READ_FULL_WIDTH

#define HAS_FIRST_LAYER_OPTIMIZATIONS

#endif // GEMMINI_PARAMS_H
