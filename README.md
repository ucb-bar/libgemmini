Gemmini Spike ISA Functional Model Extensions
=============================================

This repository builds libgemmini.so, which can be dynamically linked into Spike to support executing custom Gemmini instructions.

To use this, first install a recent version of [spike](https://github.com/riscv-software-src/riscv-isa-sim), and set the `$RISCV` environment variable to the install location.

Usage:
```
make
make install
spike --extension=gemmini <custom_gemmini_program>
```

To see Gemmini commit logs, use the `--log-commits` flag in `spike`.

## MX (microscaling) extension

Spike functional model for the new microscaling RoCC instructions used by
the FP4 / FP6 / FP8 tiled-matmul tests. Equivalence vs. the Python golden
(`fp4_matmul_model.py`, `fp8_matmul_model.py`, `lut_mapping_demo.py`) is
bit-exact for BF16 outputs and code-exact for requantized outputs.

### Ported tests (`gemmini-rocc-tests/bareMetalC`)

All tests are gated on `-DSPIKE_SIM` (added automatically when
`RUNNER=spike`). Build with `./build_spike.sh`.

| Test | Format | Output | Status |
|------|--------|--------|--------|
| matmul_tiled_fp8_64x64                  | FP8 E4M3       | BF16                       | PASS |
| matmul_tiled_fp8_128x128                | FP8 E4M3       | BF16                       | PASS |
| matmul_tiled_fp8_128x128x256            | FP8 E4M3       | BF16                       | PASS |
| matmul_tiled_fp8_128x128_requant        | FP8 E4M3       | FP8 + e8m0 scales          | PASS |
| matmul_tiled_fp4_64x64                  | FP4 E2M1       | BF16                       | PASS |
| matmul_tiled_fp4_64x64_requant          | FP4 E2M1       | FP4 + e8m0 scales          | PASS |
| matmul_tiled_fp4_128x128                | FP4 E2M1       | BF16                       | PASS |
| matmul_tiled_fp4_128x128_requant        | FP4 E2M1       | FP4 + e8m0 scales          | PASS |
| matmul_tiled_fp4_128x128x512            | FP4 E2M1       | BF16                       | PASS |
| matmul_tiled_fp4_128x128x512_requant    | FP4 E2M1       | FP4 + e8m0 scales          | PASS |
| matmul_tiled_fp6_128x128                | FP6 E3M2 (LUT) | BF16                       | PASS |
| matmul_tiled_fp6_128x128x512            | FP6 E3M2 (LUT) | BF16                       | PASS |
| matmul_tiled_fp6_128x128x512_requant    | FP6 E3M2 (LUT) | 4-bit LUT indices + scales | PASS |
| matmul_tiled_fp4_64x64_DRAMMvout        | FP4 E2M1       | DRAM via HW accumulator    | not supported (uses the accumulator → DRAM mvout path which the spike kernel does not model) |

### New custom-ISA instructions

All use opcode `XCUSTOM_ACC` (custom-2). The funct numbers extend the
existing gemmini funct space; only the new ones are listed. Wrapper
macros live in `gemmini-rocc-tests/include/gemmini.h`.

| funct | Name | rs1 | rs2 | Effect |
|-------|------|-----|-----|--------|
| 23 | `MVOUT_SPAD`              | (reserved) | (reserved) | Placeholder; no-op in spike. |
| 24 | `LOOP_WS_CONFIG_SPAD_AB`  | A spad base address | B spad end address | Sets `mx_loop_a_spad`, `mx_loop_b_spad`. Also marks the next `LOOP_WS` as the MX (`mx_loop_ws_spad`) variant. |
| 25 | `LOOP_WS_CONFIG_SPAD_C`   | C spad base address | – | Sets `mx_loop_c_spad`. |
| 26 | `MXQUANT_CONFIG_MVOUT`    | `[32:0]` = `scale_dram` (low 33 bits), `[33:42]` = `tiles_I`, `[42:51]` = `tiles_J`, `[51:60]` = `tiles_K`, `[60]` = `scale_act_sel`, `[61]` = `scale_wgt_sel` | `[15:0]` = `lut_update_granularity` (G) | Configures the requant write-back. Per-row, per-N-group e8m0 scale codes are stored at `scale_dram + m*N_blocks + bi`. |
| 27 | `MX_LOAD_SCALES`          | DRAM byte address of scale block | `[32]` = `sel` (0 = A-row scales, 1 = B-col scales), `[31:0]` = `len` (bytes) | Streams `len` e8m0 codes from DRAM into `mx_scale_a_mem` / `mx_scale_b_mem`. |
| 28 | `MX_READ_SMEM`            | DRAM byte address | `[63:32]` = `num_u16`, `[31:0]` = `smem_word_offset` | Copies `num_u16` 16-bit words from the MX shared memory (`mx_smem`) to DRAM. Used to drain BF16 outputs or packed FP4 / FP6 codes after the matmul. |
| 29 | `MX_LOAD_LUT`             | DRAM byte address | `[33:32]` = `sel` (0 = B, 1 = A, 2 = C), `[31:0]` = `num_luts` | Loads `num_luts × (3 LE uint32 = 96 bits)` from DRAM and unpacks 16 × 6-bit FP6 E3M2 codes per LUT into `mx_lut_a / b / c`. |

The MX matmul itself reuses `LOOP_WS` (funct 8). When the preceding
instruction was `LOOP_WS_CONFIG_SPAD_AB`, the spike handler dispatches
to `mx_loop_ws_spad` instead of the legacy CISC loop; `rs1` is unused
and in `rs2` the upper 32 bits are the C-side spad word offset, while
`rs2[5:0]` is the existing skip-mask byte.

### CONFIG_EX changes (funct 0, CONFIG_EX subtype, `rs1[1:0] == 0b00`)

`gemmini_extended3_config_ex` now packs four MX format fields into
previously-unused bits of `rs1`; the macro signature changed to take
`(act_fmt, wgt_fmt, out_fmt, uselut)` as the last four arguments. The
spike handler reads:

| `rs1` bits | Field | Meaning |
|------------|-------|---------|
| `[5]`     | `uselut`  | When 1, the MX kernel takes the FP6 LUT path. |
| `[11:10]` | `act_fmt` | A-side / activation format: 0 = FP8 E4M3, 1 = FP6 E3M2 (LUT-indexed), 2 = FP4 E2M1. Selects the matmul kernel. |
| `[13:12]` | `wgt_fmt` | B-side / weight format (same encoding). |
| `[15:14]` | `out_fmt` | Output format: 0 = FP8 E4M3 packed, 1 = FP6 (4-bit LUT indices), 2 = FP4 E2M1 packed, 3 = BF16. Selects whether a requant post-pass runs and which projection it uses. |

All other config-EX bits (dataflow, sys_act, strides, transposes, sys
shifts) keep their pre-existing meanings.

### State added to `gemmini_state_t`

`mx_act_fmt`, `mx_wgt_fmt`, `mx_out_fmt`, `mx_use_lut`,
`mx_lut_update_granularity`, `mx_scale_dram`, `mx_tiles_{I,J,K}`,
`mx_scale_{act,wgt}_sel`, `mx_loop_{a,b,c}_spad`, `mx_loop_skips`,
`mx_loop_spad_marker`, plus the buffers `mx_scale_a_mem`,
`mx_scale_b_mem`, `mx_smem`, and per-format LUTs `mx_lut_{a,b,c}` (each
16 codes × max LUTs). All cleared on `reset()`.
