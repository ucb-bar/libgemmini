ifndef RISCV
$(error RISCV is unset)
else
$(info Running with RISCV=$(RISCV))
endif

PREFIX ?= $RISCV/

BF16_LIBS = libgemmini_bf16_dim8.so libgemmini_bf16_dim16.so libgemmini_bf16_dim32.so
MXBF16_LIBS = libgemmini_mxbf16_dim16.so libgemmini_mxbf16_dim32.so

# Per mesh row product / accumulator precision: each variant takes MESH_*_PRECISION from a config's gemmini_params.h.
# Override with e.g. `make libgemmini_dim32.so PARAMS_dim32=<path>/gemmini_params.h`.
ROCC_INC ?= ../gemmini-rocc-tests/include
PARAMS_dim16 ?= $(ROCC_INC)/gemmini_params_fp4_vpu.h
PARAMS_dim32 ?= $(ROCC_INC)/gemmini_params_dim32_fp4_vpu.h
PARAMS_dim8 ?= $(ROCC_INC)/gemmini_params_dim8_fp4.h
PARAMS_dim4 ?= $(ROCC_INC)/gemmini_params_dim4_fp4_vpu.h
PARAMS_dim32q ?= $(ROCC_INC)/gemmini_params_dim32_e4m3_quad.h
PARAMS_mxbf16_dim16 ?= $(ROCC_INC)/gemmini_params_e4m3_lut_bf16_dim16.h
PARAMS_mxbf16_dim32 ?= $(ROCC_INC)/gemmini_params_e4m3_lut_bf16_dim32.h

.SECONDEXPANSION:
prec_%.h: $$(PARAMS_$$*)
	grep -h '^#define MESH_\(PROD\|ACC\)_PRECISION ' $< > $@ && test `wc -l < $@` = 2 || \
	  { echo "$<: no MESH_*_PRECISION"; rm -f $@; exit 1; }
# plain BF16 Gemmini (no MX config): BF16 in every row
prec_bf16_dim%.h:
	python3 -c "r='{'+','.join(['{8,7}']*$*)+'}'; print('#define MESH_PROD_PRECISION',r); print('#define MESH_ACC_PRECISION',r)" > $@

.PRECIOUS: prec_%.h

default: libgemmini.so libgemmini_dim32.so libgemmini_dim8.so libgemmini_dim32q.so libgemmini_dim4.so $(BF16_LIBS) $(MXBF16_LIBS)

libgemmini.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc) prec_dim16.h
	g++ -include prec_dim16.h -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 -I $(RISCV)/include -fPIC -O3 $(filter %.cc,$^)

# DIM=32 mesh model, registered as extension "gemmini_dim32" (run: spike --extension=gemmini_dim32 ...)
libgemmini_dim32.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc) prec_dim32.h
	g++ -include prec_dim32.h -DGEMMINI_DIM=32 -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 -I $(RISCV)/include -fPIC -O3 $(filter %.cc,$^)

# DIM=32 native E4M3 quad (64-B spad rows, branch large-mesh-fp8), extension "gemmini_dim32q"
libgemmini_dim32q.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc) prec_dim32q.h
	g++ -include prec_dim32q.h -DGEMMINI_DIM=32 -DGEMMINI_E4M3_NATIVE_QUAD -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 -I $(RISCV)/include -fPIC -O3 $(filter %.cc,$^)

# DIM=4 mesh model (all-bf16 acc, 128 KB spad), extension "gemmini_dim4"
libgemmini_dim4.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc) prec_dim4.h
	g++ -include prec_dim4.h -DGEMMINI_DIM=4 -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 -I $(RISCV)/include -fPIC -O3 $(filter %.cc,$^)

# DIM=8 mesh model (all-bf16 acc), registered as extension "gemmini_dim8" (run: spike --extension=gemmini_dim8 ...)
libgemmini_dim8.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc) prec_dim8.h
	g++ -include prec_dim8.h -DGEMMINI_DIM=8 -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 -I $(RISCV)/include -fPIC -O3 $(filter %.cc,$^)

# Plain BF16 Gemmini (BF16Dim*BaselineConfig), registered as "gemmini_bf16_dim<N>"
# (run: spike --extension=gemmini_bf16_dim16 ...)
libgemmini_bf16_dim%.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc) prec_bf16_dim%.h
	g++ -include prec_bf16_dim$*.h -DGEMMINI_BF16 -DGEMMINI_DIM=$* -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 -I $(RISCV)/include -fPIC -O3 $(filter %.cc,$^)

# MX mesh with full BF16 PE arithmetic (MxE4M3LutBf16Dim*GemminiBoomConfig), extension "gemmini_mxbf16_dim<N>"
libgemmini_mxbf16_dim%.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc) prec_mxbf16_dim%.h
	g++ -include prec_mxbf16_dim$*.h -DGEMMINI_MX_BF16_ARITH -DGEMMINI_DIM=$* -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 -I $(RISCV)/include -fPIC -O3 $(filter %.cc,$^)

.PHONY: install
install: libgemmini.so libgemmini_dim32.so libgemmini_dim8.so $(BF16_LIBS) $(MXBF16_LIBS)
	cp libgemmini.so libgemmini_dim32.so libgemmini_dim8.so $(BF16_LIBS) $(MXBF16_LIBS) $(RISCV)/lib

clean:
	rm -rf *.o *.so prec_*.h
