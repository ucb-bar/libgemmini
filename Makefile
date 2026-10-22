ifndef RISCV
$(error RISCV is unset)
else
$(info Running with RISCV=$(RISCV))
endif

PREFIX ?= $RISCV/

default: libgemmini.so libgemmini_dim32.so libgemmini_dim8.so libgemmini_dim32q.so libgemmini_dim4.so

libgemmini.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc)
	g++ -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 -I $(RISCV)/include -fPIC -O3 $(filter %.cc,$^)

# DIM=32 mesh model, registered as extension "gemmini_dim32" (run: spike --extension=gemmini_dim32 ...)
libgemmini_dim32.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc)
	g++ -DGEMMINI_DIM=32 -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 -I $(RISCV)/include -fPIC -O3 $(filter %.cc,$^)

# DIM=32 native E4M3 quad (64-B spad rows, branch large-mesh-fp8), extension "gemmini_dim32q"
libgemmini_dim32q.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc)
	g++ -DGEMMINI_DIM=32 -DGEMMINI_E4M3_NATIVE_QUAD -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 -I $(RISCV)/include -fPIC -O3 $(filter %.cc,$^)

# DIM=4 mesh model (all-bf16 acc, 128 KB spad), extension "gemmini_dim4"
libgemmini_dim4.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc)
	g++ -DGEMMINI_DIM=4 -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 -I $(RISCV)/include -fPIC -O3 $(filter %.cc,$^)

# DIM=8 mesh model (all-bf16 acc), registered as extension "gemmini_dim8" (run: spike --extension=gemmini_dim8 ...)
libgemmini_dim8.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc)
	g++ -DGEMMINI_DIM=8 -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 -I $(RISCV)/include -fPIC -O3 $(filter %.cc,$^)

.PHONY: install
install: libgemmini.so libgemmini_dim32.so libgemmini_dim8.so
	cp libgemmini.so libgemmini_dim32.so libgemmini_dim8.so $(RISCV)/lib

clean:
	rm -rf *.o *.so
