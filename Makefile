ifndef RISCV
$(error RISCV is unset)
else
$(info Running with RISCV=$(RISCV))
endif

PREFIX ?= $RISCV/
# gemmini-rocc-tests, for its bit-exact VPU reference (include/vpu_ref.h)
ROCC_TESTS ?= ../gemmini-rocc-tests
CXXFLAGS_INC = -I $(RISCV)/include -I $(ROCC_TESTS)/include

default: libgemmini.so libgemmini_dim32.so libgemmini_dim8.so

libgemmini.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc)
	g++ -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 $(CXXFLAGS_INC) -fPIC -O3 $(filter %.cc,$^)

# DIM=32 mesh model, registered as extension "gemmini_dim32" (run: spike --extension=gemmini_dim32 ...)
libgemmini_dim32.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc)
	g++ -DGEMMINI_DIM=32 -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 $(CXXFLAGS_INC) -fPIC -O3 $(filter %.cc,$^)

# DIM=8 mesh model (all-bf16 acc), registered as extension "gemmini_dim8" (run: spike --extension=gemmini_dim8 ...)
libgemmini_dim8.so: gemmini.cc gemmini_perf.cc $(wildcard perf/*.cc perf/*/*.cc)
	g++ -DGEMMINI_DIM=8 -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 $(CXXFLAGS_INC) -fPIC -O3 $(filter %.cc,$^)

.PHONY: install
install: libgemmini.so libgemmini_dim32.so libgemmini_dim8.so
	cp libgemmini.so libgemmini_dim32.so libgemmini_dim8.so $(RISCV)/lib

clean:
	rm -rf *.o *.so
