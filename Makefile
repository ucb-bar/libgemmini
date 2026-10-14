# the spike extensions need spike's headers ($RISCV); the radiance library (no spike) does not
RADIANCE_GOALS = libgemmini_radiance.a libgemmini_radiance.so radiance_capi_test radiance_replay
ifneq ($(filter-out $(RADIANCE_GOALS) clean,$(or $(MAKECMDGOALS),default)),)
ifndef RISCV
$(error RISCV is unset)
else
$(info Running with RISCV=$(RISCV))
endif
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

# MX-Gemmini in a Radiance cluster (gemmini_radiance.h), without spike: for cyclotron
RADIANCE_SRCS = gemmini.cc gemmini_radiance.cc $(wildcard perf/*.cc perf/*/*.cc)
RADIANCE_FLAGS = -DGEMMINI_NO_SPIKE -DGEMMINI_RADIANCE -std=c++17 -I $(ROCC_TESTS)/include -fPIC -O3
RADIANCE_OBJS = $(patsubst %.cc,build_radiance/%.o,$(RADIANCE_SRCS))
build_radiance/%.o: %.cc $(wildcard *.h perf/*.h perf/*/*.h)
	@mkdir -p $(dir $@)
	g++ $(RADIANCE_FLAGS) -c -o $@ $<
libgemmini_radiance.a: $(RADIANCE_OBJS)
	ar rcs $@ $^
libgemmini_radiance.so: $(RADIANCE_OBJS)
	g++ -shared -o $@ $^
radiance_capi_test: tests/radiance_capi_test.cc libgemmini_radiance.a
	g++ $(RADIANCE_FLAGS) -o $@ $< libgemmini_radiance.a
# the radiance host with the spike build's geometry: replays a spike GEMMINI_FUNC_LOG bit for bit
REPLAY_OBJS = $(patsubst %.cc,build_replay/%.o,$(RADIANCE_SRCS))
build_replay/%.o: %.cc $(wildcard *.h perf/*.h perf/*/*.h)
	@mkdir -p $(dir $@)
	g++ $(subst -DGEMMINI_RADIANCE,,$(RADIANCE_FLAGS)) -c -o $@ $<
radiance_replay: tests/radiance_replay.cc $(REPLAY_OBJS)
	g++ $(subst -DGEMMINI_RADIANCE,,$(RADIANCE_FLAGS)) -o $@ $^

.PHONY: install
install: libgemmini.so libgemmini_dim32.so libgemmini_dim8.so
	cp libgemmini.so libgemmini_dim32.so libgemmini_dim8.so $(RISCV)/lib

clean:
	rm -rf *.o *.so *.a build_radiance build_replay radiance_capi_test radiance_replay
