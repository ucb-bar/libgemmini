ifndef RISCV
$(error RISCV is unset)
else
$(info Running with RISCV=$(RISCV))
endif

PREFIX ?= $RISCV/

default: libgemmini.so libgemmini_dim32.so

libgemmini.so: gemmini.cc
	g++ -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 -I $(RISCV)/include -fPIC -O3 $^

# DIM=32 mesh model, registered as extension "gemmini_dim32" (run: spike --extension=gemmini_dim32 ...)
libgemmini_dim32.so: gemmini.cc
	g++ -DGEMMINI_DIM=32 -L $(RISCV)/lib -Wl,-rpath,$(RISCV)/lib -shared -o $@ -std=c++17 -I $(RISCV)/include -fPIC -O3 $^

.PHONY: install
install: libgemmini.so libgemmini_dim32.so
	cp libgemmini.so libgemmini_dim32.so $(RISCV)/lib

clean:
	rm -rf *.o *.so
