#ifndef _GEMMINI_HOST_LOG_H
#define _GEMMINI_HOST_LOG_H

// A gemmini_host_t that records what the functional model does through its host -- every command and every memory
// load / store, in order -- to a text file, for replaying it on another host (tests/radiance_replay.cc):
//   C <funct> <rs1> <rs2>           (hex)
//   L <addr> <bytes> <value>        a load and the value it returned
//   S <addr> <bytes> <value>        a store
// Spike: GEMMINI_FUNC_LOG=<file> (func / both modes).

#include <cstdio>
#include <memory>

#include "gemmini_host.h"

class logging_host_t : public gemmini_host_t {
public:
  logging_host_t(std::unique_ptr<gemmini_host_t> inner, FILE *f) : in_(std::move(inner)), f_(f) {}
  gemmini_host_kind_t kind() const override { return in_->kind(); }
  uint64_t load(uint64_t addr, unsigned bytes) override {
    const uint64_t v = in_->load(addr, bytes);
    fprintf(f_, "L %llx %u %llx\n", (unsigned long long)addr, bytes, (unsigned long long)v);
    return v;
  }
  void store(uint64_t addr, uint64_t value, unsigned bytes) override {
    fprintf(f_, "S %llx %u %llx\n", (unsigned long long)addr, bytes, (unsigned long long)value);
    in_->store(addr, value, bytes);
  }
  uint8_t *shared_scratchpad(size_t *rows) override { return in_->shared_scratchpad(rows); }
  bool log_commits() const override { return in_->log_commits(); }
  void command(unsigned funct, uint64_t rs1, uint64_t rs2) override {
    fprintf(f_, "C %x %llx %llx\n", funct, (unsigned long long)rs1, (unsigned long long)rs2);
  }

private:
  std::unique_ptr<gemmini_host_t> in_;
  FILE *f_;
};

#endif
