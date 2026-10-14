#ifndef _ROCKET_HOST_H
#define _ROCKET_HOST_H

// gemmini_host_t for Gemmini under a Rocket core in spike: memory through the processor's mmu.

#include <riscv/mmu.h>
#include <riscv/processor.h>

#include "gemmini_host.h"

class rocket_host_t : public gemmini_host_t {
public:
  explicit rocket_host_t(processor_t *p) : p_(p) {}
  gemmini_host_kind_t kind() const override { return gemmini_host_kind_t::ROCKET; }

  uint64_t load(uint64_t addr, unsigned bytes) override {
    mmu_t *m = p_->get_mmu();
    switch (bytes) {
      case 1: return m->load<uint8_t>(addr);
      case 2: return m->load<uint16_t>(addr);
      case 4: return m->load<uint32_t>(addr);
      default: return m->load<uint64_t>(addr);
    }
  }
  void store(uint64_t addr, uint64_t value, unsigned bytes) override {
    mmu_t *m = p_->get_mmu();
    switch (bytes) {
      case 1: m->store<uint8_t>(addr, (uint8_t)value); break;
      case 2: m->store<uint16_t>(addr, (uint16_t)value); break;
      case 4: m->store<uint32_t>(addr, (uint32_t)value); break;
      default: m->store<uint64_t>(addr, value); break;
    }
  }
  bool log_commits() const override { return p_->get_log_commits_enabled(); }

private:
  processor_t *p_;
};

#endif
