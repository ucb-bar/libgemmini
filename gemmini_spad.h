#ifndef _GEMMINI_SPAD_H
#define _GEMMINI_SPAD_H

// The scratchpad: rows of DIM elements. Gemmini's own SRAM (rocket), or a window onto the host's memory (radiance:
// the cluster's shared memory, row r at byte r * DIM -- bank b's rows follow bank b-1's, Controller.scala:226-228).
// spad[r] / spad.at(r) are row views: spad[r][c], spad.at(r).at(c), `auto row = spad[r]; row[c] = x`. Own storage
// behaves as the vector of rows it replaces (zeroed by clear+resize, grows on resize, at() checks, [] does not); a
// host window never resizes or clears (the memory is the host's) and wraps the row address, as the RTL's
// address bits do.

#include <cstddef>
#include <stdexcept>
#include <vector>

#include "gemmini_params.h"

class spad_t {
public:
  class row_t {
  public:
    explicit row_t(elem_t *p) : p_(p) {}
    elem_t &operator[](size_t c) const { return p_[c]; }
    elem_t &at(size_t c) const {
      if (c >= DIM) throw std::out_of_range("spad column");
      return p_[c];
    }
    size_t size() const { return DIM; }

  private:
    elem_t *p_;
  };

  // Back the scratchpad with the host's memory: `rows` rows (a power of two) from `mem`
  void attach(elem_t *mem, size_t rows) {
    base_ = mem;
    rows_ = rows;
    mask_ = rows - 1;
    external_ = true;
  }
  bool external() const { return external_; }

  size_t size() const { return rows_; }
  void clear() {
    if (external_) return;
    own_.clear();
    rows_ = 0;
    base_ = nullptr;
  }
  template <class Fill>
  void resize(size_t rows, const Fill &) {
    if (external_) return;
    own_.resize(rows * DIM, 0);
    rows_ = rows;
    base_ = own_.data();
  }

  row_t operator[](size_t r) const { return row_t(base_ + (r & mask_) * DIM); }
  row_t at(size_t r) const {
    if (!external_ && r >= rows_) throw std::out_of_range("spad row");
    return (*this)[r];
  }

private:
  std::vector<elem_t> own_;
  elem_t *base_ = nullptr;
  size_t rows_ = 0, mask_ = ~(size_t)0;
  bool external_ = false;
};

#endif
