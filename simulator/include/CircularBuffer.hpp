#pragma once

#include <array>
#include <cstddef>

// API-compatible subset of rlogiacco/CircularBuffer used by throttle.cpp.
// Keeping this tiny compatibility header local makes the simulator build
// offline; production still uses the real PlatformIO dependency.
template <typename T, std::size_t Capacity>
class CircularBuffer {
 public:
  void clear() {
    head_ = 0;
    size_ = 0;
  }

  void push(const T& value) {
    if (size_ < Capacity) {
      data_[(head_ + size_) % Capacity] = value;
      ++size_;
      return;
    }
    data_[head_] = value;
    head_ = (head_ + 1) % Capacity;
  }

  bool isEmpty() const { return size_ == 0; }
  bool isFull() const { return size_ == Capacity; }
  std::size_t size() const { return size_; }

  T& operator[](std::size_t index) { return data_[(head_ + index) % Capacity]; }
  const T& operator[](std::size_t index) const {
    return data_[(head_ + index) % Capacity];
  }

  T& first() { return (*this)[0]; }
  const T& first() const { return (*this)[0]; }
  T& last() { return (*this)[size_ - 1]; }
  const T& last() const { return (*this)[size_ - 1]; }

 private:
  std::array<T, Capacity> data_{};
  std::size_t head_ = 0;
  std::size_t size_ = 0;
};
