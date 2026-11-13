#pragma once
#include <array>
#include <cstddef>

namespace platform {
template<class T, size_t Capacity> class RingBuffer {
 public:
  size_t size() const { return size_; }
  bool isEmpty() const { return size_ == 0; }
  bool isFull() const { return size_ == Capacity; }
  void clear() { start_ = size_ = 0; }
  void push(const T& value) {
    values_[(start_ + size_) % Capacity] = value;
    if (size_ < Capacity) ++size_;
    else start_ = (start_ + 1) % Capacity;
  }
  const T& operator[](size_t index) const { return values_[(start_ + index) % Capacity]; }
  T& operator[](size_t index) { return values_[(start_ + index) % Capacity]; }
  const T& first() const { return (*this)[0]; }
  const T& last() const { return (*this)[size_ - 1]; }

 private:
  std::array<T, Capacity> values_{};
  size_t start_ = 0;
  size_t size_ = 0;
};
}  // namespace platform
