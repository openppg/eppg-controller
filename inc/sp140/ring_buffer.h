#ifndef INC_SP140_RING_BUFFER_H_
#define INC_SP140_RING_BUFFER_H_

#include <stddef.h>

/**
 * Fixed-capacity ring buffer with no heap use and no Arduino dependency,
 * so it builds and tests on the host as-is.
 *
 * Semantics (drop-in for the subset of rlogiacco/CircularBuffer we used):
 * - push() appends at the newest end; when full it overwrites the oldest
 *   element and returns false (true when nothing was overwritten).
 * - Index 0 / first() is the oldest element, size()-1 / last() the newest.
 * - first()/last()/operator[] on an empty buffer, or an index >= size(),
 *   is undefined: callers check size()/isEmpty() first.
 *
 * Not thread-safe: push and read from the same task, or guard externally.
 */
template <typename T, size_t N>
class FixedRingBuffer {
  static_assert(N > 0, "FixedRingBuffer capacity must be > 0");

 public:
  bool push(const T& value) {
    items_[(head_ + count_) % N] = value;
    if (count_ < N) {
      ++count_;
      return true;
    }
    head_ = (head_ + 1) % N;  // Full: the write above replaced the oldest
    return false;
  }

  const T& first() const { return items_[head_]; }
  const T& last() const { return items_[(head_ + count_ - 1) % N]; }
  const T& operator[](size_t index) const { return items_[(head_ + index) % N]; }

  size_t size() const { return count_; }
  static constexpr size_t capacity() { return N; }
  bool isEmpty() const { return count_ == 0; }
  bool isFull() const { return count_ == N; }

  void clear() {
    head_ = 0;
    count_ = 0;
  }

 private:
  T items_[N] = {};
  size_t head_ = 0;
  size_t count_ = 0;
};

#endif  // INC_SP140_RING_BUFFER_H_
