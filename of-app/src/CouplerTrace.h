#pragma once

// CouplerTrace.h — Fixed-capacity ring buffer for coupler point trails.
//
// Pure C++17, no openFrameworks, no glm. Reuses Vec2 from FourBar.h.
// Namespace welding::linkage.
//
// Stores a trail of coupler point positions (Vec2). Once full, appending
// new points overwrites the oldest. Logical access (oldest->newest) is
// transparent to the ring-buffer mechanics via at(i).

#include "FourBar.h"
#include <vector>
#include <cstddef>
#include <cassert>

namespace welding {
namespace linkage {

// ---------------------------------------------------------------------------
// CouplerTrace — fixed-capacity ring buffer
// ---------------------------------------------------------------------------
class CouplerTrace {
 public:
  // Construct with a fixed capacity. Buffer starts empty.
  explicit CouplerTrace(std::size_t cap)
      : buffer_(cap), capacity_(cap), head_(0), count_(0) {}

  // Append a point. If at capacity, overwrites the oldest (wrapped).
  void append(Vec2 p) {
    if (capacity_ == 0) return;  // degenerate: capacity 0 does nothing

    buffer_[head_] = p;
    head_ = (head_ + 1) % capacity_;

    if (count_ < capacity_) {
      ++count_;
    }
    // If count_ == capacity_, head_ has wrapped and count_ stays capped.
  }

  // Clear all points. Buffer becomes empty and reusable.
  void clear() {
    head_ = 0;
    count_ = 0;
  }

  // Number of points currently stored (0 .. capacity).
  std::size_t size() const { return count_; }

  // Maximum capacity of the buffer.
  std::size_t capacity() const { return capacity_; }

  // True iff size() == 0.
  bool empty() const { return count_ == 0; }

  // Access point by logical index (0 = oldest, size()-1 = newest).
  // Precondition: i < size().
  Vec2 at(std::size_t i) const {
    assert(i < count_);
    // Logical index i maps to physical index in the buffer.
    // head_ points to the *next* insertion slot (wrapping).
    // The oldest point is at physical index (head_ + capacity_ - count_ + i) % capacity_.
    // Use safe modulo to handle wrap-around correctly.
    std::size_t oldest_idx = (head_ + capacity_ - count_) % capacity_;
    std::size_t physical_idx = (oldest_idx + i) % capacity_;
    return buffer_[physical_idx];
  }

 private:
  std::vector<Vec2> buffer_;     // underlying storage, fixed size
  std::size_t capacity_;         // max capacity (buffer_.size())
  std::size_t head_;             // next insertion slot (wraps at capacity)
  std::size_t count_;            // current number of points stored
};

}  // namespace linkage
}  // namespace welding
