#pragma once

// DebugHistory.h — pure, oF-free ring buffer of debug samples for the time-series
// plots in the pantograph debug overlay. Host-tested; PantographDebugView
// draws from it. Mirrors the JointDebug.h split: math/state here, drawing in the
// oF layer.
//
// One DebugSample is pushed per rendered frame from ofApp (torque, ERM breakdown,
// singularity det, speed/target, off-path + speed error, gating state). The
// buffer is fixed-capacity (no per-frame allocation once filled): when full, the
// oldest sample is overwritten. at(0) is always the oldest retained sample and
// at(size()-1) the newest, so a plot can sweep left->right over time directly.

#include <cstddef>
#include <vector>

namespace welding {
namespace debug {

struct DebugSample {
  float tau1 = 0.0f, tau2 = 0.0f;        // commanded motor torque
  float erm_pwm = 0.0f;                  // 0..255 ERM duty
  float erm_base = 0.0f, erm_speed = 0.0f, erm_blow = 0.0f;  // ERM breakdown (speed cue + signed blow)
  float det = 0.0f;                      // |det| is the singularity measure (mm^2)
  float speed = 0.0f, target = 0.0f;     // |v| and target speed (mm/s)
  float perp_err = 0.0f, vel_err = 0.0f; // off-path distance, along-path speed error
  bool  force_on = false;                // controller emitting force this frame?
  bool  pose_valid = false;              // invKin feasible this frame?
};

// Fixed-capacity ring buffer. push() overwrites the oldest once full; at() is
// indexed oldest(0) -> newest(size()-1) so callers iterate in time order.
class DebugHistory {
 public:
  explicit DebugHistory(std::size_t capacity)
      : cap_(capacity == 0 ? 1 : capacity) {
    buf_.reserve(cap_);
  }

  void push(const DebugSample& s) {
    if (buf_.size() < cap_) {
      buf_.push_back(s);
    } else {
      buf_[head_] = s;            // overwrite oldest
      head_ = (head_ + 1) % cap_;
    }
  }

  std::size_t size()     const { return buf_.size(); }
  std::size_t capacity() const { return cap_; }
  bool        empty()    const { return buf_.empty(); }

  // Oldest-first indexing. i in [0, size()). Once full, at(0) is the sample that
  // will be overwritten next (the oldest), at(size()-1) is the newest.
  const DebugSample& at(std::size_t i) const {
    const std::size_t start = (buf_.size() < cap_) ? 0 : head_;
    return buf_[(start + i) % buf_.size()];
  }
  const DebugSample& latest() const { return at(buf_.size() - 1); }

  void clear() { buf_.clear(); head_ = 0; }

 private:
  std::vector<DebugSample> buf_;
  std::size_t cap_  = 1;
  std::size_t head_ = 0;   // index of the oldest element once full
};

// --- Pure plot mapping (host-tested so a layout tweak can't silently invert a
// plot or run it off its rect) ----------------------------------------------
// A plot rect is (rx, ry, rw, rh) in pixels, y growing downward. plotY maps a
// value in [ymin, ymax] to a pixel y INSIDE the rect, with ymax at the top
// (ry) and ymin at the bottom (ry+rh) — the natural "up is more" orientation.
// Values outside [ymin,ymax] are clamped to the rect edges.
inline float plotY(float v, float ymin, float ymax, float ry, float rh) {
  if (ymax <= ymin) return ry + rh;            // degenerate range -> baseline
  float t = (v - ymin) / (ymax - ymin);        // 0 at ymin, 1 at ymax
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return ry + rh - t * rh;                      // flip: top = ymax
}

// Map sample index i in [0, n) to a pixel x across the rect, oldest at the left
// (rx), newest at the right (rx+rw). n<=1 pins to the left edge.
inline float plotX(std::size_t i, std::size_t n, float rx, float rw) {
  if (n <= 1) return rx;
  return rx + rw * (static_cast<float>(i) / static_cast<float>(n - 1));
}

}  // namespace debug
}  // namespace welding
