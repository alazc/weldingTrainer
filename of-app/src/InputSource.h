#pragma once

// IInputSource is the seam between "where state comes from" and "what the
// rest of the CPU stack does with it." Three production impls plus a Null:
//
//   * SerialSource     — wraps ProtocolIO RX from a USB-CDC serial port.
//                        ofSerial is forward-declared; the impl lives in
//                        SerialSource.cpp and is only built into the oF app,
//                        keeping this header stdlib-only.
//   * MouseSource      — turns (x_mm, y_mm) injected by the caller (after
//                        mapping the cursor through the workspace transform)
//                        into a snapshot with finite-difference velocity.
//                        No oF dependency.
//   * LogReplaySource  — deterministic playback of a session-log CSV.
//                        Used for: reproducible bug reports, integration
//                        tests, and demo mode without hardware. CSV format
//                        is documented in the loader, kept in sync with
//                        SessionLog.
//
// StateSnapshot is intentionally a plain struct (not atomic). The control
// layer wraps the snapshot in atomic semantics for cross-thread sharing; a
// source is single-thread
// (per-source) and just hands out POD.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// Forward declaration only — the actual ofSerial type lives in oF headers,
// which we deliberately do not include here. SerialSource holds a pointer
// to one and only touches it from SerialSource.cpp.
class ofSerial;

namespace welding {
namespace input {

// --- Shared snapshot type ----------------------------------------------------
// Units are engineering units (mm, mm/s), not q15. The wire decode happens
// inside SerialSource and presents the rest of the stack with floats.
struct StateSnapshot {
  float    x = 0.0f;          // workspace mm
  float    y = 0.0f;
  float    vx = 0.0f;         // mm/s
  float    vy = 0.0f;
  uint16_t max_loop_us = 0;   // peak Arduino inner-loop time over last 100 ms
  uint8_t  status = 0;        // mirrors protocol::StateUp::status bits
};

// --- Interface --------------------------------------------------------------
class IInputSource {
 public:
  virtual ~IInputSource() = default;

  // Called by the consumer at its own cadence. `now_sec` is a monotonic
  // wall-clock in seconds (caller's clock — only relative differences matter).
  //
  // Returns true and fills `out` if a fresh snapshot is available since the
  // last call. Returns false if nothing new (the source is allowed to
  // produce snapshots slower than the polling cadence).
  virtual bool poll(double now_sec, StateSnapshot& out) = 0;

  // True once the source will never produce another snapshot — e.g. replay
  // has run off the end of its CSV. Mouse and Serial sources never exhaust.
  virtual bool isExhausted() const { return false; }
};

// --- MouseSource ------------------------------------------------------------
// The caller (ofApp) is responsible for mapping the OS cursor (pixels) to
// workspace mm before calling `inject`. Velocity is computed as a finite
// difference. To avoid first-poll velocity spikes we wait for the second
// `inject` before reporting any velocity at all.
class MouseSource : public IInputSource {
 public:
  // Push the latest cursor position (in workspace mm) and the wall-clock
  // time it was sampled. Safe to call faster or slower than `poll`.
  void inject(double now_sec, float x_mm, float y_mm);

  bool poll(double now_sec, StateSnapshot& out) override;

  // Test/debug accessors.
  std::size_t injectCount() const { return inject_count_; }

 private:
  bool   has_prev_ = false;
  double prev_t_   = 0.0;
  float  prev_x_   = 0.0f;
  float  prev_y_   = 0.0f;
  StateSnapshot latest_{};
  bool   fresh_    = false;
  std::size_t inject_count_ = 0;
};

// --- LogReplaySource --------------------------------------------------------
// CSV format (header line required, comma-separated, no quoting):
//
//   t_ms,x_mm,y_mm,vx_mms,vy_mms,status
//   0,100.000,100.000,0.0,0.0,0
//   10,100.500,100.000,50.0,0.0,0
//   ...
//
// `t_ms` is sample time relative to the first row (which must be zero or the
// loader rebases it). `status` is the protocol::StateUp::status byte. The
// `max_loop_us` field is not in the CSV — it is not load-bearing for replay
// scenarios and defaults to 0 in the emitted snapshot.
class LogReplaySource : public IInputSource {
 public:
  // Returns true on success. On failure, optionally fills `err` with a
  // human-readable message (line number + reason). On success, `reset()` is
  // implicitly called.
  bool loadFromFile  (const std::string& path,
                      std::string* err = nullptr);
  bool loadFromString(const std::string& csv,
                      std::string* err = nullptr);

  // Rewind playback. The next `poll` will rebase its time origin.
  void reset();

  bool poll(double now_sec, StateSnapshot& out) override;
  bool isExhausted() const override { return exhausted_; }

  std::size_t rowCount() const { return rows_.size(); }

 private:
  struct Row {
    double t_sec;
    float  x, y, vx, vy;
    uint8_t status;
  };
  std::vector<Row> rows_;
  std::size_t      next_idx_   = 0;
  double           t0_         = -1.0;  // -1 = not yet started
  bool             exhausted_  = false;
  bool             ever_emitted_ = false;
};

// --- SerialSource -----------------------------------------------------------
// Wraps ProtocolIO::FrameReassembler over an ofSerial port. The header
// forward-declares ofSerial; the implementation in SerialSource.cpp is the
// only place that includes the oF header. Host tests do not build that .cpp.
class SerialSource : public IInputSource {
 public:
  // `port` must outlive this source. Pass a configured + opened ofSerial.
  explicit SerialSource(ofSerial& port);
  ~SerialSource() override;

  bool poll(double now_sec, StateSnapshot& out) override;

  // Diagnostic counters (populated by the reassembler).
  std::size_t framesAccepted() const;
  std::size_t crcErrors()      const;

 private:
  struct Impl;        // PIMPL — keeps protocol headers out of this .h
  Impl* impl_;
};

}  // namespace input
}  // namespace welding
