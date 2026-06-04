#pragma once

// CPU-side concurrency: one writer (RX thread) and two readers
// (outer-loop thread, render thread + session log) share the latest
// StateSnapshot through a SnapshotSlot (seqlock). The outer-loop thread
// reads the snapshot, asks the active Path for nearest-point, runs a
// PD-on-perpendicular / P-on-tangential controller, and emits CMD_DOWN
// via an IMotorSink. The heartbeat thread emits HEARTBEAT at 50 Hz on a
// dedicated cadence.
//
// Test seam: every thread's body is a public `tick(now_sec)`
// method. Tests construct the object, hand-feed an IInputSource and a
// fake Path*, then call the ticks themselves with a fake clock. Live
// mode (`start()`) spawns three std::threads that each loop on
// sleep_until + tick. No test ever waits on real wall-clock cadence.
//
// Why a seqlock: StateSnapshot is 24 bytes after alignment
// (4 floats + uint16 + uint8 + 1 padding byte, padded to next 4-byte
// boundary). MSVC's std::atomic is lock-free only up to 16 bytes — a
// std::atomic<StateSnapshot> here silently degrades to a mutex-backed
// implementation, defeating the point of having an "atomic snapshot."
// Seqlock is the textbook single-producer / multi-consumer answer:
// reader retries on detected torn read, writer never blocks.

#include "InputSource.h"
#include "MotorSink.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <thread>

namespace welding {

namespace path { class Path; }
namespace log  { class SessionLog; }  // optional RX-thread log sink

namespace control {

// =============================================================================
// SnapshotSlot — single-producer / multi-consumer seqlock for StateSnapshot.
// =============================================================================
//
// Writer: bump seq to odd, write data, bump seq to even. Reader: snapshot
// seq, copy data, re-read seq; retry if any of (start was odd, end differs
// from start). The std::atomic_thread_fence calls keep the data writes from
// reordering past the seq increments under -O2.
class SnapshotSlot {
 public:
  void write(const input::StateSnapshot& s) {
    const uint64_t seq = seq_.load(std::memory_order_relaxed) + 1;
    seq_.store(seq, std::memory_order_relaxed);              // now odd
    std::atomic_thread_fence(std::memory_order_release);
    data_ = s;
    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(seq + 1, std::memory_order_release);          // now even
    ++writes_;
  }

  // Returns true if a coherent read landed within `max_tries`. With a
  // single producer at 1 kHz and readers polling at <= 1 kHz, two retries
  // is almost always enough; eight is the loud failure ceiling. Also
  // returns false before the first write so consumers can distinguish
  // "no data yet" from "stale data."
  bool read(input::StateSnapshot& out, int max_tries = 8) const {
    for (int i = 0; i < max_tries; ++i) {
      const uint64_t s1 = seq_.load(std::memory_order_acquire);
      if (s1 == 0) return false;                             // no writes yet
      if (s1 & 1) continue;                                  // writer mid-update
      std::atomic_thread_fence(std::memory_order_acquire);
      const input::StateSnapshot copy = data_;
      std::atomic_thread_fence(std::memory_order_acquire);
      const uint64_t s2 = seq_.load(std::memory_order_acquire);
      if (s1 == s2) { out = copy; return true; }
    }
    return false;
  }

  std::size_t writeCount() const { return writes_.load(std::memory_order_relaxed); }

 private:
  std::atomic<uint64_t>     seq_{0};
  input::StateSnapshot      data_{};
  std::atomic<std::size_t>  writes_{0};
};

// =============================================================================
// LowPassFilter — one-pole IIR on each StateSnapshot field that warrants it.
// =============================================================================
//
// Applied to x, y, vx, vy. status and max_loop_us are pass-through (they
// are categorical / instrumentation, not signal). On the very first sample
// the filter latches state to the sample to avoid an exponential ramp-up
// from zero.
class LowPassFilter {
 public:
  explicit LowPassFilter(float alpha)
      : alpha_(alpha) {}

  void reset() { primed_ = false; }

  void apply(input::StateSnapshot& s) {
    if (!primed_) {
      x_  = s.x;  y_  = s.y;
      vx_ = s.vx; vy_ = s.vy;
      primed_ = true;
      return;
    }
    const float a = alpha_, b = 1.0f - alpha_;
    x_  = a * s.x  + b * x_;
    y_  = a * s.y  + b * y_;
    vx_ = a * s.vx + b * vx_;
    vy_ = a * s.vy + b * vy_;
    s.x = x_; s.y = y_; s.vx = vx_; s.vy = vy_;
  }

  bool primed() const { return primed_; }

 private:
  float alpha_  = 0.1f;
  float x_  = 0, y_  = 0, vx_ = 0, vy_ = 0;
  bool  primed_ = false;
};

// =============================================================================
// OuterLoopController — error metric + PD-on-perp + P-on-tangential + ERM.
// =============================================================================
//
// Decomposes the planar error into (perpendicular, tangential) basis
// established by the path's local tangent, runs PD on perp and P on the
// tangential velocity, then maps the (F_perp, F_tan) pair back to cartesian
// (fx, fy) for the wire. Force-magnitude clamp is applied last.
//
// ERM signal = carrier + a signed blow term, clamped to
// [0, ceil]:
//   carrier = base hum (always on) + monotonic speed cue (max at standstill,
//             fading to 0 at kErmSpeedCueSpan) — see computeErmCarrier.
//   blow    = a triggered one-shot envelope (BlowThroughEnvelope): proximity-
//             driven approach → peak hold → NEGATIVE dip ("suction") → recover →
//             refractory lockout. Signed: the dip subtracts below the base hum to
//             drive the total to 0 (silence), faking a bipolar cue on a unipolar
//             ERM. Off-path error is corrected by force feedback, NOT the ERM.
struct OuterCmd {
  float   fx_N = 0.0f;
  float   fy_N = 0.0f;
  uint8_t erm_pwm = 0;
  uint8_t flags = 0;

  // Diagnostics — populated for tests, ignored by the wire.
  float   erm_base  = 0.0f;
  float   erm_speed = 0.0f;  // monotonic speed cue (was the detent bell)
  float   erm_blow  = 0.0f;  // signed envelope term — negative during the suction dip
  float   perp_err  = 0.0f;
  float   vel_err   = 0.0f;
};

// Arm-in ramp. When guidance re-engages out of the RETURN-TO-START gap
// (also E-stop release / completion auto-reset), force + ERM must not STEP from
// 0 to full — they ease in over kArmRampS. armRampGain is the linear [0,1] gain
// vs. seconds since arming (<=0 → 0, >= ramp → 1; a non-positive ramp disables
// the feature → 1). scaleCmd applies that gain to the force + ERM of a computed
// OuterCmd; the ERM diagnostics scale too so the 'D' overlay plots the ramp,
// while perp_err/vel_err (measured errors, not outputs) stay untouched.
float    armRampGain(double secs_since_arm, double ramp_secs);
OuterCmd scaleCmd(const OuterCmd& c, float gain);

struct OuterGains {
  float kp_perp     = 0.0f;
  float kd_perp     = 0.0f;
  float kp_tan      = 0.0f;
  float target_mms  = 0.0f;
  float max_force_N = 0.0f;
  // ERM global mute (gains-panel toggle). When false the haptic buzz is silenced
  // (erm_pwm + breakdown forced to 0) while force feedback is unaffected; flip it
  // back to true to recover the normal carrier/blow signal.
  bool  erm_enabled = true;
};

// =============================================================================
// ERM carrier: base hum + monotonic speed cue. Pure, stateless, testable.
// =============================================================================
// speed_mms is the (non-negative) traverse speed: the buzz is max at standstill
// and fades linearly to 0 at kErmSpeedCueSpan. `blow` is filled in later by the
// envelope; `sum`/`pwm` are computed by composeErmPwm once blow is known.
struct ErmBreakdown {
  float base  = 0.0f;
  float speed = 0.0f;  // monotonic speed cue (replaces the detent bell)
  float blow  = 0.0f;  // signed envelope term (negative during the suction dip)
  float sum   = 0.0f;  // base + speed + blow, pre-clamp
  uint8_t pwm = 0;     // clamp(sum, 0, kErmSafeCeilPwm), rounded
};

// base hum + monotonic speed cue. No time, no state, no blow term.
ErmBreakdown computeErmCarrier(float speed_mms);

// Fold a signed blow term into a carrier breakdown: sets blow/sum/pwm. The dip
// may push sum below the cogging floor (the carrier never does); the clamp floor
// is 0, not kErmSafeFloorPwm, so the suction can reach silence.
void composeErmPwm(ErmBreakdown& b, float blow_signed);

// What the envelope commands this tick: the signed blow term AND the speed cue
// to actually use (the envelope gates the speed cue during a burn event).
struct BlowOutput {
  float blow      = 0.0f;  // signed blow PWM term added to the carrier
  float speed_cue = 0.0f;  // speed cue to use this tick (gated during firing)
};

// =============================================================================
// BlowThroughEnvelope — stateful one-shot ERM "burn-through" cue.
// =============================================================================
//
// Fed each outer-loop tick with (now_sec, burn_proximity in [0,1], live speed
// cue); returns the signed blow term AND the speed cue to use. Waveform:
//   Approaching (proximity-driven, reversible): proximity in [thresh, sat) ->
//                blow ramps 0..peak with proximity; speed cue = live. Below
//                thresh -> blow 0, speed cue = live.
//   Firing (one-shot, latched when proximity first reaches sat — the burn — and
//                then run on its OWN clock, ignoring proximity). The speed cue is
//                FROZEN at its value when the burn latched, then gated by phase so
//                the blow-through reads cleanly instead of fighting the buzz:
//                  [0, hold)                    : blow = peak; speed = frozen
//                  [hold, hold+dip)             : blow ramps peak -> dip (<0);  ← suction
//                                                 speed = 0 (held silent)
//                  [hold+dip, hold+dip+recover) : blow ramps dip -> 0;
//                                                 speed ramps 0 -> live (gentle return)
//   Refractory: after the envelope completes, blow = 0, speed = live, until BOTH
//                now >= t_latch + refractory AND proximity has fallen below thresh
//                (so a still-burning spot does not machine-gun re-fire).
//
// Time is injected (never wall-clock) so the waveform is deterministically unit-
// testable; now_sec must be monotonic non-decreasing across update() calls.
class BlowThroughEnvelope {
 public:
  BlowOutput update(double now_sec, float proximity, float live_speed_cue);
  void  reset() { phase_ = Phase::Idle; t_latch_ = 0.0; frozen_speed_ = 0.0f; }

 private:
  enum class Phase { Idle, Firing, Refractory };
  Phase  phase_        = Phase::Idle;
  double t_latch_      = 0.0;    // now_sec at which the one-shot fired
  float  frozen_speed_ = 0.0f;   // speed cue captured at the latch (held during firing)
};

class OuterLoopController {
 public:
  explicit OuterLoopController(OuterGains g) : g_(g) {}

  // Compute the OuterCmd to send. Path* may be null, in which case the
  // controller emits zero force and a carrier ERM signal driven by the raw
  // handle speed — used by the test seam's NullPath equivalent.
  // overpen_norm: burn proximity (0..1, exposure-dose distance to a blow-through;
  // 1.0 = burning) at the spot under the torch, from the render-side HeatModel
  // drives the ERM blow-through envelope. now_sec: monotonic clock for
  // the time-based blow envelope. NOT const: the envelope carries state.
  OuterCmd compute(const input::StateSnapshot& s,
                   const path::Path* path,
                   float overpen_norm = 0.0f,
                   double now_sec = 0.0);

  const OuterGains& gains() const { return g_; }
  void setGains(const OuterGains& g) { g_ = g; }

 private:
  OuterGains g_;
  BlowThroughEnvelope blow_env_;  // stateful one-shot blow-through cue
};

// =============================================================================
// ControlThreads — owns the SnapshotSlot, LP filter, controller, and three
// thread-bodies. Live mode spins std::threads at the configured rates;
// tests call the ticks directly.
// =============================================================================
class ControlThreads {
 public:
  // None of the pointers are owned. `path` may be null (e.g. demo mode
  // before a path is loaded); the outer-loop tick handles null safely.
  ControlThreads(input::IInputSource* in,
                 motor::IMotorSink*   sink,
                 const path::Path*    path,
                 OuterGains           gains);
  ~ControlThreads();

  // --- Test seam: drive each thread's body manually -------------------------
  // rxTick: drain the input source, apply LP filter, publish snapshot.
  //         Returns the number of new snapshots published (0 if poll empty).
  std::size_t rxTick(double now_sec);

  // outerLoopTick: read snapshot, compute OuterCmd, scale it by `gain` (the
  //                arm-in ramp; default 1 = full output), send the scaled
  //                cmd via sink, and return it. gain<1 eases force + ERM in after
  //                guidance re-arms. Returns zero if no snapshot yet.
  OuterCmd    outerLoopTick(double now_sec, float gain = 1.0f);

  // heartbeatTick: send one HEARTBEAT. No-op if sink is null.
  void        heartbeatTick(double now_sec);

  // sendIdleCmd: send a single zero CMD_DOWN (force 0, ERM 0). The app calls
  // this every frame the outer loop is NOT armed (E-stop, RETURN-TO-START gap,
  // end-of-weld release) instead of just skipping outerLoopTick. The firmware
  // latches its last force/ERM command and applies it every inner-loop tick
  // until the next CMD_DOWN, so NOT sending leaves the trainee feeling the last
  // force and hearing the last ERM buzz across the gap; an explicit zero
  // actively releases both. No-op if sink is null.
  void        sendIdleCmd();

  // --- Live mode -----------------------------------------------------------
  // Spawn the three threads, each looping on sleep_until + tick. Idempotent;
  // calling start() twice is a programmer error and triggers an assert.
  void start();
  // Set the stop flag, then join all three threads. Safe to call from any
  // thread, multiple times.
  void stop();

  // --- Accessors -----------------------------------------------------------
  const SnapshotSlot& slot() const { return slot_; }
  SnapshotSlot&       slot()       { return slot_; }
  const OuterLoopController& outer() const { return outer_; }
  // Runtime gain retune (HUD sliders are the only runtime-tunable surface).
  void setGains(const OuterGains& g) { outer_.setGains(g); }
  void setPath(const path::Path* p) { path_ = p; }
  // The active path pointer, for the app's setActivePath() assert that the
  // controller and the renderer share one Path (visual guide == force path).
  const path::Path* path() const { return path_; }
  // Re-prime the input low-pass filter. The app calls this on a reset / shape
  // switch so the handle's position jump (e.g. teleport to a new shape) latches
  // cleanly instead of being smeared into a phantom velocity spike for the few
  // frames the one-pole filter takes to catch up (kLpAlpha = 0.6).
  void resetFilter() { filter_.reset(); }
  bool running() const { return running_.load(std::memory_order_acquire); }

  // Burn proximity (0..1, exposure-dose distance to a blow-through; 1.0 =
  // burning) at the spot under the torch, fed from the render-side HeatModel
  // each frame. The outer-loop tick reads it and shapes the ERM
  // blow-through cue from it. Atomic: written by the app/render thread, read by
  // the outer-loop thread in live mode.
  void setOverpenetration(float norm) {
    overpen_norm_.store(norm, std::memory_order_relaxed);
  }

  // Host-side position re-home. tareCurrentTo shifts the reported (x, y)
  // so the LATEST snapshot reads exactly (anchor_x, anchor_y): the app passes the
  // point 20 mm above the active shape's start, treating the operator's current
  // physical max-reach (pi/2) pose as that spot. Returns false if no snapshot has
  // landed yet (nothing to tare against). The offset is a constant subtracted in
  // rxTick, so it never touches vx/vy and persists until the next tare/reset.
  // Math: slot holds displayed = raw - tare; to land the current reading d on the
  // anchor, tare += (d - anchor) (new output = raw - tare = anchor). Written by
  // the app/render thread, read by the RX thread (atomic, like overpen_norm_).
  bool tareCurrentTo(float anchor_x, float anchor_y) {
    input::StateSnapshot s;
    if (!slot_.read(s)) return false;  // no snapshot yet -> nothing to tare
    // Single writer (the app thread), so load-then-store is race-free; the RX
    // thread only reads. (std::atomic<float>::fetch_add is C++20; we are C++17.)
    tare_x_.store(tare_x_.load(std::memory_order_relaxed) + (s.x - anchor_x),
                  std::memory_order_relaxed);
    tare_y_.store(tare_y_.load(std::memory_order_relaxed) + (s.y - anchor_y),
                  std::memory_order_relaxed);
    return true;
  }
  // Set the re-home offset directly (absolute). Used at path-load to SEED the
  // boot/default convention so the reported centerline/pi-2 origin (0,0) lands
  // 20 mm above the active shape's start — i.e. a fresh boot already matches the
  // post-'T' tared layout. Single writer (app thread).
  void setTare(float tx, float ty) {
    tare_x_.store(tx, std::memory_order_relaxed);
    tare_y_.store(ty, std::memory_order_relaxed);
  }
  // Clear the re-home offset, returning to the raw firmware-reported frame.
  void resetTare() { setTare(0.0f, 0.0f); }

  // Optional session-log sink. When set, rxTick appends every published
  // snapshot to it (the log does its own 1 kHz->100 Hz decimation).
  // Null by default; the log must outlive the RX thread.
  void setSessionLog(log::SessionLog* l) { log_ = l; }

 private:
  void runRxThread();
  void runOuterLoopThread();
  void runHeartbeatThread();

  input::IInputSource*  in_   = nullptr;
  motor::IMotorSink*    sink_ = nullptr;
  const path::Path*     path_ = nullptr;
  log::SessionLog*      log_  = nullptr;

  SnapshotSlot          slot_;
  LowPassFilter         filter_;
  OuterLoopController   outer_;

  std::thread           rx_thr_;
  std::thread           outer_thr_;
  std::thread           hb_thr_;
  std::atomic<bool>     running_{false};
  std::atomic<float>    overpen_norm_{0.0f};  // burn proximity (0..1, exposure-dose distance to a blow-through; 1.0 = burning)
  std::atomic<float>    tare_x_{0.0f};        // host-side position re-home offset, subtracted in rxTick
  std::atomic<float>    tare_y_{0.0f};        // written by app thread, read by RX thread
};

}  // namespace control
}  // namespace welding
