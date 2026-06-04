#include "ControlThreads.h"

#include "Config.h"
#include "PathLibrary.h"
#include "SessionLog.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>

namespace welding {
namespace control {

// =============================================================================
// computeErmCarrier — base hum + monotonic speed cue. Pure, stateless.
// =============================================================================
ErmBreakdown computeErmCarrier(float speed_mms) {
  ErmBreakdown b;

  // (1) Base hum: constant, above the floor so the blow dip has room to subtract.
  b.base = config::kErmBaseHum;

  // (2) Monotonic speed cue: max at standstill, fading linearly to 0 at the span.
  // Replaces the old detent bell: dwelling is the dangerous failure, so the
  // buzz nags when slow and quiets as you traverse, then hands off to the blow
  // envelope when a dwell starts to burn.
  const float spd = speed_mms < 0.0f ? 0.0f : speed_mms;
  const float t   = spd >= config::kErmSpeedCueSpan ? 1.0f
                                                    : spd / config::kErmSpeedCueSpan;
  b.speed = config::kErmSpeedCuePeak * (1.0f - t);
  return b;
}

// =============================================================================
// composeErmPwm — fold a signed blow term into the carrier; clamp to [0, ceil].
// =============================================================================
// The clamp FLOOR is 0, not kErmSafeFloorPwm: the carrier alone is always >=
// base (24) >= floor, but the blow dip is *meant* to briefly drive the total to
// 0 (the suction). The floor is a sustained-spin constraint on the carrier, not
// a hard limit on a ~0.1 s transient.
void composeErmPwm(ErmBreakdown& b, float blow_signed) {
  b.blow = blow_signed;
  b.sum  = b.base + b.speed + b.blow;
  const float clamped = std::min(std::max(b.sum, 0.0f), config::kErmSafeCeilPwm);
  b.pwm = static_cast<uint8_t>(clamped + 0.5f);
}

// =============================================================================
// armRampGain / scaleCmd — arm-in ramp for force + ERM.
// =============================================================================
// Linear 0→1 over ramp_secs. A non-positive ramp_secs means "no ramp" (gain 1)
// so the feature can be disabled from config without a branch at the call site.
float armRampGain(double secs_since_arm, double ramp_secs) {
  if (ramp_secs <= 0.0)         return 1.0f;
  if (secs_since_arm <= 0.0)    return 0.0f;
  if (secs_since_arm >= ramp_secs) return 1.0f;
  return static_cast<float>(secs_since_arm / ramp_secs);
}

OuterCmd scaleCmd(const OuterCmd& c, float gain) {
  const float g = gain < 0.0f ? 0.0f : (gain > 1.0f ? 1.0f : gain);
  OuterCmd out  = c;
  out.fx_N      = c.fx_N * g;
  out.fy_N      = c.fy_N * g;
  out.erm_pwm   = static_cast<uint8_t>(c.erm_pwm * g + 0.5f);
  out.erm_base  = c.erm_base  * g;
  out.erm_speed = c.erm_speed * g;
  out.erm_blow  = c.erm_blow  * g;  // signed; gain in [0,1] preserves its sign
  return out;
}

// =============================================================================
// BlowThroughEnvelope::update — proximity-driven approach + timed one-shot.
// =============================================================================
BlowOutput BlowThroughEnvelope::update(double now_sec, float proximity, float live_speed_cue) {
  const float  thresh  = config::kErmBlowThresh;
  const float  sat     = config::kErmBlowSat;
  const float  peak    = config::kErmBlowPeak;
  const float  dip     = config::kErmBlowDip;
  const double hold    = config::kErmBlowHoldS;
  const double dipS    = config::kErmBlowDipS;
  const double recover = config::kErmBlowRecoverS;
  const double refrac  = config::kErmBlowRefractoryS;

  switch (phase_) {
    case Phase::Idle: {
      if (proximity >= sat) {              // burn latched -> fire the one-shot now
        phase_        = Phase::Firing;
        t_latch_      = now_sec;
        frozen_speed_ = live_speed_cue;    // hold the speed cue at its value at the burn
        return {peak, frozen_speed_};
      }
      if (proximity > thresh) {            // approach: ramp 0..peak with proximity; speed live
        return {peak * ((proximity - thresh) / (sat - thresh)), live_speed_cue};
      }
      return {0.0f, live_speed_cue};
    }
    case Phase::Firing: {                  // one-shot on its own clock; ignores proximity
      const double tau = now_sec - t_latch_;
      if (tau < hold) {                                              // hold the peak; speed frozen
        return {peak, frozen_speed_};
      }
      if (tau < hold + dipS) {                                       // peak -> dip (suction); speed silent
        const float u = static_cast<float>((tau - hold) / dipS);
        return {peak + (dip - peak) * u, 0.0f};
      }
      if (tau < hold + dipS + recover) {                             // dip -> 0; speed gently back to live
        const float u = static_cast<float>((tau - hold - dipS) / recover);
        return {dip * (1.0f - u), live_speed_cue * u};
      }
      phase_ = Phase::Refractory;                                    // envelope complete
      return {0.0f, live_speed_cue};
    }
    case Phase::Refractory: {              // locked out until window elapses AND not burning
      if (now_sec - t_latch_ >= refrac && proximity < thresh) {
        phase_ = Phase::Idle;
      }
      return {0.0f, live_speed_cue};
    }
  }
  return {0.0f, live_speed_cue};
}

// =============================================================================
// OuterLoopController::compute
// =============================================================================
OuterCmd OuterLoopController::compute(const input::StateSnapshot& s,
                                      const path::Path* path,
                                      float overpen_norm,
                                      double now_sec) {
  OuterCmd cmd;

  // No path: zero force; the ERM carrier is driven by the raw handle speed and
  // the blow envelope still tracks over-penetration, so a demo dwell warns even
  // before a path is loaded. Keeps the controller well-defined.
  if (path == nullptr || path->empty()) {
    const float speed = std::sqrt(s.vx * s.vx + s.vy * s.vy);
    ErmBreakdown b = computeErmCarrier(speed);
    const BlowOutput out = blow_env_.update(now_sec, overpen_norm, b.speed);
    b.speed = out.speed_cue;          // envelope gates the speed cue during a burn
    composeErmPwm(b, out.blow);
    if (!g_.erm_enabled) b = ErmBreakdown{};  // gains-panel ERM mute
    cmd.erm_base  = b.base;
    cmd.erm_speed = b.speed;
    cmd.erm_blow  = b.blow;
    cmd.erm_pwm   = b.pwm;
    cmd.vel_err   = -g_.target_mms;
    return cmd;
  }

  const path::PathPoint pp = path->nearestPoint(s.x, s.y);

  // Decompose the (state - nearest) vector along (tangent, normal). Normal is
  // 90 deg CCW of tangent: (-ty, tx). The sign convention "left of tangent
  // positive" matches PathPoint::distance documentation but we recompute it
  // here from the raw geometry so we are independent of the path layer's sign.
  const float dx = s.x - pp.position.x;
  const float dy = s.y - pp.position.y;
  const float tx = pp.tangent.x;
  const float ty = pp.tangent.y;
  const float nx = -ty;
  const float ny =  tx;

  const float perp_err = dx * nx + dy * ny;             // signed
  const float perp_vel = s.vx * nx + s.vy * ny;
  const float tan_vel  = s.vx * tx + s.vy * ty;
  const float vel_err  = tan_vel - g_.target_mms;       // along-path error

  // PD-on-perp, P-on-tangential.
  const float f_perp = -g_.kp_perp * perp_err - g_.kd_perp * perp_vel;
  const float f_tan  = -g_.kp_tan  * vel_err;

  // Back to cartesian.
  float fx = f_perp * nx + f_tan * tx;
  float fy = f_perp * ny + f_tan * ty;

  // Force-magnitude clamp.
  const float mag2 = fx * fx + fy * fy;
  const float maxF = g_.max_force_N;
  if (mag2 > maxF * maxF && maxF > 0.0f) {
    const float scale = maxF / std::sqrt(mag2);
    fx *= scale;
    fy *= scale;
  }

  // ERM carrier rides the traverse speed along the seam (backward counts as
  // standstill = full buzz); the blow envelope tracks over-penetration.
  const float speed = tan_vel > 0.0f ? tan_vel : 0.0f;
  ErmBreakdown b = computeErmCarrier(speed);
  const BlowOutput out = blow_env_.update(now_sec, overpen_norm, b.speed);
  b.speed = out.speed_cue;            // envelope gates the speed cue during a burn
  composeErmPwm(b, out.blow);
  if (!g_.erm_enabled) b = ErmBreakdown{};  // gains-panel ERM mute

  cmd.fx_N      = fx;
  cmd.fy_N      = fy;
  cmd.erm_pwm   = b.pwm;
  cmd.flags     = 0;
  cmd.erm_base  = b.base;
  cmd.erm_speed = b.speed;
  cmd.erm_blow  = b.blow;
  cmd.perp_err  = perp_err;
  cmd.vel_err   = vel_err;
  return cmd;
}

// =============================================================================
// ControlThreads
// =============================================================================
ControlThreads::ControlThreads(input::IInputSource* in,
                               motor::IMotorSink*   sink,
                               const path::Path*    path,
                               OuterGains           gains)
    : in_(in), sink_(sink), path_(path),
      filter_(config::kLpAlpha),
      outer_(gains) {}

ControlThreads::~ControlThreads() {
  stop();
}

std::size_t ControlThreads::rxTick(double now_sec) {
  if (in_ == nullptr) return 0;
  std::size_t n = 0;
  input::StateSnapshot s;
  // Drain everything the source has queued. For 1 kHz tick with a source
  // that produces at <= 1 kHz this loop runs 0 or 1 times in practice; the
  // loop shape is what makes it correct under burstier sources too.
  //
  // NOTE: a PC-side burst-average was tried here to de-noise velocity and
  // REJECTED — the serial rig delivers STATE_UP at only ~60 Hz (loop() is paced
  // by the I2C exchange), so this drains exactly one frame per pump (measured:
  // frames/pump = 1) and there is nothing to average. The velocity de-noising
  // lives in the firmware ISR instead (inner::VelocityEstimator). The return
  // value is logged by ofApp as a frames/pump diagnostic.
  while (in_->poll(now_sec, s)) {
    filter_.apply(s);
    // Host-side position re-home. A constant offset, so it shifts the dot
    // without perturbing vx/vy (a derivative) and — applied AFTER the filter,
    // which stays in the raw frame — re-taring is a clean step with no transient.
    s.x -= tare_x_.load(std::memory_order_relaxed);
    s.y -= tare_y_.load(std::memory_order_relaxed);
    slot_.write(s);
    if (log_ != nullptr) log_->append(now_sec, s);  // decimates internally
    ++n;
  }
  return n;
}

OuterCmd ControlThreads::outerLoopTick(double now_sec, float gain) {
  input::StateSnapshot s;
  if (!slot_.read(s)) return {};  // no snapshot yet
  const float overpen = overpen_norm_.load(std::memory_order_relaxed);
  OuterCmd cmd = outer_.compute(s, path_, overpen, now_sec);
  if (gain < 1.0f) cmd = scaleCmd(cmd, gain);  // arm-in ramp: send the
                                               // SCALED cmd, not the full one
  if (sink_ != nullptr) {
    sink_->sendCmdDown(cmd.fx_N, cmd.fy_N, cmd.erm_pwm, cmd.flags);
  }
  return cmd;
}

void ControlThreads::heartbeatTick(double /*now_sec*/) {
  if (sink_ != nullptr) sink_->sendHeartbeat();
}

void ControlThreads::sendIdleCmd() {
  if (sink_ != nullptr) sink_->sendCmdDown(0.0f, 0.0f, /*erm_pwm=*/0, /*flags=*/0);
}

// --- Live-mode thread bodies ------------------------------------------------
//
// Each thread's body is a sleep_until loop. We anchor the schedule to a fixed
// origin and advance it by exactly one period per iteration, so the thread
// "catches up" if a tick runs long instead of accumulating drift.

namespace {
using clk      = std::chrono::steady_clock;
using ns       = std::chrono::nanoseconds;
inline ns      period(double hz) {
  return ns(static_cast<long long>(1e9 / hz));
}
inline double  sec_since(clk::time_point t0) {
  using fs = std::chrono::duration<double>;
  return std::chrono::duration_cast<fs>(clk::now() - t0).count();
}
}  // namespace

void ControlThreads::start() {
  assert(!running_.load(std::memory_order_acquire) &&
         "ControlThreads::start() called twice");
  running_.store(true, std::memory_order_release);
  rx_thr_    = std::thread([this] { runRxThread(); });
  outer_thr_ = std::thread([this] { runOuterLoopThread(); });
  hb_thr_    = std::thread([this] { runHeartbeatThread(); });
}

void ControlThreads::stop() {
  running_.store(false, std::memory_order_release);
  if (rx_thr_.joinable())    rx_thr_.join();
  if (outer_thr_.joinable()) outer_thr_.join();
  if (hb_thr_.joinable())    hb_thr_.join();
}

void ControlThreads::runRxThread() {
  const auto t0  = clk::now();
  const auto dt  = period(config::kRxHz);
  auto       nxt = t0 + dt;
  while (running_.load(std::memory_order_acquire)) {
    rxTick(sec_since(t0));
    std::this_thread::sleep_until(nxt);
    nxt += dt;
  }
}

void ControlThreads::runOuterLoopThread() {
  const auto t0  = clk::now();
  const auto dt  = period(config::kOuterLoopHz);
  auto       nxt = t0 + dt;
  while (running_.load(std::memory_order_acquire)) {
    outerLoopTick(sec_since(t0));
    std::this_thread::sleep_until(nxt);
    nxt += dt;
  }
}

void ControlThreads::runHeartbeatThread() {
  const auto t0  = clk::now();
  const auto dt  = period(config::kHeartbeatHz);
  auto       nxt = t0 + dt;
  while (running_.load(std::memory_order_acquire)) {
    heartbeatTick(sec_since(t0));
    std::this_thread::sleep_until(nxt);
    nxt += dt;
  }
}

}  // namespace control
}  // namespace welding
