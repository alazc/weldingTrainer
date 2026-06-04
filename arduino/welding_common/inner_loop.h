#ifndef WELDING_INNER_LOOP_H
#define WELDING_INNER_LOOP_H

// Pure helpers for the inner control loop, factored out so the host build
// can unit-test them. Anything that touches hardware (Encoder.h reads,
// analogWrite, ISR registers) stays in welding_trainer.ino.
//
// All functions here are header-only, free of static state, and compile
// identically on AVR and MSVC.

#include <stdint.h>
#ifndef ARDUINO
  #include <cstddef>
#endif

#include "kinematics.h"  // for trig::kTwoPi
#include "config.h"

namespace welding {
namespace inner {

// --- MR analog angle sensor: flip-unwrap + linear fit + tare ----------------
//
// A standard Hapkit senses joint angle with an MR analog sensor (analogRead,
// 0..1023), NOT a quadrature encoder. The raw value sweeps up and "flips" back
// roughly every half-turn; we count flips to UNWRAP it into a continuous count,
// then a per-kit linear fit maps unwrapped counts -> degrees -> radians in the
// kinematic frame. Ported from the stock Hapkit `read_mr_sensor`.
//
// All state lives in this struct so the conversion is host-testable: feed a
// synthetic raw sequence that crosses the flip boundary; no hardware here. The
// .ino owns the analogRead and an instance.

struct MrUnwrapper {
  long flips         = 0;
  int  last_raw      = 0;
  int  last_last_raw = 0;
  bool flipped       = false;
  bool primed        = false;

  // Seed the history with the first reading (call once at boot before update()).
  void prime(int raw) {
    last_raw = raw;
    last_last_raw = raw;
    flips = 0;
    flipped = false;
    primed = true;
  }

  // Feed a raw analogRead value; return the unwrapped count (raw + flips*cpr).
  // A jump larger than flip_threshold between readings means the sensor wrapped
  // past its 180-degree mark; the sign of the jump tells us which way.
  long update(int raw, int flip_threshold, int cpr) {
    if (!primed) prime(raw);
    const int last_raw_diff   = raw - last_last_raw;
    const int last_raw_offset = (last_raw_diff < 0) ? -last_raw_diff : last_raw_diff;
    last_last_raw = last_raw;
    last_raw      = raw;
    if (last_raw_offset > flip_threshold && !flipped) {
      if (last_raw_diff > 0) --flips;   // cw wrap
      else                   ++flips;   // ccw wrap
      flipped = true;                   // debounce: next tick won't re-trigger
    } else {
      flipped = false;
    }
    return (long)raw + flips * (long)cpr;
  }
};

// Unwrapped MR counts -> joint angle (rad) in the kinematic frame.
//   slope_deg_per_count: per-kit linear-fit slope (bench fit: 0.01555 deg/count)
//   sign:    +/-1, aligns the sensor's increasing direction with CCW-from-+x
//   tare_offset_rad: set at boot so a known reference pose maps to theta_ref
// Net: theta = sign*(slope*counts in rad) + offset, so theta moves by
// sign*Delta from the tare pose. The per-board zero lives entirely in the tare.
inline float mrCountsToThetaRad(long unwrapped, float slope_deg_per_count,
                                int8_t sign, float tare_offset_rad) {
  const float rad = slope_deg_per_count * (float)unwrapped * (trig::kPi / 180.0f);
  return (float)sign * rad + tare_offset_rad;
}

// Tare: the offset that makes `unwrapped_at_ref` read as theta_ref_rad. Called
// once at boot with the device held at the reference pose (BRINGUP).
inline float mrTareOffsetRad(long unwrapped_at_ref, float slope_deg_per_count,
                             int8_t sign, float theta_ref_rad) {
  const float rad = slope_deg_per_count * (float)unwrapped_at_ref * (trig::kPi / 180.0f);
  return theta_ref_rad - (float)sign * rad;
}

// --- Windowed finite-difference velocity -------------------------------------
//
// The 1-step diff at 1 kHz, g_vx = (x - last_x)*1000, amplifies the MR sensor's
// position quantization (~tenths of a mm) into tens of mm/s of impulse noise.
// Differencing over an N-tick window instead — vel = (x_now - x_[N back])*hz/N —
// is exactly the MEAN of the N intervening one-step diffs: it telescopes to a
// ~N-ms-window velocity that cuts the zero-mean quantization noise by ~sqrt(N).
// Runs in the ISR where the full 1 kHz position stream lives; this CANNOT be
// done on the PC because STATE_UP arrives there at only ~60 Hz (bench:
// of-app frames/pump = 1, so a PC-side average is a no-op).
//
// Ring of N positions. `count` (saturating at N) handles warm-up: before the
// ring fills we difference over the samples we have, so the estimate is well-
// defined from the 2nd sample on (and exactly 0 on the very first — no boot
// spike, unlike the old last_x=0 seed). All state is in the struct; no globals,
// no hardware — the host test feeds synthetic position sequences.
template <int N>
struct VelocityEstimator {
  float xbuf[N];
  float ybuf[N];
  int   head  = 0;   // index of the slot the NEXT sample will overwrite
  int   count = 0;   // samples pushed so far (saturates at N)

  void reset() { head = 0; count = 0; }

  // Push the latest position; write the windowed velocity (mm/s) to vx, vy.
  // `hz` is the push rate (config::kInnerLoopHz). Diff span is min(count, N)
  // ticks, so the divisor matches the actual elapsed window during warm-up.
  void update(float x, float y, float hz, float& vx, float& vy) {
    const int steps = (count < N) ? count : N;   // intervals available (pre-push)
    if (steps <= 0) {
      vx = 0.0f; vy = 0.0f;                       // first ever sample: no velocity
    } else {
      int oi = head - steps;                      // sample `steps` ticks back
      if (oi < 0) oi += N;
      const float inv = hz / (float)steps;
      vx = (x - xbuf[oi]) * inv;
      vy = (y - ybuf[oi]) * inv;
    }
    xbuf[head] = x; ybuf[head] = y;
    head = (head + 1) % N;
    if (count < N) ++count;
  }
};

// --- Watchdog: has the heartbeat gone stale? --------------------------------
//
// Returns true if (now_ms - last_heartbeat_ms) >= timeout_ms.
// Uses unsigned subtraction so a single 32-bit millis() rollover (every
// ~49.7 days) does not falsely trip the watchdog. The wraparound math
// assumes last_heartbeat_ms is updated regularly.

inline bool watchdogStale(uint32_t now_ms, uint32_t last_heartbeat_ms,
                          uint32_t timeout_ms) {
  return (uint32_t)(now_ms - last_heartbeat_ms) >= timeout_ms;
}

// --- Signed PWM packing -----------------------------------------------------
//
// Splits a signed PWM magnitude into (direction, |pwm|) for an H-bridge
// driven by a separate DIR pin + PWM pin. Clamps |pwm| to max_abs.
//
// signed_pwm: signed PWM units; range +/-255 nominal, but we clamp by
//             max_abs (typically below 255 for safety margin).
// max_abs:    hard ceiling, e.g. config::kPwmMaxClamp (200).
//
// dir_out:    true  -> drive DIR pin HIGH for "positive" direction
//             false -> drive DIR pin LOW.
// pwm_out:    unsigned PWM magnitude in [0, max_abs].

struct ClampResult {
  bool    dir;
  uint8_t pwm;
};

inline ClampResult clampSignedPwm(float signed_pwm, uint8_t max_abs) {
  ClampResult r;
  r.dir = signed_pwm >= 0.0f;
  float a = r.dir ? signed_pwm : -signed_pwm;
  if (a > (float)max_abs) a = (float)max_abs;
  r.pwm = (uint8_t)a;
  return r;
}

// --- Joint torque (N*mm) -> signed PWM units (capstan + sqrt law) -----------
//
// The motor+drive produces torque ~ duty^2, so duty ~ sqrt(torque) linearizes
// it (supersedes the earlier linear map). The device is capstan-driven, so the
// joint torque from jacobianTranspose is first reduced to the motor-pulley
// torque by the radius ratio rp/rs, then square-rooted to a duty:
//   Tp[N*m] = rp_over_rs * |tau_nmm| * 1e-3      (N*mm -> N*m, capstan reduction)
//   duty    = sqrt(Tp / k_torque_const_nm)        clamped to [0, 1]
//   pwm     = sign(tau) * duty * 255
// rp_over_rs = config::kCapstanRpOverRs, k_torque_const_nm = config::kMotorTorqueConstNm.
// Parameterized (not reading config directly) so the host test can pin the
// curve shape independent of the per-kit calibration.

inline float torqueToSignedPwmCapstan(float tau_nmm, float rp_over_rs,
                                      float k_torque_const_nm) {
  if (k_torque_const_nm <= 0.0f) return 0.0f;
  const float sign = (tau_nmm >= 0.0f) ? 1.0f : -1.0f;
  const float abs_nmm = (tau_nmm >= 0.0f) ? tau_nmm : -tau_nmm;
  const float tp_nm = rp_over_rs * abs_nmm * 1e-3f;   // |motor-pulley torque|, N*m
  float duty = math::nrSqrt(tp_nm / k_torque_const_nm);
  if (duty > 1.0f) duty = 1.0f;
  return sign * duty * 255.0f;
}

// --- Cartesian force command clamp (cable-slip ceiling) ---------------------
//
// The capstan cable slips above a per-kit force (bench Hapkit: ~4 N). Past that
// the motor just slips the cable instead of delivering force, so scale the
// (fx,fy) command vector down to magnitude max_n, preserving direction. A
// max_n <= 0 disables the clamp. Pure + host-tested so the ceiling stays a
// per-kit bench number; max_n = config::kForceCmdMaxN at the call site.
inline void clampForceMagN(float& fx, float& fy, float max_n) {
  if (max_n <= 0.0f) return;
  const float mag2 = fx * fx + fy * fy;
  const float max2 = max_n * max_n;
  if (mag2 > max2) {
    const float s = max_n / math::nrSqrt(mag2);
    fx *= s;
    fy *= s;
  }
}

// --- JOG_MODE per-motor output ----------------------------------------------
//
// Cycles through four 500-ms phases:
//   phase 0:  motor 0 +kJogPwm,  motor 1 0
//   phase 1:  motor 0 -kJogPwm,  motor 1 0
//   phase 2:  motor 0 0,         motor 1 +kJogPwm
//   phase 3:  motor 0 0,         motor 1 -kJogPwm
//
// Returns the signed PWM (range +/- kJogPwm) for the requested motor index.
// The .ino dispatches this to two motor-pin pairs; the host test just
// verifies the phase-table behavior.
//
// Return type is int16_t, NOT int8_t: PWM magnitude can be up to 255, which an
// int8_t (max +127) cannot hold as a positive value — kJogPwm=150 silently
// wrapped to -106 on the bench. int16_t represents the full +/-255 range.
inline int16_t jogModePwm(uint32_t now_ms, uint8_t motor_idx) {
  const uint8_t phase = (uint8_t)((now_ms / config::kJogPhaseMs) & 0x03);
  if (motor_idx == 0) {
    if (phase == 0) return  (int16_t)config::kJogPwm;
    if (phase == 1) return -(int16_t)config::kJogPwm;
    return 0;
  } else {
    if (phase == 2) return  (int16_t)config::kJogPwm;
    if (phase == 3) return -(int16_t)config::kJogPwm;
    return 0;
  }
}

// --- Loop-budget overrun check ----------------------------------------------
// Returns true if the observed max should set kStatusLoopOverrun.
inline bool loopOverrun(uint16_t max_loop_us) {
  return max_loop_us > config::kLoopBudgetUs;
}

}  // namespace inner
}  // namespace welding

#endif  // WELDING_INNER_LOOP_H
