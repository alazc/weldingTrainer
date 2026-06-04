#include "AudioModel.h"

#include <algorithm>
#include <cmath>

namespace welding {
namespace audio {

float crackleRate(float speed_mms, float min_hz, float max_hz,
                  float speed_cap_mms) {
  const float cap = (speed_cap_mms > 0.0f) ? speed_cap_mms : 1.0f;
  float t = speed_mms / cap;
  t = std::max(0.0f, std::min(t, 1.0f));
  return min_hz + (max_hz - min_hz) * t;
}

CrackleModulator::CrackleModulator(float min_hz, float max_hz,
                                   float speed_cap_mms,
                                   float hiss_idle_gain, float hiss_active_gain,
                                   float hiss_active_speed_mms, float hiss_alpha,
                                   float weld_ramp_s)
    : min_hz_(min_hz), max_hz_(max_hz), speed_cap_(speed_cap_mms),
      hiss_idle_(hiss_idle_gain), hiss_active_(hiss_active_gain),
      hiss_active_speed_(hiss_active_speed_mms), hiss_alpha_(hiss_alpha),
      weld_ramp_s_(weld_ramp_s) {}

int CrackleModulator::update(float speed_mms, float dt_s, bool weld_active) {
  rate_hz_ = crackleRate(speed_mms, min_hz_, max_hz_, speed_cap_);

  // Phase accumulator: each unit crossing is one crackle trigger. Carrying
  // the fractional remainder forward makes trigger timing independent of the
  // frame rate (a long frame fires the right number and keeps the remainder).
  phase_ += rate_hz_ * dt_s;
  int triggers = 0;
  while (phase_ >= 1.0f) {
    phase_ -= 1.0f;
    ++triggers;
  }

  // Hiss gain crossfades idle<->active by whether the welder is moving,
  // one-pole smoothed so it ramps instead of clicking.
  const float target = (speed_mms >= hiss_active_speed_) ? hiss_active_ : hiss_idle_;
  if (!primed_) {
    hiss_gain_ = target;     // latch on first call (no ramp-up from zero)
    primed_    = true;
  } else {
    hiss_gain_ += hiss_alpha_ * (target - hiss_gain_);
  }

  // Overall weld-active envelope: linear ramp toward 1 (armed) or 0 (idle)
  // over weld_ramp_s_, so the welding voices pause and ramp in/out with the arm
  // state instead of cutting abruptly. weld_ramp_s_ <= 0 snaps (legacy/instant).
  const float weld_target = weld_active ? 1.0f : 0.0f;
  if (weld_ramp_s_ <= 0.0f) {
    weld_gain_ = weld_target;
  } else {
    const float step = dt_s / weld_ramp_s_;
    weld_gain_ = (weld_gain_ < weld_target)
                     ? std::min(weld_target, weld_gain_ + step)
                     : std::max(weld_target, weld_gain_ - step);
  }
  return triggers;
}

float CrackleModulator::cracklePitch(float speed_mms, float spread) const {
  const float cap = (speed_cap_ > 0.0f) ? speed_cap_ : 1.0f;
  float t = speed_mms / cap;
  t = std::max(0.0f, std::min(t, 1.0f));
  // Centered at 1.0; +/- spread across the speed range.
  return 1.0f + spread * (t - 0.5f) * 2.0f;
}

void CrackleModulator::reset() {
  phase_     = 0.0f;
  rate_hz_   = 0.0f;
  hiss_gain_ = 0.0f;
  weld_gain_ = 0.0f;   // re-arm ramps the welding voices in from silence
  primed_    = false;
}

}  // namespace audio
}  // namespace welding
