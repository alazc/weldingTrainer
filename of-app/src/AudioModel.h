#pragma once

// Audio core — the pure-logic half of the audio engine, zero
// openFrameworks dependency so it can be host-tested with cl.exe. The
// oF-facing AudioEngine (AudioEngine.{h,cpp}) owns the ofSoundPlayer loops and
// asks this model, each frame, "how loud is the hiss, and did a crackle fire?"
//
// Two signals (mirrors the renderer's model/draw split, and the ERM detent
// math that was tested without hardware):
//   * Arc hiss gain — a continuous looped sound whose gain crossfades between
//     an idle level (welder stopped) and an active level (welder moving),
//     one-pole smoothed so it never clicks.
//   * Crackle rate  — discrete spatter pops whose rate scales with travel
//     speed. This is the audible speed cue. Implemented as a phase
//     accumulator so trigger timing is frame-rate independent.

namespace welding {
namespace audio {

class CrackleModulator {
 public:
  // min_hz/max_hz bound the crackle trigger rate; speed_cap_mms is the travel
  // speed at (and above) which the rate saturates to max_hz.
  // weld_ramp_s: linear in/out ramp time (s) for the overall weld-active envelope
  // see weldGain(). 0 means "instant" (envelope snaps; legacy behavior).
  CrackleModulator(float min_hz, float max_hz, float speed_cap_mms,
                   float hiss_idle_gain, float hiss_active_gain,
                   float hiss_active_speed_mms, float hiss_alpha,
                   float weld_ramp_s = 0.0f);

  // Advance dt at the current travel speed. Returns how many crackle triggers
  // fired this tick (0, 1, or more if dt is large or rate is high). Updates
  // the smoothed hiss gain AND the weld-active envelope (weldGain) as side
  // effects. weld_active gates the welding voices: true while guidance is armed,
  // false in RETURN-TO-START / end-of-weld so the voices ramp OUT.
  int update(float speed_mms, float dt_s, bool weld_active = true);

  float crackleRateHz() const { return rate_hz_; }   // current trigger rate
  float hissGain()      const { return hiss_gain_; } // smoothed, [idle, active]

  // Overall weld-active envelope in [0,1]: ramps 0→1 over weld_ramp_s when
  // armed, 1→0 when not. The oF layer multiplies BOTH welding voices (hiss +
  // crackle) by this so they pause and ramp in/out with the arm state, matching
  // the force/ERM arm-in ramp. Separate from hissGain's idle↔active fade.
  float weldGain()      const { return weld_gain_; }

  // Playback-rate multiplier for a crackle sample: faster travel -> slightly
  // higher pitch, giving variety. Centered at 1.0. Deterministic (no RNG):
  // a function of the normalized speed only.
  float cracklePitch(float speed_mms, float spread) const;

  void reset();

 private:
  float min_hz_, max_hz_, speed_cap_;
  float hiss_idle_, hiss_active_, hiss_active_speed_, hiss_alpha_;
  float weld_ramp_s_;        // linear in/out ramp time for the weld-active envelope
  float phase_     = 0.0f;   // accumulates rate*dt; integer crossings = triggers
  float rate_hz_   = 0.0f;
  float hiss_gain_ = 0.0f;
  float weld_gain_ = 0.0f;   // overall weld-active envelope [0,1]
  bool  primed_    = false;
};

// Standalone rate mapping, exposed for tests: clamp(speed/cap) lerped between
// min_hz and max_hz.
float crackleRate(float speed_mms, float min_hz, float max_hz, float speed_cap_mms);

}  // namespace audio
}  // namespace welding
