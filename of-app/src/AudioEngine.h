#pragma once

// Audio engine — the openFrameworks-facing half of the audio layer. Owns
// two ofSoundPlayer voices and drives them from the pure-logic CrackleModulator
// (AudioModel.{h,cpp}, host-tested). The split mirrors the renderer's
// model/draw split: AudioModel is unit math with zero oF dependency; this layer
// is the oF translation unit and is only ever built into the app.
//
// Voices:
//   * arc hiss — a looping ofSoundPlayer whose volume tracks the modulator's
//     smoothed hiss gain (idle <-> active crossfade).
//   * crackle  — a one-shot, multi-play sample re-triggered when the modulator
//     reports a trigger fired this tick; pitched by travel speed for variety.
//
// Failure mode (documented "no audio -> graceful skip"): if either WAV fails to
// load, the engine logs a warning, marks itself disabled, and update()/stop()
// become no-ops so the app still runs silently.

#include "ofMain.h"

#include "AudioModel.h"
#include "Config.h"

namespace welding {
namespace audio {

class AudioEngine {
 public:
  bool setup();                              // load loops; false if audio disabled
  // modulate hiss gain, trigger crackles. weld_active gates the welding voices
  // (hiss + crackle): they ramp in while armed and ramp out otherwise.
  void update(float speed_mms, float dt_s, bool weld_active);
  void playStart();                          // one-shot "returned to start / armed" blip
  void playComplete();                       // one-shot "weld complete" success chime
  void stop();                               // stop all playback (SAFE_LATCHED / exit)
  bool enabled() const { return enabled_; }  // false if setup() failed / files missing

 private:
  ofSoundPlayer hiss_;
  ofSoundPlayer crackle_;
  ofSoundPlayer complete_;   // one-shot success chime (independent of enabled_:
                             // the chime still plays if only it loaded)
  ofSoundPlayer start_;      // one-shot "returned to start" blip (same independence)

  CrackleModulator modulator_{ welding::config::kCrackleMinHz,
                               welding::config::kCrackleMaxHz,
                               welding::config::kCrackleSpeedCapMms,
                               welding::config::kHissIdleGain,
                               welding::config::kHissActiveGain,
                               welding::config::kHissActiveSpeedMms,
                               welding::config::kHissGainAlpha,
                               welding::config::kArmRampS };  // weld in/out ramp

  bool enabled_ = false;
};

}  // namespace audio
}  // namespace welding
