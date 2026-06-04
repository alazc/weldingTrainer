#include "AudioEngine.h"

namespace welding {
namespace audio {

namespace {
// Crackle pops play at a fixed, modest level; the speed cue comes from the
// trigger *rate* (and the pitch jitter), not from the per-pop volume.
constexpr float kCrackleVolume = 0.6f;
constexpr float kCompleteVolume = 0.85f;   // the success chime, a touch below full
constexpr float kStartVolume = 0.7f;       // the "returned to start" blip, lighter still
constexpr const char* kHissFile = "sounds/arc_hiss.wav";
constexpr const char* kCrackleFile = "sounds/crackle.wav";
constexpr const char* kCompleteFile = "sounds/complete.wav";
constexpr const char* kStartFile = "sounds/start.wav";
}  // namespace

bool AudioEngine::setup() {
  // load() resolves relative to bin/data/. The hiss loops; the crackle is a
  // one-shot allowed to overlap (multi-play) so rapid triggers don't cut off.
  hiss_.load(kHissFile);
  crackle_.load(kCrackleFile);

  // The completion chime is independent of the arc loops: load + configure it
  // before the hiss/crackle gate so the success cue can still play even if the
  // loop assets are missing. A missing chime just means no success sound.
  complete_.load(kCompleteFile);
  if (complete_.isLoaded()) {
    complete_.setLoop(false);
    complete_.setMultiPlay(true);
    complete_.setVolume(kCompleteVolume);
  } else {
    ofLogWarning("AudioEngine")
        << "completion chime " << kCompleteFile << " missing (no success sound)";
  }

  // The "returned to start" blip mirrors the completion chime: a standalone
  // one-shot, independent of the arc loops, with the same graceful "missing
  // file -> just no blip" fallback.
  start_.load(kStartFile);
  if (start_.isLoaded()) {
    start_.setLoop(false);
    start_.setMultiPlay(true);
    start_.setVolume(kStartVolume);
  } else {
    ofLogWarning("AudioEngine")
        << "start blip " << kStartFile << " missing (no start sound)";
  }

  if (!hiss_.isLoaded() || !crackle_.isLoaded()) {
    ofLogWarning("AudioEngine")
        << "audio disabled: failed to load "
        << (hiss_.isLoaded() ? "" : kHissFile) << " "
        << (crackle_.isLoaded() ? "" : kCrackleFile)
        << "(app runs silently)";
    enabled_ = false;
    return false;
  }

  hiss_.setLoop(true);
  hiss_.setMultiPlay(false);
  hiss_.setVolume(0.0f);   // start silent; update() crossfades the gain in
  hiss_.play();

  crackle_.setLoop(false);
  crackle_.setMultiPlay(true);

  modulator_.reset();
  enabled_ = true;
  return true;
}

void AudioEngine::update(float speed_mms, float dt_s, bool weld_active) {
  if (!enabled_) return;

  const int triggers = modulator_.update(speed_mms, dt_s, weld_active);

  // Welding voices (hiss + crackle) are scaled by kWeldVolumeScale AND by the
  // weld-active envelope, so they pause and ramp in/out with the arm
  // state. The success chime is a separate voice, unscaled and ungated.
  const float weld = modulator_.weldGain();
  hiss_.setVolume(modulator_.hissGain() * welding::config::kWeldVolumeScale * weld);

  // One play per tick when any trigger fired — re-triggers the multi-play
  // sample, avoiding a burst of dozens of overlapping voices at high rate.
  // Suppressed once the envelope has ramped to ~silence (no pops in the gap).
  if (triggers > 0 && weld > 0.01f) {
    crackle_.setSpeed(
        modulator_.cracklePitch(speed_mms, welding::config::kCracklePitchSpread));
    crackle_.setVolume(kCrackleVolume * welding::config::kWeldVolumeScale * weld);
    crackle_.play();
  }
}

void AudioEngine::playStart() {
  // Independent of enabled_ (the arc loops): the start blip stands alone.
  if (start_.isLoaded()) start_.play();
}

void AudioEngine::playComplete() {
  // Independent of enabled_ (the arc loops): the success chime stands alone.
  if (complete_.isLoaded()) complete_.play();
}

void AudioEngine::stop() {
  if (complete_.isLoaded()) complete_.stop();  // stop the chime regardless
  if (start_.isLoaded()) start_.stop();        // and the start blip
  if (!enabled_) return;
  hiss_.stop();
  crackle_.stop();
}

}  // namespace audio
}  // namespace welding
