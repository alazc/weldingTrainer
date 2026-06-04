#include "RenderModel.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace welding {
namespace render {

namespace {
// Microseconds elapsed since a steady_clock timestamp. Used only for the
// HUD latency readout; std::chrono keeps RenderModel oF-free.
long long microsSince(std::chrono::steady_clock::time_point t0) {
  const auto dt = std::chrono::steady_clock::now() - t0;
  return std::chrono::duration_cast<std::chrono::microseconds>(dt).count();
}
}  // namespace

// =============================================================================
// HeatModel
// =============================================================================
HeatModel::HeatModel(float radius_mm, float arc_power, float decay_per_frame,
                     float heat_max, float conduction_coeff)
    : radius_(radius_mm), arc_power_(arc_power), decay_(decay_per_frame),
      heat_max_(heat_max), conduction_(conduction_coeff) {}

void HeatModel::appendVertex(float x, float y) {
  xs_.push_back(x);
  ys_.push_back(y);
  heat_.push_back(0.0f);
  burned_.push_back(0);
  peak_.push_back(0.0f);
  exposure_.push_back(0.0f);
}

void HeatModel::accumulate(float torch_x, float torch_y, float dt_s) {
  const auto t0 = std::chrono::steady_clock::now();
  // Tick the post-burn refractory clock first, before any early return, so the
  // lockout elapses on wall time even while the torch is off the bead.
  if (refractory_remaining_ > 0.0f) {
    refractory_remaining_ = std::max(0.0f, refractory_remaining_ - dt_s);
  }
  const bool in_refractory = refractory_remaining_ > 0.0f;
  const std::size_t n = heat_.size();
  if (n == 0) { last_burn_proximity_ = 0.0f; last_update_micros_ = microsSince(t0); return; }  // no vertices

  // Pass 1: total falloff weight of all vertices within R of the torch. A
  // self-crossing path can put several disjoint stretches of bead in range;
  // the full scan is correct for all of them and O(n).
  const float R = radius_;
  float sumw = 0.0f;
  for (std::size_t i = 0; i < n; ++i) {
    const float dx = xs_[i] - torch_x;
    const float dy = ys_[i] - torch_y;
    const float d = std::sqrt(dx * dx + dy * dy);
    if (d < R) sumw += 1.0f - d / R;       // triangular falloff
  }
  if (sumw <= 0.0f) { last_burn_proximity_ = 0.0f; last_update_micros_ = microsSince(t0); return; }  // Σw=0 guard

  // Pass 2: deposit normalized share so the per-frame total is kArcPower*dt
  // independent of how densely vertices were laid. Update peak/burn latches in
  // ascending index order so the contiguous-run burn count stays correct.
  const float energy = arc_power_ * dt_s;
  float max_exposure = 0.0f;
  for (std::size_t i = 0; i < n; ++i) {
    const float dx = xs_[i] - torch_x;
    const float dy = ys_[i] - torch_y;
    const float d = std::sqrt(dx * dx + dy * dy);
    if (d >= R) continue;
    const float w = 1.0f - d / R;
    heat_[i] += energy * (w / sumw);
    // During the post-burn refractory the dose is FROZEN: glow (heat_)
    // keeps building for color, but exposure makes no progress, so the window
    // is a genuine reprieve rather than a deferred burn that fires the instant
    // it lapses. Mirrors the ERM envelope, which ignores proximity while locked.
    if (!in_refractory) {
      exposure_[i] += arc_power_ * dt_s * w;   // un-normalized dose: no /Σw, no decay
    }
    if (exposure_[i] > max_exposure) max_exposure = exposure_[i];

    const float now_norm = normAt(i);
    // Peak-heat latch: max normalized heat ever seen; only grows.
    if (now_norm > peak_[i]) peak_[i] = now_norm;
    // Burn-through latch: permanent, decay never clears it. Gated by the
    // refractory so no new hole appears within kErmBlowRefractoryS of the last.
    if (!in_refractory && !burned_[i] && exposure_[i] >= burn_exposure_) {
      burned_[i] = 1;
      // New "spot" only if the path predecessor isn't already burned, so a
      // contiguous burned run counts once (rising-edge along the bead).
      if (i == 0 || !burned_[i - 1]) ++burn_events_;
      refractory_remaining_ = burn_refractory_s_;   // arm the lockout for the next frames
    }
  }
  last_burn_proximity_ = (burn_exposure_ > 0.0f)
                       ? std::min(max_exposure / burn_exposure_, 1.0f) : 0.0f;
  last_update_micros_ = microsSince(t0);
}

void HeatModel::conduct(float /*dt_s*/) {
  const auto t0 = std::chrono::steady_clock::now();
  // Self-gates on the toggle so the caller can invoke it unconditionally:
  // disabled -> identity (no smoothing), and the conduction-off timing reads
  // ~0 µs for the HUD A/B. Also a no-op on fewer than 2 vertices.
  const std::size_t n = heat_.size();
  if (!conduction_enabled_ || n < 2) { last_update_micros_ += microsSince(t0); return; }

  scratch_.resize(n);  // reused buffer; resize is a no-op once it has grown
  for (std::size_t i = 0; i < n; ++i) {
    // Endpoints use a one-sided stencil (missing neighbor = self), so the
    // Laplacian term degenerates to a single forward/backward difference.
    const float left  = (i > 0)         ? heat_[i - 1] : heat_[i];
    const float right = (i + 1 < n)     ? heat_[i + 1] : heat_[i];
    scratch_[i] = heat_[i] + conduction_ * (left + right - 2.0f * heat_[i]);
  }
  heat_.swap(scratch_);  // O(1); scratch_ now holds the pre-conduction heat
  last_update_micros_ += microsSince(t0);
}

void HeatModel::decayAll() {
  const auto t0 = std::chrono::steady_clock::now();
  for (float& h : heat_) h *= decay_;
  last_update_micros_ += microsSince(t0);
}

float HeatModel::normAt(std::size_t i) const {
  if (heat_max_ <= 0.0f) return 0.0f;
  const float t = heat_[i] / heat_max_;
  return std::max(0.0f, std::min(t, 1.0f));
}

Color HeatModel::colorAt(std::size_t i) const {
  return heatToColor(normAt(i));
}

void HeatModel::clear() {
  xs_.clear();
  ys_.clear();
  heat_.clear();
  burned_.clear();
  peak_.clear();
  exposure_.clear();
  last_burn_proximity_ = 0.0f;
  refractory_remaining_ = 0.0f;
  scratch_.clear();
  burn_events_ = 0;
}

// =============================================================================
// Weld-quality classification
// =============================================================================
WeldQuality classifyQuality(bool burned, float peak_norm, float fusion_min) {
  if (burned)                 return WeldQuality::OverPenetrated;
  if (peak_norm < fusion_min) return WeldQuality::UnderFused;
  return WeldQuality::Good;
}

Color qualityColor(WeldQuality q) {
  switch (q) {
    case WeldQuality::UnderFused:     return Color{0.25f, 0.35f, 0.60f};  // blue-gray
    case WeldQuality::Good:           return Color{0.20f, 0.75f, 0.30f};  // green
    case WeldQuality::OverPenetrated: return Color{0.90f, 0.30f, 0.10f};  // red-orange
  }
  return Color{0.5f, 0.5f, 0.5f};  // unreachable; keeps the compiler happy
}

// =============================================================================
// heatToColor — piecewise-linear ramp through 5 stops.
// =============================================================================
namespace {
Color lerp(const Color& a, const Color& b, float u) {
  return Color{ a.r + (b.r - a.r) * u,
                a.g + (b.g - a.g) * u,
                a.b + (b.b - a.b) * u };
}
}  // namespace

Color heatToColor(float t) {
  t = std::max(0.0f, std::min(t, 1.0f));

  // Five evenly-spaced stops. Each successive stop is brighter (monotonic
  // luminance), which is what makes the bead read as "hotter = lighter."
  static const Color stops[5] = {
      {0.05f, 0.05f, 0.05f},  // cold metal
      {0.45f, 0.05f, 0.02f},  // dark red
      {0.90f, 0.25f, 0.05f},  // red-orange
      {1.00f, 0.65f, 0.15f},  // orange-yellow
      {1.00f, 0.95f, 0.80f},  // yellow-white (over-penetration)
  };
  const float scaled = t * 4.0f;           // [0,4]
  int   seg = static_cast<int>(scaled);    // 0..4
  if (seg >= 4) return stops[4];
  const float u = scaled - static_cast<float>(seg);
  return lerp(stops[seg], stops[seg + 1], u);
}

Color fusionQualityColor(bool burned, float peak_norm,
                         float fusion_min, float fusion_ideal) {
  // Edge cases reuse the discrete swatches so the bead and HUD legend agree.
  if (burned)                 return qualityColor(WeldQuality::OverPenetrated);
  if (peak_norm < fusion_min) return qualityColor(WeldQuality::UnderFused);

  // Closeness to the ideal peak: 1 at fusion_ideal, falling to 0 at the band
  // edges (fusion_min below, and the same distance above). Symmetric so a too-
  // slow (too-deep) weld dims just like a too-fast (too-thin) one.
  const float band = std::max(fusion_ideal - fusion_min, 1e-3f);
  const float dev  = std::min(std::abs(peak_norm - fusion_ideal) / band, 1.0f);
  const float c    = 1.0f - dev;  // 0..1 closeness

  // Hold green hue throughout; brightness/saturation rises toward the ideal.
  static const Color kDimGreen   {0.10f, 0.30f, 0.12f};  // far from ideal
  static const Color kBrightGreen{0.20f, 0.85f, 0.35f};  // perfect weld
  return lerp(kDimGreen, kBrightGreen, c);
}

// =============================================================================
// SparkSystem
// =============================================================================
SparkSystem::SparkSystem(std::size_t max_particles,
                         float gravity_mms2, float drag_per_frame, float life_s)
    : cap_(max_particles), gravity_(gravity_mms2),
      drag_(drag_per_frame), life_(life_s) {
  particles_.reserve(cap_);
}

void SparkSystem::emit(float x, float y, float source_speed_mms,
                       int count, float speed_scale, const RngFn& rng) {
  for (int i = 0; i < count; ++i) {
    if (particles_.size() >= cap_) return;  // hard cap; drop the rest
    // Random direction over the full circle, ejection speed proportional to
    // how fast the welder is moving (faster pass throws more energetic sparks).
    const float ang   = rng() * 6.2831853f;            // [0, 2pi)
    const float speed = source_speed_mms * speed_scale * (0.5f + rng());
    Spark s;
    s.x  = x;
    s.y  = y;
    s.vx = std::cos(ang) * speed;
    s.vy = std::sin(ang) * speed;
    s.age  = 0.0f;
    s.life = life_;
    particles_.push_back(s);
  }
}

void SparkSystem::update(float dt_s) {
  for (Spark& s : particles_) {
    s.vy += gravity_ * dt_s;     // gravity pulls +y
    s.vx *= drag_;
    s.vy *= drag_;
    s.x  += s.vx * dt_s;
    s.y  += s.vy * dt_s;
    s.age += dt_s;
  }
  // Cull dead in place (swap-and-pop preserves no order, which sparks don't need).
  std::size_t w = 0;
  for (std::size_t r = 0; r < particles_.size(); ++r) {
    if (particles_[r].age < particles_[r].life) {
      if (w != r) particles_[w] = particles_[r];
      ++w;
    }
  }
  particles_.resize(w);
}

void SparkSystem::clear() {
  particles_.clear();
}

}  // namespace render
}  // namespace welding
