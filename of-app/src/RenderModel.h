#pragma once

// Simulation core — the pure-logic half of the renderer, with zero
// openFrameworks dependency so it can be host-tested with cl.exe. The
// oF-facing Renderer (Renderer.{h,cpp}) consumes these and turns them into
// ofMesh / ofFbo / shader draw calls. The split mirrors the model/draw
// pattern: simulation is engineering-unit math; drawing is the oF translation unit.
//
// Two pieces:
//   * HeatModel    — dwell-with-decay heat field along the bead. This is
//                    the pedagogical core (speed -> bead quality), so it is
//                    the most heavily tested module here.
//   * SparkSystem  — deterministic particle integrator (pos/vel/age), RNG
//                    injected so tests are reproducible.
//
// Plus heatToColor(): the heat -> RGB ramp (cold metal -> white-hot).

#include <cstddef>
#include <functional>
#include <vector>

namespace welding {
namespace render {

struct Color { float r = 0.0f, g = 0.0f, b = 0.0f; };  // linear, 0..1
struct Vec2  { float x = 0.0f, y = 0.0f; };            // workspace mm

// =============================================================================
// HeatModel — heat-affected-zone (HAZ) heat field along the bead.
// =============================================================================
//
// Storage model: one heat cell per bead vertex, appended (with its workspace-mm
// position) as the trainee lays down the bead. The model is the single source
// of truth for vertex positions — the Renderer reads them back via
// posAt() instead of keeping a parallel copy.
//
// Deposition: a real arc delivers roughly constant *power*; a point on
// the work integrates that power over the time the arc is near it. Each frame
// accumulate() spreads a constant per-frame energy `kArcPower*dt` over every
// vertex within radius R of the torch, weighted by a triangular distance
// falloff and normalized by Σw. A vertex stays within R for ~2R/v, so total
// deposited heat ∝ 1/v — the textbook moving-heat-source (Rosenthal) law. This
// replaces the old single-vertex `dt/v` deposit, which integrated to an
// accidental 1/v² and left moving welds under-fused (the calibration gap).
//
// After deposition, ALL cells decay by `decay_per_frame` (conductive cooling),
// and optionally conduct() smooths heat between neighbors (1-D Laplacian).
// Dwelling (torch parked, many frames) piles up heat -> over-penetration; a
// fast pass deposits little -> under-fusion. The model owns the heat array so
// "slow dwell yields high heat" is directly assertable.
class HeatModel {
 public:
  // radius_mm     — HAZ radius R (mm); vertices within R receive deposit.
  // arc_power     — constant per-second energy spread across the HAZ.
  // decay_per_frame, heat_max — as before.
  // conduction_coeff — 1-D Laplacian mixing strength for conduct().
  HeatModel(float radius_mm, float arc_power, float decay_per_frame,
            float heat_max, float conduction_coeff);

  // Arm the blow-through latch: a vertex burns once its un-normalized exposure
  // dose crosses this. Default (large) = disabled until armed.
  void setBurnExposure(float exposure_threshold) { burn_exposure_ = exposure_threshold; }

  // Arm a post-burn refractory window (seconds): for this long after a new
  // burnout latches, exposure accumulation is frozen and no further burn can
  // latch — the visual analog of the ERM blow-through envelope's Refractory
  // phase, fed the SAME kErmBlowRefractoryS so the two stay in lock-step.
  // Default 0 = disabled (no behavior change until armed).
  void setBurnRefractory(float refractory_s) { burn_refractory_s_ = refractory_s; }

  // Enable/disable spatial conduction (default off). The Renderer only calls
  // conduct() when this is on; the flag lives here so the HUD can read it back.
  void setConduction(bool on) { conduction_enabled_ = on; }
  bool conductionEnabled() const { return conduction_enabled_; }

  // Append a fresh bead vertex at (x, y) with zero heat. The model stores the
  // position; accumulate() then deposits onto it whenever the torch
  // passes within R.
  void appendVertex(float x, float y);

  // Spread this frame's arc energy over the heat-affected zone around the torch
  // at (torch_x, torch_y). Weights w_i = max(0, 1 - d_i/R) (triangular falloff);
  // each in-range vertex gets `kArcPower*dt * w_i/Σw`, so the per-frame total is
  // `kArcPower*dt` regardless of vertex density. No-op if there are no vertices
  // or none lie within R (Σw == 0 guard — no divide-by-zero). Every vertex that
  // receives heat updates its peak and burn-through latch.
  void accumulate(float torch_x, float torch_y, float dt_s);

  // Spatial conduction: one explicit 1-D Laplacian smoothing step along the
  // bead, `h[i] += kConduction*(h[i-1]+h[i+1]-2h[i])`, computed into a reused
  // scratch buffer then swapped in. Endpoints use a one-sided (clamped) stencil.
  // Self-gating: identity when conduction is disabled, and a no-op on fewer than
  // 2 vertices — so the Renderer can call it every frame unconditionally. dt is
  // reserved for a future frame-rate-independent rate; the current coefficient
  // is per-frame, matching the per-frame decay model.
  void conduct(float dt_s);

  // Multiply every cell by decay_per_frame (conductive cooling). Call once
  // per rendered frame.
  void decayAll();

  std::size_t size()            const { return heat_.size(); }
  float       heatAt(std::size_t i) const { return heat_[i]; }
  float       heatMax()         const { return heat_max_; }
  // Workspace-mm position of vertex i (HeatModel owns positions).
  Vec2        posAt(std::size_t i) const { return Vec2{xs_[i], ys_[i]}; }

  // Un-normalized cumulative exposure dose at vertex i (drives blow-through).
  float exposureAt(std::size_t i) const { return exposure_[i]; }

  // Burn proximity in [0,1] of the spot currently under the torch: the largest
  // in-range exposure dose over kBurnExposure, clamped. 1.0 exactly when a burn
  // latches; 0 when over no bead or the latch is unarmed. Drives the ERM
  // blow-through haptic cue so the buzz tracks the visible burn.
  float burnProximity() const { return last_burn_proximity_; }

  // Normalized heat in [0,1] = clamp(heat / heat_max).
  float normAt(std::size_t i) const;
  // Convenience: color of vertex i through the ramp.
  Color colorAt(std::size_t i) const;

  // --- Blow-through (over-penetration) designator -------------------------
  // True once vertex i has ever crossed the burn-through threshold. Latched:
  // stays true regardless of subsequent decay.
  bool burnedAt(std::size_t i) const { return burned_[i] != 0; }
  // Number of distinct burn-through spots: a contiguous run of burned
  // vertices counts as one (rising-edge count along the bead).
  int  burnEvents() const { return burn_events_; }

  // --- Peak heat / weld-quality record ------------------------------------
  // The maximum normalized heat vertex i ever reached. Latched (only grows),
  // so it survives decay and records "what fused here" after the live bead
  // has cooled. This is the fusion-quality signal (heat is monotonic in 1/v).
  float peakAt(std::size_t i) const { return peak_[i]; }

  // Wall-clock microseconds of the most recent per-frame heat work
  // (deposit + conduct + decay). Reset by accumulate(), added to by conduct()
  // and decayAll(), so after the canonical per-frame sequence it is the whole
  // block. For the HUD conduction-on/off latency A/B. 0 before any work.
  long long lastUpdateMicros() const { return last_update_micros_; }

  void clear();

 private:
  std::vector<float> xs_, ys_;         // parallel to heat_; vertex positions (mm)
  std::vector<float> heat_;
  std::vector<char>  burned_;          // parallel to heat_; latched burn flags
  std::vector<float> peak_;            // parallel to heat_; latched max normAt
  std::vector<float> exposure_;        // parallel to heat_; un-normalized dose
  std::vector<float> scratch_;         // reused conduction work buffer
  int   burn_events_    = 0;           // contiguous burned runs
  float radius_ = 8.0f;                // HAZ radius R (mm)
  float arc_power_ = 1.0f;             // constant per-second arc energy
  float decay_  = 0.98f;
  float heat_max_ = 0.13f;
  float conduction_ = 0.15f;           // 1-D Laplacian mixing coefficient
  bool  conduction_enabled_ = false;   // runtime toggle (default off)
  float burn_exposure_ = 1e30f;        // dose latch threshold; large = disabled
  float burn_refractory_s_ = 0.0f;     // post-burn lockout window; 0 = disabled
  float refractory_remaining_ = 0.0f;  // seconds left in the current lockout
  float last_burn_proximity_ = 0.0f;   // in-range max exposure / burn_exposure_, clamped
  long long last_update_micros_ = 0;   // instrumentation
};

// --- Weld-quality classification --------------------------------------------
// Derived from a vertex's latched PEAK normalized heat. Pure + host-tested.
enum class WeldQuality { UnderFused, Good, OverPenetrated };

// Over-penetration now comes from the blow-through latch (burned), not a peak
// band; under-vs-good still splits the normalized peak at fusion_min.
WeldQuality classifyQuality(bool burned, float peak_norm, float fusion_min);

// Inspection palette: under = blue-gray, good = green, over = red-orange.
// Discrete swatch colors; used for the HUD legend.
Color qualityColor(WeldQuality q);

// Continuous quality color for the bead in quality view. Burned -> the
// over-penetration swatch; peak_norm < fusion_min -> the under-fused swatch;
// otherwise a GREEN GRADIENT brightest at fusion_ideal and dimming with |peak -
// ideal| (too thin toward fusion_min, too deep toward burn). Encodes how close
// the trainee's speed/dwell was to correct instead of a flat "good". Pure + host-
// tested. The gradient half-width is (fusion_ideal - fusion_min).
Color fusionQualityColor(bool burned, float peak_norm,
                         float fusion_min, float fusion_ideal);

// =============================================================================
// heatToColor — heat ramp, exposed standalone for tests.
// =============================================================================
// t is normalized heat in [0,1] (values outside are clamped). Ramp:
//   0.00  black            (cold, unwelded)
//   0.25  dark red         (heat-affected)
//   0.50  red-orange       (fusion)
//   0.75  orange-yellow    (good penetration)
//   1.00  yellow-white     (over-penetration / blow-through risk)
// Luminance is monotonic non-decreasing in t, which is the test invariant.
Color heatToColor(float t);

// =============================================================================
// SparkSystem — deterministic particle integrator.
// =============================================================================
struct Spark {
  float x = 0, y = 0;      // sim-frame position (mm)
  float vx = 0, vy = 0;    // sim-frame velocity (mm/s)
  float age = 0;           // seconds since spawn
  float life = 0;          // seconds to live
};

class SparkSystem {
 public:
  // rng returns a float in [0,1). Inject a deterministic one in tests; the
  // renderer passes a real PRNG. gravity is +y accel (sim frame); drag is a
  // per-frame velocity multiplier.
  using RngFn = std::function<float()>;

  SparkSystem(std::size_t max_particles,
              float gravity_mms2, float drag_per_frame, float life_s);

  // Emit up to `count` sparks at (x,y). Ejection speed scales with
  // source_speed_mms; direction is randomized via rng. Respects the hard cap
  // (extra emits beyond capacity are dropped, not reallocated).
  void emit(float x, float y, float source_speed_mms,
            int count, float speed_scale, const RngFn& rng);

  // Advance dt: integrate position, apply gravity + drag, age, cull dead.
  void update(float dt_s);

  std::size_t alive()    const { return particles_.size(); }
  std::size_t capacity() const { return cap_; }
  const std::vector<Spark>& particles() const { return particles_; }

  void clear();

 private:
  std::vector<Spark> particles_;
  std::size_t cap_     = 0;
  float gravity_       = 0.0f;
  float drag_          = 0.92f;
  float life_          = 0.5f;
};

}  // namespace render
}  // namespace welding
