#pragma once

// The openFrameworks-facing renderer. Consumes the pure-logic simulation
// core (RenderModel.{h,cpp}) and turns it into ofMesh / ofFbo / ofShader draw
// calls. The split mirrors the model/draw pattern: RenderModel is engineering-unit math with
// zero oF dependency (host-tested); this layer is the oF translation unit and
// is only ever built into the app.
//
// Pipeline (draw()):
//   1. Render the "scene" into an offscreen ofFbo:
//        * bead   — thick connected polyline, per-vertex color = heat ramp
//        * arc    — additive radial-gradient glow disc at the pen tip
//        * sparks — small additive points
//   2. Two-pass separable gaussian bloom (bright-pass + H blur -> ping,
//      V blur -> pong), shaders loaded from inline GLSL (no external files).
//   3. Composite: scene FBO to screen, blurred pong FBO additively on top.
//
// FBOs are allocated as normalized GL_TEXTURE_2D (ofDisableArbTex) so the
// bloom shaders can use sampler2D + 0..1 texcoords and stay self-contained.

#include "ofMain.h"

#include "Config.h"
#include "InputSource.h"
#include "RenderModel.h"

namespace welding {

namespace path { class Path; }  // forward-decl: active reference guide

namespace render {

class Renderer {
 public:
  Renderer();

  void setup();                 // allocate FBOs at ofGetWidth()/Height(), build shaders
  void onResize(int w, int h);  // reallocate FBOs to new size
  // advance sim. weld_active gates trail/heat/burn/spark TRACKING: only
  // while guidance is armed do we lay down bead vertices, accumulate dwell heat
  // (→ burnouts), and emit sparks. When idle (RETURN-TO-START gap, end-of-weld,
  // E-stop) the torch cursor still follows the handle and existing sparks age
  // out, but nothing new is deposited and a finished bead freezes (no decay).
  void update(const welding::input::StateSnapshot& s, float dt_s, bool weld_active);
  void draw();                  // render bead + arc + sparks, bloom, blit to screen
  void reset();                 // clear bead vertices, heat, sparks

  // --- Public transform accessors -------------------------------------------
  // ofApp shares the SAME letterboxed workspace transform as the renderer so
  // the bead lands exactly under the cursor. worldToScreen is the public alias
  // of the internal toScreen; screenToWorld is its inverse (px -> workspace mm).
  glm::vec2 worldToScreen(float mm_x, float mm_y) const { return toScreen(mm_x, mm_y); }
  glm::vec2 screenToWorld(float px, float py) const;

  // Draw the workspace boundary rectangle as a thin outline. Called from
  // draw() after the composite so the user always sees the reachable limits.
  void drawWorkspaceBounds() const;

  // Set the active reference path to draw as a faint on-screen guide.
  // Caches its sampled polyline; pass nullptr / empty path to draw nothing.
  void setPath(const welding::path::Path* p);

  // The path currently drawn as the guide, for the app's setActivePath() assert
  // that the renderer and the controller share one Path (visual == force).
  const welding::path::Path* path() const { return ref_path_; }

  // Number of blow-through (over-penetration) spots laid down this run, for the
  // HUD readout. Forwards the heat model's latched burn-event count.
  int burnThroughCount() const { return heat_.burnEvents(); }

  // Burn proximity (0..1) of the spot currently under the torch — how close the
  // arc's current heat-affected zone is to a blow-through (1.0 = burning). Feeds
  // the ERM blow-through haptic cue so the buzz tracks the visible burn.
  float currentBurnProximity() const {
    return heat_.size() ? heat_.burnProximity() : 0.0f;
  }

  // Weld-quality view: flip the bead between the live heat glow and a
  // permanent peak-heat quality map (under-fused / good / over-penetration).
  void toggleQualityView() { quality_view_ = !quality_view_; }
  bool qualityView() const { return quality_view_; }

  // Spatial conduction: 'C' in ofApp toggles it; the HUD reads the state
  // and the per-frame heat-update time for the conduction on/off latency A/B.
  void toggleConduction() { heat_.setConduction(!heat_.conductionEnabled()); }
  bool conductionEnabled() const { return heat_.conductionEnabled(); }
  long long heatUpdateMicros() const { return heat_.lastUpdateMicros(); }

 private:
  // --- workspace mm -> FBO pixel rect, aspect-preserving (letterboxed) ------
  glm::vec2 toScreen(float mm_x, float mm_y) const;
  void      recomputeTransform();   // recompute letterbox rect for current size

  void allocateFbos(int w, int h);
  void buildShaders();

  void drawScene();                 // bead + arc + sparks into scene_
  void drawReferencePath() const;   // faint hairline overlay of ref_path_

  // --- simulation state -----------------------------------------------------
  HeatModel   heat_{ welding::config::kHeatRadiusMm,
                     welding::config::kArcPower,
                     welding::config::kHeatDecayPerFrame,
                     welding::config::kHeatMax,
                     welding::config::kConduction };
  SparkSystem sparks_{ static_cast<std::size_t>(welding::config::kMaxSparks),
                       welding::config::kSparkGravityMms2,
                       welding::config::kSparkDrag,
                       welding::config::kSparkLifeS };

  // Bead vertices are owned by heat_ (posAt). We only keep the count sentinel.
  bool   has_pen_      = false;   // do we have a current pen position yet?
  glm::vec2 pen_mm_{0.0f, 0.0f};  // latest pen position (workspace mm)
  float  pen_speed_    = 0.0f;    // latest |v| (mm/s)

  // --- gl resources ---------------------------------------------------------
  ofFbo    scene_;
  ofFbo    ping_;
  ofFbo    pong_;
  ofShader bright_blur_h_;        // bright-pass + horizontal gaussian
  ofShader blur_v_;              // vertical gaussian
  bool     shaders_ok_ = false;

  bool  quality_view_ = false;    // false = live heat glow; true = weld-quality map

  int   fbo_w_ = 0;
  int   fbo_h_ = 0;

  // Letterboxed destination rect (pixels) for the workspace.
  float vp_x_ = 0.0f, vp_y_ = 0.0f, vp_w_ = 0.0f, vp_h_ = 0.0f;

  // --- active reference-path guide ------------------------------------------
  const welding::path::Path* ref_path_ = nullptr;
  std::vector<glm::vec2>     guide_mm_;   // cached polyline samples (workspace mm)
};

}  // namespace render
}  // namespace welding
