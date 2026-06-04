#include "LinkageRenderer.h"

#include <cmath>

namespace welding {
namespace render {

// =============================================================================
// Construction
// =============================================================================

LinkageRenderer::LinkageRenderer() = default;

// =============================================================================
// Lifecycle
// =============================================================================

void LinkageRenderer::setup(float ui_scale) {
  ui_scale_ = (ui_scale > 0.0f) ? ui_scale : 1.0f;
  // Initialise the letterbox transform to the current window size.
  transform_.setViewport(ofGetWidth(), ofGetHeight());
  // Scaled font for the "UNREACHABLE" indicator (bitmap font is fixed 8px).
  label_font_.load("fonts/hud.ttf",
                   static_cast<int>(std::lround(16 * ui_scale_)), true, true);
  ready_ = true;
}

void LinkageRenderer::onResize(int w, int h) {
  if (w <= 0 || h <= 0) return;
  transform_.setViewport(w, h);
}

// =============================================================================
// Per-frame kinematics update
// =============================================================================

void LinkageRenderer::update(float theta,
                              float a, float b, float c, float d,
                              welding::linkage::Vec2 o2,
                              float s, float h) {
  using namespace welding::linkage;

  // Run forward kinematics (stateless; always returns both roots).
  SolveResult sr = solve(theta, a, b, c, d, o2);

  if (!sr.feasible) {
    // Hold the last-good pose; do not append to the trace.
    infeasible_ = true;
    return;
  }

  infeasible_ = false;

  // --- Branch continuity ---------------------------------------------------
  Vec2 chosen_B;
  if (!seeded_) {
    // First feasible frame: deterministic seed via kAssemblyBranchSign.
    chosen_B = pickSeededRoot(sr.B0, sr.B1,
                              welding::config::kAssemblyBranchSign);
    seeded_ = true;
  } else {
    // Subsequent frames: nearest root to last-good B to suppress elbow-flip.
    chosen_B = pickNearestRoot(prev_B_, sr.B0, sr.B1);
  }
  prev_B_ = chosen_B;

  // --- Store the joint positions for draw() --------------------------------
  O2_ = sr.O2;
  O4_ = sr.O4;
  A_  = sr.A;
  B_  = chosen_B;
  P_  = couplerPoint(sr.A, chosen_B, s, h);

  // --- Append coupler point to trace ---------------------------------------
  trace_.append(P_);
  updated_ = true;
}

// =============================================================================
// draw — all in workspace mm via WorkspaceTransform, oF draw calls in px
// =============================================================================

void LinkageRenderer::draw() {
  if (!ready_ || !updated_) return;

  using welding::config::kLinkLineWidthPx;
  using welding::config::kJointRadiusPx;
  using welding::config::kCouplerPtRadiusPx;
  using welding::config::kTraceLineWidthPx;

  // --- Coupler trace (oldest->newest) --------------------------------------
  // Drawn first so it is behind the linkage.
  if (trace_.size() >= 2) {
    ofPushStyle();
    ofSetColor(welding::config::kColorTrace.r,
               welding::config::kColorTrace.g,
               welding::config::kColorTrace.b,
               160);   // slightly dim so bars read on top
    ofSetLineWidth(kTraceLineWidthPx * ui_scale_);

    trace_poly_.clear();
    const std::size_t n = trace_.size();
    for (std::size_t i = 0; i < n; ++i) {
      ofPoint sp = toScreen(trace_.at(i));
      trace_poly_.addVertex(sp.x, sp.y);
    }
    trace_poly_.draw();
    ofPopStyle();
  }

  // --- Link bars -----------------------------------------------------------

  // Ground link: O2 -> O4
  {
    ofPushStyle();
    ofSetColor(welding::config::kColorGround.r,
               welding::config::kColorGround.g,
               welding::config::kColorGround.b);
    ofSetLineWidth(kLinkLineWidthPx * ui_scale_);
    ofPoint so2 = toScreen(O2_);
    ofPoint so4 = toScreen(O4_);
    ofDrawLine(so2.x, so2.y, so4.x, so4.y);
    ofPopStyle();
  }

  // Input crank: O2 -> A
  {
    ofPushStyle();
    ofSetColor(welding::config::kColorInputCrank.r,
               welding::config::kColorInputCrank.g,
               welding::config::kColorInputCrank.b);
    ofSetLineWidth(kLinkLineWidthPx * ui_scale_);
    ofPoint so2 = toScreen(O2_);
    ofPoint sA  = toScreen(A_);
    ofDrawLine(so2.x, so2.y, sA.x, sA.y);
    ofPopStyle();
  }

  // Coupler bar: A -> B
  {
    ofPushStyle();
    ofSetColor(welding::config::kColorCoupler.r,
               welding::config::kColorCoupler.g,
               welding::config::kColorCoupler.b);
    ofSetLineWidth(kLinkLineWidthPx * ui_scale_);
    ofPoint sA = toScreen(A_);
    ofPoint sB = toScreen(B_);
    ofDrawLine(sA.x, sA.y, sB.x, sB.y);
    ofPopStyle();
  }

  // Output crank: B -> O4
  {
    ofPushStyle();
    ofSetColor(welding::config::kColorOutputCrank.r,
               welding::config::kColorOutputCrank.g,
               welding::config::kColorOutputCrank.b);
    ofSetLineWidth(kLinkLineWidthPx * ui_scale_);
    ofPoint sB  = toScreen(B_);
    ofPoint so4 = toScreen(O4_);
    ofDrawLine(sB.x, sB.y, so4.x, so4.y);
    ofPopStyle();
  }

  // --- Joint circles -------------------------------------------------------
  {
    ofPushStyle();
    ofFill();
    ofSetColor(welding::config::kColorJoint.r,
               welding::config::kColorJoint.g,
               welding::config::kColorJoint.b);
    for (const welding::linkage::Vec2& joint : { O2_, O4_, A_, B_ }) {
      ofPoint sp = toScreen(joint);
      ofDrawCircle(sp.x, sp.y, kJointRadiusPx * ui_scale_);
    }
    ofPopStyle();
  }

  // --- Coupler point P marker ----------------------------------------------
  {
    ofPushStyle();
    ofFill();
    ofSetColor(welding::config::kColorCouplerPt.r,
               welding::config::kColorCouplerPt.g,
               welding::config::kColorCouplerPt.b);
    ofPoint sP = toScreen(P_);
    ofDrawCircle(sP.x, sP.y, kCouplerPtRadiusPx * ui_scale_);
    ofPopStyle();
  }

  // --- Infeasible indicator ------------------------------------------------
  if (infeasible_) {
    ofPushStyle();
    ofSetColor(255, 60, 60);
    // Draw a small "UNREACHABLE" text near the top-left of the viewport.
    const float x = transform_.vpX() + 8.0f * ui_scale_;
    const float y = transform_.vpY() + label_font_.getAscenderHeight() + 8.0f * ui_scale_;
    label_font_.drawString("UNREACHABLE", x, y);
    ofPopStyle();
  }
}

// =============================================================================
// clearTrace — empty trail and reset branch continuity
// =============================================================================

void LinkageRenderer::clearTrace() {
  trace_.clear();
  seeded_     = false;
  infeasible_ = false;
  updated_    = false;
}

// =============================================================================
// Private helpers
// =============================================================================

ofPoint LinkageRenderer::toScreen(welding::linkage::Vec2 p) const {
  welding::linkage::Vec2 s = transform_.worldToScreen(p.x, p.y);
  return ofPoint(s.x, s.y);
}

ofPoint LinkageRenderer::toScreen(float mm_x, float mm_y) const {
  welding::linkage::Vec2 s = transform_.worldToScreen(mm_x, mm_y);
  return ofPoint(s.x, s.y);
}

}  // namespace render
}  // namespace welding
