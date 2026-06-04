#pragma once

// LinkageRenderer.h — openFrameworks drawing layer for the 4-bar linkage
// simulator.
//
// Responsibilities (this class owns):
//   1. WorkspaceTransform  — letterbox mm->px mapping.
//   2. CouplerTrace        — ring buffer of historic coupler point P positions.
//   3. Branch continuity   — calls pickSeededRoot on the first feasible frame,
//                            pickNearestRoot on every subsequent frame.
//                            On an infeasible frame, holds the last-good pose
//                            and sets a "unreachable" flag.
//
// Decomposition rationale:
//   FourBar.h holds the pure kinematics (stateless solve).
//   LinkageRenderer owns all rendering state and the continuity logic because
//   it is the only consumer — there is no separate simulation update thread
//   driving this at a different rate.  Keeping it here avoids a separate
//   FourBarSim class that would just be a thin wrapper.
//
// API (mirrors Renderer):
//   setup()            — call once after oF window is ready.
//   onResize(w,h)      — call from ofApp::windowResized.
//   update(theta,a,b,c,d,o2,s,h) — advance kinematics for the current frame.
//   draw()             — draw into the current oF context (no FBO; caller's
//                        responsibility to wrap in an FBO if desired).
//   clearTrace()       — empty the coupler-point trail.
//
// All values in workspace mm; pixels only inside draw().
//
// Namespace: welding::render

#include "ofMain.h"

#include "Config.h"
#include "CouplerTrace.h"
#include "FourBar.h"
#include "WorkspaceTransform.h"

namespace welding {
namespace render {

class LinkageRenderer {
 public:
  LinkageRenderer();

  // --- Lifecycle -----------------------------------------------------------

  // Initialise: allocate CouplerTrace, set up transform from Config defaults.
  // Call once in ofApp::setup() after the window exists. ui_scale (~1 at 96dpi,
  // larger on high-DPI) scales line widths, joint radii, and the label font.
  void setup(float ui_scale = 1.0f);

  // Recompute the letterbox transform for the new window size.
  // Call from ofApp::windowResized().
  void onResize(int w, int h);

  // --- Per-frame update -----------------------------------------------------

  // Advance kinematics for the new crank angle theta (radians).
  // Link lengths and ground pivot come from Config defaults; the caller may
  // pass overrides for interactive tuning.
  //
  //   theta — input crank angle (radians from +x, at O2)
  //   a,b,c,d — link lengths (mm); defaults from welding::config
  //   o2      — ground input pivot in workspace mm; default from Config
  //   s,h     — coupler-point parameters (fraction along A->B; perp offset mm)
  //
  // Continuity: first feasible call uses pickSeededRoot(b0,b1,kAssemblyBranchSign);
  // subsequent calls use pickNearestRoot(prev_B, b0, b1).
  // On infeasible: holds last-good joint positions, sets infeasible_ flag,
  // does NOT append to the trace.
  void update(float theta,
              float a = welding::config::kLinkA_mm,
              float b = welding::config::kLinkB_mm,
              float c = welding::config::kLinkC_mm,
              float d = welding::config::kLinkD_mm,
              welding::linkage::Vec2 o2 = { welding::config::kO2_X_mm,
                                           welding::config::kO2_Y_mm },
              float s = welding::config::kCouplerPointSDefault,
              float h = welding::config::kCouplerPointHDefaultMm);

  // --- Draw -----------------------------------------------------------------

  // Draw the linkage into the current oF context.
  // Call from ofApp::draw() (or inside an FBO begin/end block).
  // Draws:
  //   * Ground link O2->O4 (grey)
  //   * Input  crank O2->A  (orange)
  //   * Coupler bar  A->B   (yellow-green)
  //   * Output crank B->O4  (blue)
  //   * Filled joint circles at O2, A, B, O4 (white)
  //   * Coupler point P marker (magenta)
  //   * CouplerTrace polyline oldest->newest (magenta/dim)
  //   * "UNREACHABLE" text indicator when infeasible
  // Uses ofPushStyle/ofPopStyle throughout.
  void draw();

  // Clear the coupler-point trail and reset branch continuity.
  void clearTrace();

 private:
  // Helper: convert workspace Vec2 -> ofPoint for oF draw calls.
  ofPoint toScreen(welding::linkage::Vec2 p) const;
  // Overload for explicit x,y.
  ofPoint toScreen(float mm_x, float mm_y) const;

  // --- Transform ------------------------------------------------------------
  welding::linkage::WorkspaceTransform transform_{
      welding::config::kLinkageWorkspaceMinX, welding::config::kLinkageWorkspaceMaxX,
      welding::config::kLinkageWorkspaceMinY, welding::config::kLinkageWorkspaceMaxY};

  // --- Coupler trace ring buffer -------------------------------------------
  welding::linkage::CouplerTrace trace_{welding::config::kCouplerTraceMaxPoints};

  // --- Current frame joint solution ----------------------------------------
  welding::linkage::Vec2 O2_, O4_, A_, B_, P_;

  // --- Continuity state ----------------------------------------------------
  bool seeded_    = false;   // false until first feasible frame
  bool infeasible_ = false;  // true when current frame has no solution
  welding::linkage::Vec2 prev_B_{ 0.0f, 0.0f };

  // Track whether setup() has been called.
  bool ready_   = false;
  // Track whether at least one successful update() has run.
  // draw() is gated on this so zero-position joints are never rendered
  // before the first kinematics frame.
  bool updated_ = false;

  // Reusable polyline for the coupler trace; cleared each frame instead of
  // constructing a new local ofPolyline on the stack every draw call.
  ofPolyline trace_poly_;

  // --- High-DPI scaling (set in setup) -------------------------------------
  float          ui_scale_ = 1.0f;  // scales line widths / joint radii / label font
  ofTrueTypeFont label_font_;       // "UNREACHABLE" indicator (replaces bitmap font)
};

}  // namespace render
}  // namespace welding
