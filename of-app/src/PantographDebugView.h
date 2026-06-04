#pragma once

// PantographDebugView.h — openFrameworks inset overlay for the 'D' debug menu.
// Draws the reconstructed pantograph pose ("two sticks rotating" per
// side) plus a numeric readout, inside a corner inset over the live trainer
// scene. Trainer mode only.
//
// This is the oF half of the split: it consumes a pure welding::debug::
// JointDebugState (computed host-side in JointDebug.h) and turns it into draw
// calls. Mirrors the FourBar/LinkageRenderer and RenderModel/Renderer pattern —
// no kinematics math lives here.
//
// Holds the last feasible pose so an UNREACHABLE frame freezes the arm instead
// of blanking it (same UX as LinkageRenderer's infeasible-hold). theta1,theta2
// arrive already constrained to (0,pi) by invKin, so there is no atan2 wrap to
// unfold here.

#include <functional>
#include <string>
#include <vector>

#include "ofMain.h"

#include "JointDebug.h"
#include "DebugHistory.h"

namespace welding {
namespace render {

class PantographDebugView {
 public:
  // Load the readout font (mirrors LinkageRenderer). ui_scale ~1 at 96dpi.
  void setup(float ui_scale = 1.0f);

  // Draw the overlay into `panel` (pixels): stick figure + numerics on top, then
  // stacked time-series plots (torque / ERM / singularity / speed) from `hist`
  // below. `force_off_reason` is shown when d.force_on is false (E-STOP/GAP/DONE).
  void draw(const welding::debug::JointDebugState& d, const ofRectangle& panel,
            const welding::debug::DebugHistory& hist,
            const std::string& force_off_reason);

 private:
  // Map a kinematic-frame point (mm) into a rect (px), aspect-preserving with a
  // y-flip. Uses a fixed mm window so the arm does not rescale as it moves.
  ofPoint toInset(float mm_x, float mm_y, const ofRectangle& r) const;

  // One series within a plot: a value extractor + a colour.
  struct PlotLine {
    std::function<float(const welding::debug::DebugSample&)> get;
    ofColor color;
  };
  // Draw one time-series plot in `r`: auto-ranged (always includes 0), oldest
  // sample at the left. Optional horizontal threshold line (e.g. singularity
  // cutoff). The newest value of the FIRST line is echoed in the title.
  void drawPlot(const ofRectangle& r, const std::string& title,
                const welding::debug::DebugHistory& hist,
                const std::vector<PlotLine>& lines,
                bool has_threshold = false, float threshold = 0.0f);

  bool  has_last_ = false;   // have we ever seen a feasible pose?
  float last_t1_ = 0.0f, last_t2_ = 0.0f;
  float last_ex_ = 0.0f, last_ey_ = 0.0f;

  float          ui_scale_ = 1.0f;
  ofTrueTypeFont font_;
  ofPolyline     scratch_;   // reused per plot line (no per-frame alloc)
};

}  // namespace render
}  // namespace welding
