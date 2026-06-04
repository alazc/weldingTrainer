#pragma once

// WorkspaceTransform.h — Pure, header-only workspace-to-screen transform.
//
// Extracts the aspect-preserving letterbox math from Renderer so it can be
// reused by LinkageRenderer and tested without any openFrameworks dependency.
//
// Coordinate convention:
//   * Workspace: x right, y UP (engineering mm).
//   * Screen: x right, y DOWN (pixel origin at top-left).
//   The worldToScreen y-flip: workspace max_y maps to screen top (vp_y_),
//   workspace min_y maps to screen bottom (vp_y_ + vp_h_).
//
// Math is identical to Renderer::recomputeTransform / toScreen / screenToWorld
// (lines 175-224 of Renderer.cpp) so the two always stay consistent.
//
// NO openFrameworks, NO glm. Uses welding::linkage::Vec2 from FourBar.h.
// Host-compilable with plain MSVC /std:c++17.
//
// Namespace: welding::linkage

#include "FourBar.h"  // Vec2

namespace welding {
namespace linkage {

class WorkspaceTransform {
 public:
  // Workspace bounds in mm (x right, y UP).
  WorkspaceTransform(float min_x, float max_x, float min_y, float max_y)
      : min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y) {}

  // Set (or update) the pixel viewport size and recompute the letterbox rect.
  // Call once on setup and again whenever the window resizes.
  // Before setViewport is called, worldToScreen returns (0,0) for all inputs.
  void setViewport(int fbo_w, int fbo_h) {
    fbo_w_ = fbo_w;
    fbo_h_ = fbo_h;
    recomputeTransform();
  }

  // Workspace mm -> screen pixel (with y-flip: workspace y-up -> screen y-down).
  // Matches Renderer::toScreen exactly.
  Vec2 worldToScreen(float mm_x, float mm_y) const {
    const float ws_w = max_x_ - min_x_;
    const float ws_h = max_y_ - min_y_;
    const float u = (ws_w > 0.0f) ? (mm_x - min_x_) / ws_w : 0.0f;
    const float v = (ws_h > 0.0f) ? (mm_y - min_y_) / ws_h : 0.0f;
    // y-flip: v=0 (workspace bottom) -> screen bottom (vp_y_+vp_h_),
    //         v=1 (workspace top)    -> screen top    (vp_y_).
    return Vec2{vp_x_ + u * vp_w_,
                vp_y_ + (1.0f - v) * vp_h_};
  }

  // Screen pixel -> workspace mm. Inverse of worldToScreen.
  // Matches Renderer::screenToWorld exactly.
  Vec2 screenToWorld(float px, float py) const {
    const float ws_w = max_x_ - min_x_;
    const float ws_h = max_y_ - min_y_;
    const float u = (vp_w_ > 0.0f) ? (px - vp_x_) / vp_w_ : 0.0f;
    // Undo the y-flip from worldToScreen.
    const float v = (vp_h_ > 0.0f) ? 1.0f - (py - vp_y_) / vp_h_ : 0.0f;
    return Vec2{min_x_ + u * ws_w,
                min_y_ + v * ws_h};
  }

  // Accessors for the computed viewport rect (useful for drawing borders etc.)
  float vpX() const { return vp_x_; }
  float vpY() const { return vp_y_; }
  float vpW() const { return vp_w_; }
  float vpH() const { return vp_h_; }

 private:
  // Recompute the aspect-preserving letterbox rect.
  // Matches Renderer::recomputeTransform exactly (lines 175-200 of Renderer.cpp).
  void recomputeTransform() {
    const float ws_w = max_x_ - min_x_;
    const float ws_h = max_y_ - min_y_;
    if (ws_w <= 0.0f || ws_h <= 0.0f || fbo_w_ <= 0 || fbo_h_ <= 0) {
      vp_x_ = 0.0f;
      vp_y_ = 0.0f;
      vp_w_ = static_cast<float>(fbo_w_);
      vp_h_ = static_cast<float>(fbo_h_);
      return;
    }
    const float ws_aspect  = ws_w / ws_h;
    const float scr_aspect = static_cast<float>(fbo_w_) / static_cast<float>(fbo_h_);
    if (scr_aspect > ws_aspect) {
      // Screen wider than workspace -> pillarbox (bars left/right).
      vp_h_ = static_cast<float>(fbo_h_);
      vp_w_ = vp_h_ * ws_aspect;
      vp_x_ = (static_cast<float>(fbo_w_) - vp_w_) * 0.5f;
      vp_y_ = 0.0f;
    } else {
      // Screen taller than workspace -> letterbox (bars top/bottom).
      vp_w_ = static_cast<float>(fbo_w_);
      vp_h_ = vp_w_ / ws_aspect;
      vp_x_ = 0.0f;
      vp_y_ = (static_cast<float>(fbo_h_) - vp_h_) * 0.5f;
    }
  }

  // Workspace bounds (mm)
  float min_x_, max_x_, min_y_, max_y_;

  // Viewport pixel size (set via setViewport)
  int fbo_w_ = 0;
  int fbo_h_ = 0;

  // Computed letterbox rect (pixels)
  float vp_x_ = 0.0f;
  float vp_y_ = 0.0f;
  float vp_w_ = 0.0f;
  float vp_h_ = 0.0f;
};

}  // namespace linkage
}  // namespace welding
