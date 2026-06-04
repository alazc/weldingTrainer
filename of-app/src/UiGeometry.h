#pragma once

// UiGeometry — tiny pure UI math for the shape-library strip. No openFrameworks,
// no glm, so it is host-testable with cl.exe (mirrors the model/draw split). The
// oF layer (ofApp) adapts these to ofRectangle / glm::vec2 / ofDrawX calls.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace welding {
namespace ui {

struct Pt   { float x = 0.0f, y = 0.0f; };
struct Rect { float x = 0.0f, y = 0.0f, w = 0.0f, h = 0.0f; };

// Fit `pts` (in their own arbitrary coordinate space, e.g. workspace mm) into
// `r`, preserving aspect ratio and centering, with `pad` pixels of inset on
// every side. Used to draw a shape's own polyline as a small icon thumbnail.
//
// `flip_y`: workspace mm uses y-up (the tip frame's y grows upward, downward
// into the work is negative), while screen pixels are y-down. Pass true so the
// thumbnail reads the same way up as the main on-screen guide.
//
// Degenerate inputs are handled without NaN/inf: empty -> empty; a single point
// or a zero-extent axis -> centered in the rect (scale on that axis is 0).
inline std::vector<Pt> fitPolylineToRect(const std::vector<Pt>& pts,
                                         Rect r, float pad, bool flip_y = true) {
  std::vector<Pt> out;
  if (pts.empty()) return out;

  float minx = pts[0].x, maxx = pts[0].x;
  float miny = pts[0].y, maxy = pts[0].y;
  for (const auto& p : pts) {
    minx = std::min(minx, p.x); maxx = std::max(maxx, p.x);
    miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
  }

  const float avail_w = std::max(0.0f, r.w - 2.0f * pad);
  const float avail_h = std::max(0.0f, r.h - 2.0f * pad);
  const float span_x  = maxx - minx;
  const float span_y  = maxy - miny;

  // Uniform scale = the tighter of the two axis fits (0 if both spans are 0).
  float sx = (span_x > 1e-6f) ? (avail_w / span_x) : 0.0f;
  float sy = (span_y > 1e-6f) ? (avail_h / span_y) : 0.0f;
  float s;
  if (span_x > 1e-6f && span_y > 1e-6f)      s = std::min(sx, sy);
  else if (span_x > 1e-6f)                   s = sx;
  else if (span_y > 1e-6f)                   s = sy;
  else                                       s = 0.0f;  // single point

  // Center the scaled bbox in the rect.
  const float scaled_w = span_x * s;
  const float scaled_h = span_y * s;
  const float ox = r.x + pad + (avail_w - scaled_w) * 0.5f;
  const float oy = r.y + pad + (avail_h - scaled_h) * 0.5f;

  out.reserve(pts.size());
  for (const auto& p : pts) {
    const float fx = ox + (p.x - minx) * s;
    float fy;
    if (flip_y) fy = oy + (maxy - p.y) * s;   // invert: world y-up -> screen y-down
    else        fy = oy + (p.y - miny) * s;
    out.push_back(Pt{fx, fy});
  }
  return out;
}

// True if (px,py) is within `radius` of (cx,cy). Used for the "handle reached
// the shape's start" re-arm check (force-feedback GAP -> ARMED).
inline bool withinCapture(float px, float py, float cx, float cy, float radius) {
  const float dx = px - cx, dy = py - cy;
  return (dx * dx + dy * dy) <= (radius * radius);
}

}  // namespace ui
}  // namespace welding
