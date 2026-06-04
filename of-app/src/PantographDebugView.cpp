#include "PantographDebugView.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "../../arduino/welding_common/kinematics.h"  // kBaseMm, kL1Mm, kL4Mm, kSingularityDet

namespace welding {
namespace render {

namespace {
// Fixed mm window for the arm sub-rect so the arm does not rescale as it moves.
// Spans both motor pivots + full proximal reach in x, and floor->upper reach
// in y. Center is (0, 210).
constexpr float kWinMinX = -240.0f, kWinMaxX = 240.0f;
constexpr float kWinMinY = -40.0f,  kWinMaxY = 460.0f;
constexpr float kWinCx   = 0.5f * (kWinMinX + kWinMaxX);   // 0
constexpr float kWinCy   = 0.5f * (kWinMinY + kWinMaxY);   // 210
}  // namespace

void PantographDebugView::setup(float ui_scale) {
  ui_scale_ = ui_scale;
  font_.load("fonts/hud.ttf", static_cast<int>(std::lround(11 * ui_scale)), true, true);
}

ofPoint PantographDebugView::toInset(float mm_x, float mm_y,
                                     const ofRectangle& r) const {
  const float sx = r.width  / (kWinMaxX - kWinMinX);
  const float sy = r.height / (kWinMaxY - kWinMinY);
  const float s  = std::min(sx, sy);
  const float cx = r.x + r.width  * 0.5f;
  const float cy = r.y + r.height * 0.5f;
  return ofPoint(cx + (mm_x - kWinCx) * s,
                 cy - (mm_y - kWinCy) * s);   // y-flip: screen grows downward
}

void PantographDebugView::drawPlot(const ofRectangle& r, const std::string& title,
                                   const welding::debug::DebugHistory& hist,
                                   const std::vector<PlotLine>& lines,
                                   bool has_threshold, float threshold) {
  // Panel + border.
  ofSetColor(14, 16, 24, 180);
  ofDrawRectangle(r);
  ofNoFill();
  ofSetColor(50, 58, 74);
  ofDrawRectangle(r);
  ofFill();

  // Auto-range over every line in the buffer, always including 0 (so a flat-zero
  // signal sits on the baseline rather than filling the plot with noise).
  float ymin = 0.0f, ymax = 0.0f;
  const std::size_t n = hist.size();
  for (std::size_t i = 0; i < n; ++i) {
    const auto& s = hist.at(i);
    for (const auto& ln : lines) {
      const float v = ln.get(s);
      ymin = std::min(ymin, v);
      ymax = std::max(ymax, v);
    }
  }
  if (has_threshold) ymax = std::max(ymax, threshold * 1.1f);
  if (ymax - ymin < 1e-3f) ymax = ymin + 1.0f;          // degenerate guard
  const float pad = 0.08f * (ymax - ymin);
  ymin -= pad; ymax += pad;

  // Title strip on top; the series live in the area below it.
  const float titleH = font_.getLineHeight();
  const ofRectangle pr(r.x + 2, r.y + titleH, r.width - 4, r.height - titleH - 2);

  // Zero baseline (faint) when the range straddles 0.
  if (ymin < 0.0f && ymax > 0.0f) {
    ofSetColor(70, 78, 92);
    const float zy = welding::debug::plotY(0.0f, ymin, ymax, pr.y, pr.height);
    ofDrawLine(pr.x, zy, pr.x + pr.width, zy);
  }
  // Singularity threshold line (red), if any.
  if (has_threshold) {
    ofSetColor(220, 70, 70, 200);
    const float ty = welding::debug::plotY(threshold, ymin, ymax, pr.y, pr.height);
    ofDrawLine(pr.x, ty, pr.x + pr.width, ty);
  }

  // Each series as a polyline (reused scratch_, no per-frame alloc).
  ofSetLineWidth(std::max(1.0f, 1.3f * ui_scale_));
  for (const auto& ln : lines) {
    scratch_.clear();
    for (std::size_t i = 0; i < n; ++i) {
      const float x = welding::debug::plotX(i, n, pr.x, pr.width);
      const float y = welding::debug::plotY(ln.get(hist.at(i)), ymin, ymax, pr.y, pr.height);
      scratch_.addVertex(x, y);
    }
    ofSetColor(ln.color);
    scratch_.draw();
  }

  // Title + the newest value of the first line.
  ofSetColor(190, 205, 225);
  std::string label = title;
  if (n > 0 && !lines.empty()) {
    char v[48];
    std::snprintf(v, sizeof(v), "  = %.1f", lines.front().get(hist.latest()));
    label += v;
  }
  font_.drawString(label, r.x + 4, r.y + titleH - 3);
}

void PantographDebugView::draw(const welding::debug::JointDebugState& d,
                               const ofRectangle& panel,
                               const welding::debug::DebugHistory& hist,
                               const std::string& force_off_reason) {
  ofPushStyle();

  // --- Panel background ------------------------------------------------------
  ofSetColor(8, 10, 16, 215);
  ofDrawRectangle(panel);
  ofNoFill();
  ofSetColor(60, 70, 90);
  ofSetLineWidth(1.0f);
  ofDrawRectangle(panel);
  ofFill();

  const float pad = 8.0f * ui_scale_;

  // --- Arm sub-rect (top ~22%; the rest goes to the stacked plots) ----------
  const ofRectangle arm(panel.x, panel.y, panel.width, panel.height * 0.22f);

  float t1, t2, ex, ey;
  bool have_pose = true;
  if (d.pose_valid) {
    t1 = d.theta1; t2 = d.theta2; ex = d.ex; ey = d.ey;
    last_t1_ = t1; last_t2_ = t2; last_ex_ = ex; last_ey_ = ey;
    has_last_ = true;
  } else if (has_last_) {
    t1 = last_t1_; t2 = last_t2_; ex = last_ex_; ey = last_ey_;   // frozen
  } else {
    have_pose = false;
  }

  if (have_pose) {
    const float o1x = -welding::kBaseMm * 0.5f, o5x = welding::kBaseMm * 0.5f;
    const ofPoint O1 = toInset(o1x, 0.0f, arm);
    const ofPoint O5 = toInset(o5x, 0.0f, arm);
    const ofPoint O2 = toInset(o1x + welding::kL1Mm * std::cos(t1),
                               welding::kL1Mm * std::sin(t1), arm);
    const ofPoint O4 = toInset(o5x + welding::kL4Mm * std::cos(t2),
                               welding::kL4Mm * std::sin(t2), arm);
    const ofPoint E  = toInset(ex, ey, arm);

    const float lw = std::max(2.0f, 3.0f * ui_scale_);
    ofSetLineWidth(std::max(1.0f, 1.5f * ui_scale_));
    ofSetColor(110, 110, 120); ofDrawLine(O1, O5);                 // base
    ofSetLineWidth(lw);
    ofSetColor(255, 150, 40); ofDrawLine(O1, O2); ofDrawLine(O5, O4);  // proximal
    ofSetColor(170, 220, 60);  ofDrawLine(O2, E);                  // distal L
    ofSetColor(70, 150, 255);  ofDrawLine(O4, E);                  // distal R
    const float jr = std::max(3.0f, 4.0f * ui_scale_);
    ofSetColor(230); ofDrawCircle(O1, jr); ofDrawCircle(O5, jr);
    ofDrawCircle(O2, jr); ofDrawCircle(O4, jr);
    ofSetColor(255, 60, 220); ofDrawCircle(E, jr * 1.3f);          // end-effector
  }

  // --- Numeric readout + state (below the arm) -------------------------------
  float ty = arm.y + arm.height + pad + font_.getLineHeight();
  const float tx = panel.x + pad;
  auto line = [&](const std::string& s, const ofColor& c) {
    ofSetColor(c);
    font_.drawString(s, tx, ty);
    ty += font_.getLineHeight() * 1.2f;
  };

  char buf[96];
  const ofColor label(180, 200, 220);
  std::snprintf(buf, sizeof(buf), "x,y = %6.1f, %6.1f mm   |v| = %5.1f", d.x_mm, d.y_mm, d.speed_mms);
  line(buf, label);
  if (d.pose_valid || has_last_) {
    std::snprintf(buf, sizeof(buf), "th1 = %5.1f   th2 = %5.1f deg",
                  ofRadToDeg(d.pose_valid ? d.theta1 : last_t1_),
                  ofRadToDeg(d.pose_valid ? d.theta2 : last_t2_));
    line(buf, ofColor(255, 180, 90));
  }
  if (d.torque_valid) {
    std::snprintf(buf, sizeof(buf), "tau1 = %6.1f  tau2 = %6.1f (cmd)", d.tau1, d.tau2);
    line(buf, ofColor(120, 220, 255));
  } else {
    line("tau1 =   --   tau2 =   --", ofColor(150, 150, 160));
  }
  std::snprintf(buf, sizeof(buf), "|det| = %.0f  (singular < %.0f)",
                std::fabs(d.det), welding::kSingularityDet);
  line(buf, ofColor(210, 200, 110));

  // State description.
  if (!d.pose_valid)      line("STATE: UNREACHABLE", ofColor(255, 80, 80));
  else if (d.singular)    line("STATE: SINGULAR (near boundary)", ofColor(255, 200, 60));
  else if (!d.force_on)   line("STATE: FORCE OFF: " + force_off_reason, ofColor(255, 120, 120));
  else                    line("STATE: ARMED", ofColor(120, 235, 160));

  // --- Time-series plots (stacked in the remaining space) --------------------
  const float plots_top = ty + pad * 0.5f;
  const float region_h  = (panel.y + panel.height - pad) - plots_top;
  if (region_h > 40.0f) {
    const int   kN   = 5;
    const float gap  = 6.0f;
    const float ph   = (region_h - gap * (kN - 1)) / kN;
    const float px   = panel.x + pad;
    const float pw   = panel.width - 2 * pad;
    float py = plots_top;
    using S = welding::debug::DebugSample;

    drawPlot(ofRectangle(px, py, pw, ph), "torque tau1",
             hist, { {[](const S& s){ return s.tau1; }, ofColor(170, 220, 60)} });
    py += ph + gap;
    drawPlot(ofRectangle(px, py, pw, ph), "torque tau2",
             hist, { {[](const S& s){ return s.tau2; }, ofColor(70, 150, 255)} });
    py += ph + gap;
    // Total ERM duty only (the base/detent/blow breakdown was cluttered; the
    // components still live in DebugSample if a breakdown view is wanted later).
    drawPlot(ofRectangle(px, py, pw, ph), "ERM pwm",
             hist, { {[](const S& s){ return s.erm_pwm; }, ofColor(235)} });
    py += ph + gap;
    drawPlot(ofRectangle(px, py, pw, ph), "singularity |det|",
             hist, { {[](const S& s){ return std::fabs(s.det); }, ofColor(210, 200, 110)} },
             /*has_threshold=*/true, welding::kSingularityDet);
    py += ph + gap;
    drawPlot(ofRectangle(px, py, pw, ph), "speed |v|",
             hist, { {[](const S& s){ return s.speed; },  ofColor(80, 220, 230)},
                     {[](const S& s){ return s.target; }, ofColor(120, 120, 140, 170)} });
  }

  ofPopStyle();
}

}  // namespace render
}  // namespace welding
