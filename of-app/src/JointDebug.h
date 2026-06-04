#pragma once

// JointDebug.h — pure compute behind the pantograph debug overlay.
//
// Bridges the of-app's CARTESIAN trainer world (reported tip x,y + commanded
// force fx,fy) into the pantograph's JOINT world (proximal angles theta1,theta2
// + motor torques tau1,tau2) by reusing the firmware's own kinematics.h. The
// firmware measures theta via MR sensors but only ships the FK-derived tip up
// the wire, so the of-app reconstructs the joints with invKin — faithful to
// within q15 rounding because the reported point IS fwdKin(measured theta).
//
// oF-FREE and header-only on purpose: this is the host-tested half of the split
// (mirrors FourBar/RenderModel). PantographDebugView consumes the POD and draws
// it; this file never includes ofMain.h. See test_joint_debug.cpp.
//
// The POD boundary is the "provider seam": a future wire-fed MEASURED source
// fills the same struct with no view change.

#include <cmath>

#include "InputSource.h"                                  // welding::input::StateSnapshot (oF-free)
#include "../../arduino/welding_common/kinematics.h"      // fwd/inv kinematics + jacobian

namespace welding {
namespace debug {

// One frame of reconstructed pantograph state. Pose validity and torque
// validity are SEPARATE: when the pose is unreachable the
// view holds the last-good pose and torque is meaningless, so tau must not be
// trusted (torque_valid=false) rather than computed from a stale/absent angle.
struct JointDebugState {
  float x_mm = 0.0f, y_mm = 0.0f;   // reported tip (echoed from the snapshot)
  float speed_mms = 0.0f;           // |v|
  float ex = 0.0f, ey = 0.0f;       // pin joint E in the kinematic frame (for the view)
  float theta1 = 0.0f, theta2 = 0.0f;  // proximal angles (rad); valid iff pose_valid
  float tau1 = 0.0f, tau2 = 0.0f;   // COMMANDED/modeled motor torque; valid iff torque_valid
  float det = 0.0f;                 // det(M)=r1xr2 (mm^2), the manipulability/singularity measure
                                    // |det| -> 0 at the boundary; jacobianTranspose cuts out
                                    // below welding::kSingularityDet. Valid iff pose_valid.
  bool  pose_valid = false;         // invKin found the physical assembly this frame
  bool  singular = false;           // feasible pose but Jacobian near-singular (tau unreliable)
  bool  torque_valid = false;       // pose_valid AND Jacobian non-singular
  bool  force_on = false;           // controller is actually emitting force this frame
};

// Reconstruct one frame. fx,fy are the controller's COMMANDED cartesian force
// (the caller passes 0,0 and force_on=false when the outer loop is not armed —
// E-stop / return-to-start gap / weld complete). force-at-tip == force-at-E
// because the tool offset is a fixed +y translation with fixed orientation (no
// moment), so jacobianTranspose's use of E is exact (kinematics.h:55-62).
inline JointDebugState computeJointDebugState(const welding::input::StateSnapshot& s,
                                              float fx, float fy, bool force_on) {
  JointDebugState d;
  d.x_mm = s.x;
  d.y_mm = s.y;
  d.speed_mms = std::sqrt(s.vx * s.vx + s.vy * s.vy);
  d.force_on = force_on;

  welding::invReportPoint(s.x, s.y, d.ex, d.ey);          // reported tip -> E
  d.pose_valid = welding::invKin(d.ex, d.ey, d.theta1, d.theta2);
  if (!d.pose_valid) return d;                            // hold-last-good is the view's job

  // Singularity measure: det(M) = r1 x r2, the SAME quantity jacobianTranspose
  // gates on internally (it just doesn't expose it). Recomputed here (one cross
  // product, of-app side) to keep the shared firmware header untouched. |det| -> 0
  // as the distal links approach collinear (workspace boundary).
  {
    const float o1x = -welding::kBaseMm * 0.5f, o5x = welding::kBaseMm * 0.5f;
    const float o2x = o1x + welding::kL1Mm * welding::trig::cosFast(d.theta1);
    const float o2y =       welding::kL1Mm * welding::trig::sinFast(d.theta1);
    const float o4x = o5x + welding::kL4Mm * welding::trig::cosFast(d.theta2);
    const float o4y =       welding::kL4Mm * welding::trig::sinFast(d.theta2);
    const float r1x = d.ex - o2x, r1y = d.ey - o2y;
    const float r2x = d.ex - o4x, r2y = d.ey - o4y;
    d.det = r1x * r2y - r1y * r2x;
  }

  float tau1 = 0.0f, tau2 = 0.0f;
  const bool jac_ok = welding::jacobianTranspose(d.theta1, d.theta2, d.ex, d.ey,
                                                 fx, fy, tau1, tau2);
  d.singular = !jac_ok;                                   // feasible pose, unreliable Jacobian
  d.torque_valid = jac_ok;                                // tau trusted only with a fresh, non-singular pose
  d.tau1 = tau1;                                          // jacobianTranspose zeroes these when !jac_ok
  d.tau2 = tau2;
  return d;
}

// Pure layout of the debug panel (host-tested so a future HUD tweak can't
// silently slide it under the gain panel / shape strip). Right-side panel: the
// ofxGui gain panel sits top-left and the shape strip runs along the bottom, so
// a right-aligned column from the top clears both. Tall enough to stack
// the stick figure + the time-series plots below it, but kept above the bottom
// strip (h <= 0.75 of the window).
inline void debugInsetRect(float win_w, float win_h,
                           float& x, float& y, float& w, float& h) {
  const float margin = 12.0f;
  w = win_w * 0.42f;        // wide enough for legible plot time-axes
  h = win_h * 0.90f;        // tall: stick figure + 5 stacked plots. While the
                            // overlay is on it covers the right end of the shape
                            // strip — fine, it is a toggleable debug view.
  x = win_w - w - margin;   // right-aligned
  y = margin;               // top
}

}  // namespace debug
}  // namespace welding
