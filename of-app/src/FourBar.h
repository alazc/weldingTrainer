#pragma once

// FourBar.h — Pure forward kinematics for a planar 4-bar linkage.
//
// No openFrameworks, no glm. Host-compilable with plain MSVC (/std:c++17).
// Namespace welding::linkage.
//
// Geometry:
//   O2 = input ground pivot (caller-supplied)
//   O4 = O2 + (d, 0)
//   a  = input crank length  (O2->A)
//   b  = coupler length      (A->B)
//   c  = output crank length (O4->B)
//   d  = ground length       (O2->O4)
//   θ  = input crank angle from O2 (radians, measured from +x)

#include <cmath>

namespace welding {
namespace linkage {

// ---------------------------------------------------------------------------
// Minimal 2D vector type (no dependency on glm or oF types)
// ---------------------------------------------------------------------------
struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;
};

// ---------------------------------------------------------------------------
// SolveResult — output of forward kinematics
// ---------------------------------------------------------------------------
struct SolveResult {
    Vec2 O2, O4, A;
    Vec2 B0, B1;        // the TWO assembly solutions for joint B
    bool feasible = false;
};

// ---------------------------------------------------------------------------
// solve — stateless forward kinematics. Returns BOTH circle-circle
// intersection roots plus a feasibility flag. Never picks a branch.
// o2 places the mechanism in workspace mm.
// ---------------------------------------------------------------------------
SolveResult solve(float theta, float a, float b, float c, float d,
                  Vec2 o2 = {0.0f, 0.0f});

// ---------------------------------------------------------------------------
// pickNearestRoot — continuity branch selection.
// Returns whichever of {b0, b1} is closer (Euclidean) to prev_b.
// Use each frame after frame 1 to prevent elbow-flip.
// ---------------------------------------------------------------------------
Vec2 pickNearestRoot(Vec2 prev_b, Vec2 b0, Vec2 b1);

// ---------------------------------------------------------------------------
// pickSeededRoot — deterministic first-frame selection.
// Returns b0 if branch_sign >= 0, else b1.
// ---------------------------------------------------------------------------
Vec2 pickSeededRoot(Vec2 b0, Vec2 b1, int branch_sign);

// ---------------------------------------------------------------------------
// couplerPoint — a point P rigidly attached to coupler bar AB.
//   s = fraction along A->B (0..1)
//   h = perpendicular offset in mm
//   perp = rot90(unit(B-A)) = (-dy, dx) / |B-A|
//   P = A + s*(B-A) + h * perp
// Returns A if |B-A| ~ 0 (degenerate coupler).
// ---------------------------------------------------------------------------
Vec2 couplerPoint(Vec2 a_pt, Vec2 b_pt, float s, float h);

// ---------------------------------------------------------------------------
// ThetaInterval — result of feasibleThetaInterval
// ---------------------------------------------------------------------------
struct ThetaInterval {
    float lo = 0.0f;
    float hi = 0.0f;
    bool  ok = false;
};

// ---------------------------------------------------------------------------
// feasibleThetaInterval — contiguous range of theta (radians) for which the
// linkage assembles, that CONTAINS theta_hint.
//   * Samples theta over [0, 2π) in `samples` steps.
//   * Returns the contiguous feasible run containing theta_hint.
//   * If theta_hint is infeasible, returns the nearest feasible interval.
//   * If fully rotatable (Grashof), returns {0, 2π}.
//   * If nothing is feasible, returns {theta_hint, theta_hint, ok=false}.
// ---------------------------------------------------------------------------
ThetaInterval feasibleThetaInterval(float a, float b, float c, float d,
                                    float theta_hint, int samples = 360);

}  // namespace linkage
}  // namespace welding
