#include "FourBar.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace welding {
namespace linkage {

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------
namespace {

inline float dist2(Vec2 p, Vec2 q) {
    const float dx = q.x - p.x;
    const float dy = q.y - p.y;
    return dx * dx + dy * dy;
}

inline float dist(Vec2 p, Vec2 q) {
    return std::sqrt(dist2(p, q));
}

}  // namespace

// ---------------------------------------------------------------------------
// solve
// ---------------------------------------------------------------------------
SolveResult solve(float theta, float a, float b, float c, float d,
                  Vec2 o2) {
    SolveResult r;
    r.O2 = o2;
    r.O4 = Vec2{o2.x + d, o2.y};

    // Guard non-positive link lengths
    if (a <= 0.0f || b <= 0.0f || c <= 0.0f || d <= 0.0f) {
        r.A  = o2;
        r.B0 = o2;
        r.B1 = o2;
        r.feasible = false;
        return r;
    }

    // Joint A
    r.A = Vec2{o2.x + a * std::cos(theta),
               o2.y + a * std::sin(theta)};

    // Distance A -> O4
    const float dx = r.O4.x - r.A.x;
    const float dy = r.O4.y - r.A.y;
    const float D  = std::sqrt(dx * dx + dy * dy);

    // Infeasibility tests
    if (D == 0.0f || D > b + c || D < std::abs(b - c)) {
        r.B0 = r.A;
        r.B1 = r.A;
        r.feasible = false;
        return r;
    }

    // Circle-circle intersection (law of cosines)
    //   aa = signed distance from A to chord foot along A->O4
    const float aa  = (b * b - c * c + D * D) / (2.0f * D);
    const float hh2 = b * b - aa * aa;

    if (hh2 < 0.0f) {
        r.B0 = r.A;
        r.B1 = r.A;
        r.feasible = false;
        return r;
    }

    // Unit vector A->O4 and its perpendicular
    const float ux   = dx / D;
    const float uy   = dy / D;
    const float px   = -uy;   // perp = rot90(u)
    const float py   =  ux;

    // Chord foot M
    const float mx = r.A.x + aa * ux;
    const float my = r.A.y + aa * uy;

    const float hh = std::sqrt(hh2);
    r.B0 = Vec2{mx + hh * px, my + hh * py};
    r.B1 = Vec2{mx - hh * px, my - hh * py};
    r.feasible = true;
    return r;
}

// ---------------------------------------------------------------------------
// pickNearestRoot
// ---------------------------------------------------------------------------
Vec2 pickNearestRoot(Vec2 prev_b, Vec2 b0, Vec2 b1) {
    const float d0 = dist2(prev_b, b0);
    const float d1 = dist2(prev_b, b1);
    return (d0 <= d1) ? b0 : b1;
}

// ---------------------------------------------------------------------------
// pickSeededRoot
// ---------------------------------------------------------------------------
Vec2 pickSeededRoot(Vec2 b0, Vec2 b1, int branch_sign) {
    return (branch_sign >= 0) ? b0 : b1;
}

// ---------------------------------------------------------------------------
// couplerPoint
// ---------------------------------------------------------------------------
Vec2 couplerPoint(Vec2 a_pt, Vec2 b_pt, float s, float h) {
    const float bx = b_pt.x - a_pt.x;
    const float by = b_pt.y - a_pt.y;
    const float len = std::sqrt(bx * bx + by * by);

    if (len < 1e-9f) return a_pt;

    const float ux = bx / len;
    const float uy = by / len;
    // perp = rot90(unit(B-A)) = (-uy, ux)
    const float px = -uy;
    const float py =  ux;

    return Vec2{
        a_pt.x + s * bx + h * px,
        a_pt.y + s * by + h * py
    };
}

// ---------------------------------------------------------------------------
// feasibleThetaInterval
// ---------------------------------------------------------------------------
ThetaInterval feasibleThetaInterval(float a, float b, float c, float d,
                                    float theta_hint, int samples) {
    if (samples < 2) samples = 2;

    const float two_pi = 6.28318530717958647f;
    const float step   = two_pi / static_cast<float>(samples);

    // Build a boolean feasibility array over [0, 2π).
    // Index i corresponds to theta = i * step.
    std::vector<bool> feas(static_cast<std::size_t>(samples));
    bool any_feasible   = false;
    bool any_infeasible = false;

    for (int i = 0; i < samples; ++i) {
        const float th = i * step;
        feas[static_cast<std::size_t>(i)] = solve(th, a, b, c, d).feasible;
        if (feas[static_cast<std::size_t>(i)]) any_feasible    = true;
        else                                    any_infeasible  = true;
    }

    // Nothing is feasible
    if (!any_feasible) {
        return ThetaInterval{theta_hint, theta_hint, false};
    }

    // Fully rotatable (Grashof) — every sample is feasible
    if (!any_infeasible) {
        return ThetaInterval{0.0f, two_pi, true};
    }

    // Normalise theta_hint into [0, 2π) and map to nearest sample index.
    float hint_norm = std::fmod(theta_hint, two_pi);
    if (hint_norm < 0.0f) hint_norm += two_pi;
    int hint_idx = static_cast<int>(hint_norm / step + 0.5f) % samples;

    // Find which sample index to use as the "anchor" of the target run.
    // If hint_idx is feasible, use it directly.
    // Otherwise scan outward to find the nearest feasible sample.
    int target_idx = -1;
    if (feas[static_cast<std::size_t>(hint_idx)]) {
        target_idx = hint_idx;
    } else {
        for (int k = 1; k < samples; ++k) {
            int plus  = (hint_idx + k) % samples;
            int minus = (hint_idx - k + samples) % samples;
            if (feas[static_cast<std::size_t>(plus)])  { target_idx = plus;  break; }
            if (feas[static_cast<std::size_t>(minus)]) { target_idx = minus; break; }
        }
    }

    if (target_idx < 0) {
        // Should not happen (any_feasible is true above), but guard
        return ThetaInterval{theta_hint, theta_hint, false};
    }

    // Measure the run length by walking forward from target_idx (circularly).
    // Because any_infeasible is true, there is at least one infeasible sample,
    // so the run length is strictly less than `samples`. No infinite loop risk.
    int run_len = 1;
    for (int k = 1; k < samples; ++k) {
        const int next = (target_idx + k) % samples;
        if (!feas[static_cast<std::size_t>(next)]) break;
        ++run_len;
    }

    // Walk backward from target_idx to find where the feasible run actually
    // starts.  Bug fix: the bound must be `k < samples` (not `k < run_len`),
    // because the run can extend further backward than `run_len-1` steps when
    // theta_hint lands in the middle or at the end of the arc.
    int start_offset = 0;  // how many steps back from target_idx the run starts
    for (int k = 1; k < samples; ++k) {
        const int prev = (target_idx - k + samples) % samples;
        if (!feas[static_cast<std::size_t>(prev)]) break;
        start_offset = k;
    }

    const int start_i = (target_idx - start_offset + samples) % samples;

    // Total arc width = backward extent + forward extent (both measured from
    // target_idx).  Bug fix: was `run_len` alone, which omitted start_offset
    // steps and made hi too small whenever theta_hint is not at the run start.
    const int total_steps = start_offset + run_len;

    // Convert to radians.
    // lo = angle of start_i; hi = lo + total_steps * step.
    // This is always a non-wrapping [lo, hi] with hi > lo.
    const float lo = start_i * step;
    const float hi = lo + total_steps * step;

    // Clamp hi to 2π (can exceed only by rounding; Grashof case handled above)
    const float hi_clamped = (hi > two_pi) ? two_pi : hi;
    return ThetaInterval{lo, hi_clamped, true};
}

}  // namespace linkage
}  // namespace welding
