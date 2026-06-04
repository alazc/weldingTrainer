#ifndef WELDING_KINEMATICS_H
#define WELDING_KINEMATICS_H

// Pantograph forward kinematics + analytical Jacobian transpose.
//
// Why a header-only library: the same source must compile twice — once for
// AVR via arduino-cli, once for the host test runner via MSVC. The two
// toolchains do not share an object format, so every translation unit
// recompiles from source. Header-only keeps test parity guaranteed.
//
// Coordinate frame: origin at the midpoint of the two motor pivots,
// +x toward the right motor, +y into the workspace (up).
//
// Five-bar (2-DOF) parallel linkage (a pantograph). Four moving links + the
// fixed base as the fifth bar. Driven through a capstan cable transmission
// (reference ratio RS/RP ~ 15.1); the capstan reduction is currently folded
// into kTorqueToPwmScale (see inner_loop.h) — a dedicated capstan term is a
// deferred calibration item.
//   O1 = (-base/2, 0)   left motor pivot  (doc joint A1, angle theta1)
//   O5 = (+base/2, 0)   right motor pivot (doc joint A5, angle theta2)
//   Link L1 (proximal left)  : O1 -> O2,  angle theta1 CCW from +x
//   Link L2 (distal  left)   : O2 -> E
//   Link L3 (distal  right)  : O4 -> E
//   Link L4 (proximal right) : O5 -> O4,  angle theta2 CCW from +x
//
// Workspace: theta1, theta2 in (0, pi); end-effector in +y half-plane.

#ifdef ARDUINO
  #include <Arduino.h>
  #include <avr/pgmspace.h>
  #define WELDING_PROGMEM PROGMEM
  #define WELDING_PGM_READ_FLOAT(p) pgm_read_float(p)
#else
  #include <cmath>
  #include <cstddef>
  #define WELDING_PROGMEM
  #define WELDING_PGM_READ_FLOAT(p) (*(p))
#endif

namespace welding {

// --- Pantograph dimensions (measured on the rig) ------------------------------
// All four moving links are 6 in = 152.4 mm pivot-to-pivot; the base
// (motor-to-motor spacing) is 8.7 cm = 87.0 mm. Calibration is firmware-baked:
// if the physical rig changes, update these constants and reflash.
constexpr float kBaseMm = 87.0f;    // 8.7 cm, A1<->A5 motor-pivot spacing
constexpr float kL1Mm   = 152.4f;   // 6 in, proximal left
constexpr float kL2Mm   = 152.4f;   // 6 in, distal left
constexpr float kL3Mm   = 152.4f;   // 6 in, distal right
constexpr float kL4Mm   = 152.4f;   // 6 in, proximal right

// --- End-effector (torch) tip + reported workspace coordinate ----------------
// The torch is a rigid extension from the pin joint E, assumed to always face
// "straight" (+y, the device symmetry axis). The tracked point is its tip:
//   tip = E + (0, kEndEffectorMm)
// Because the orientation is fixed, the tip is a CONSTANT translation of E, so
// the Jacobian is unchanged (jacobianTranspose keeps using E) and the tip
// velocity equals E's.
constexpr float kEndEffectorMm = 243.3574f;  // 9.581 in

// The absolute tip sits far outside the q15 +/-256 mm wire range (centerline
// tip ~542 mm from the motor midpoint), so the *reported* position is the tip
// re-centered on a workspace origin: the centerline pose (theta1=theta2=pi/2)
// tip. Recompute if the link dimensions change; the CPU side must apply the
// SAME origin (of-app frame reconciliation).
constexpr float kReportOriginYMm = 541.82f;  // = centerline E_y (298.46) + kEndEffectorMm

// |det(M)| below this means distal links are nearly collinear (workspace
// boundary). The outer-loop controller must scale force commands toward zero
// as |det| approaches this. Returned as a hard cutoff from jacobianTranspose.
constexpr float kSingularityDet = 50.0f;  // mm^2

// --- Trig LUT ----------------------------------------------------------------
// 256-segment quarter-period sin table with linear interpolation. Quadrant
// folding handles the full circle. Worst-case error vs std::sin is ~6e-6 (the
// linear interpolation between adjacent samples spaced pi/512 rad apart).
// On AVR, the table lives in PROGMEM so it costs flash, not the 2 KB SRAM.

namespace trig {

constexpr int   kLutN    = 256;
constexpr float kPi      = 3.14159265358979323846f;
constexpr float kTwoPi   = 6.28318530717958647692f;
constexpr float kHalfPi  = 1.57079632679489661923f;

// static: each translation unit gets its own copy. The two TUs in this
// project (the .ino sketch and the host test) each link into their own
// binary, so there is no ODR conflict and the PROGMEM attribute applies
// cleanly on AVR.
static const float kSinQ[kLutN + 1] WELDING_PROGMEM = {
  0.00000000f, 0.00613588f, 0.01227154f, 0.01840673f, 0.02454123f, 0.03067480f, 0.03680722f, 0.04293826f,
  0.04906767f, 0.05519524f, 0.06132074f, 0.06744392f, 0.07356456f, 0.07968244f, 0.08579731f, 0.09190896f,
  0.09801714f, 0.10412163f, 0.11022221f, 0.11631863f, 0.12241068f, 0.12849811f, 0.13458071f, 0.14065824f,
  0.14673047f, 0.15279719f, 0.15885814f, 0.16491312f, 0.17096189f, 0.17700422f, 0.18303989f, 0.18906866f,
  0.19509032f, 0.20110463f, 0.20711138f, 0.21311032f, 0.21910124f, 0.22508391f, 0.23105811f, 0.23702361f,
  0.24298018f, 0.24892761f, 0.25486566f, 0.26079412f, 0.26671276f, 0.27262136f, 0.27851969f, 0.28440754f,
  0.29028468f, 0.29615089f, 0.30200595f, 0.30784964f, 0.31368174f, 0.31950203f, 0.32531029f, 0.33110631f,
  0.33688985f, 0.34266072f, 0.34841868f, 0.35416353f, 0.35989504f, 0.36561300f, 0.37131719f, 0.37700741f,
  0.38268343f, 0.38834505f, 0.39399204f, 0.39962420f, 0.40524131f, 0.41084317f, 0.41642956f, 0.42200027f,
  0.42755509f, 0.43309382f, 0.43861624f, 0.44412214f, 0.44961133f, 0.45508359f, 0.46053871f, 0.46597650f,
  0.47139674f, 0.47679923f, 0.48218377f, 0.48755016f, 0.49289819f, 0.49822767f, 0.50353838f, 0.50883014f,
  0.51410274f, 0.51935599f, 0.52458968f, 0.52980362f, 0.53499762f, 0.54017147f, 0.54532499f, 0.55045797f,
  0.55557023f, 0.56066158f, 0.56573181f, 0.57078075f, 0.57580819f, 0.58081396f, 0.58579786f, 0.59075970f,
  0.59569930f, 0.60061648f, 0.60551104f, 0.61038281f, 0.61523159f, 0.62005721f, 0.62485949f, 0.62963824f,
  0.63439328f, 0.63912444f, 0.64383154f, 0.64851440f, 0.65317284f, 0.65780669f, 0.66241578f, 0.66699992f,
  0.67155895f, 0.67609270f, 0.68060100f, 0.68508367f, 0.68954054f, 0.69397146f, 0.69837625f, 0.70275474f,
  0.70710678f, 0.71143220f, 0.71573083f, 0.72000251f, 0.72424708f, 0.72846439f, 0.73265427f, 0.73681657f,
  0.74095113f, 0.74505779f, 0.74913639f, 0.75318680f, 0.75720885f, 0.76120239f, 0.76516727f, 0.76910334f,
  0.77301045f, 0.77688847f, 0.78073723f, 0.78455660f, 0.78834643f, 0.79210658f, 0.79583690f, 0.79953727f,
  0.80320753f, 0.80684755f, 0.81045720f, 0.81403633f, 0.81758481f, 0.82110251f, 0.82458930f, 0.82804505f,
  0.83146961f, 0.83486287f, 0.83822471f, 0.84155498f, 0.84485357f, 0.84812034f, 0.85135519f, 0.85455799f,
  0.85772861f, 0.86086694f, 0.86397286f, 0.86704625f, 0.87008699f, 0.87309498f, 0.87607009f, 0.87901223f,
  0.88192126f, 0.88479710f, 0.88763962f, 0.89044872f, 0.89322430f, 0.89596625f, 0.89867447f, 0.90134885f,
  0.90398929f, 0.90659570f, 0.90916798f, 0.91170603f, 0.91420976f, 0.91667906f, 0.91911385f, 0.92151404f,
  0.92387953f, 0.92621024f, 0.92850608f, 0.93076696f, 0.93299280f, 0.93518351f, 0.93733901f, 0.93945922f,
  0.94154407f, 0.94359346f, 0.94560733f, 0.94758559f, 0.94952818f, 0.95143502f, 0.95330604f, 0.95514117f,
  0.95694034f, 0.95870347f, 0.96043052f, 0.96212140f, 0.96377607f, 0.96539444f, 0.96697647f, 0.96852209f,
  0.97003125f, 0.97150389f, 0.97293995f, 0.97433938f, 0.97570213f, 0.97702814f, 0.97831737f, 0.97956977f,
  0.98078528f, 0.98196387f, 0.98310549f, 0.98421009f, 0.98527764f, 0.98630810f, 0.98730142f, 0.98825757f,
  0.98917651f, 0.99005821f, 0.99090264f, 0.99170975f, 0.99247953f, 0.99321195f, 0.99390697f, 0.99456457f,
  0.99518473f, 0.99576741f, 0.99631261f, 0.99682030f, 0.99729046f, 0.99772307f, 0.99811811f, 0.99847558f,
  0.99879546f, 0.99907773f, 0.99932238f, 0.99952942f, 0.99969882f, 0.99983058f, 0.99992470f, 0.99998118f,
  1.00000000f
};

// Read kSinQ at float index in [0, kLutN] with linear interpolation.
inline float sinQuarter(float idxF) {
  if (idxF <= 0.0f) return 0.0f;
  if (idxF >= (float)kLutN) return 1.0f;
  int i = (int)idxF;
  float frac = idxF - (float)i;
  float a = WELDING_PGM_READ_FLOAT(&kSinQ[i]);
  float b = WELDING_PGM_READ_FLOAT(&kSinQ[i + 1]);
  return a + (b - a) * frac;
}

// LUT-based sin. Wraps argument to [0, 2*pi), then quadrant-folds.
// The while-loops handle the small-multiple-of-pi range typical of motor
// angles; avoids fmod() which on AVR pulls in 1.5 KB of soft-float code.
inline float sinFast(float rad) {
  float t = rad;
  while (t < 0.0f)     t += kTwoPi;
  while (t >= kTwoPi)  t -= kTwoPi;
  const float scale = (float)kLutN / kHalfPi;
  if (t < kHalfPi)             return  sinQuarter(t * scale);
  if (t < kPi)                 return  sinQuarter((kPi - t) * scale);
  if (t < kPi + kHalfPi)       return -sinQuarter((t - kPi) * scale);
  /* t in [3pi/2, 2pi) */      return -sinQuarter((kTwoPi - t) * scale);
}

inline float cosFast(float rad) {
  return sinFast(rad + kHalfPi);
}

}  // namespace trig

// --- Newton-Raphson sqrt -----------------------------------------------------
// Two NR iterations starting from a bit-cast initial guess. On AVR (no FPU)
// this is faster than the libm sqrtf because the soft-float divide dominates;
// the NR form does two divides instead of libm's polynomial reduction.
// On host, fall through to std::sqrt — clarity matters more than speed there,
// and the host test only validates correctness, not performance.

namespace math {

inline float nrSqrt(float x) {
  if (x <= 0.0f) return 0.0f;
#ifdef ARDUINO
  union { float f; unsigned long i; } u;
  u.f = x;
  u.i = (u.i + 0x3F800000UL) >> 1;
  float y = u.f;
  y = 0.5f * (y + x / y);
  y = 0.5f * (y + x / y);
  return y;
#else
  return std::sqrt(x);
#endif
}

// Portable atan2. Only used by invKin (host / of-app side), never on the AVR
// 1 kHz path — but the header must still compile for AVR, where <cmath> is not
// pulled in. avr-libc exposes atan2 via Arduino.h; the host uses std::atan2.
inline float atan2safe(float y, float x) {
#ifdef ARDUINO
  return atan2(y, x);
#else
  return std::atan2(y, x);
#endif
}

}  // namespace math

// --- Forward kinematics ------------------------------------------------------
// theta1, theta2 in radians. On success writes (x, y) in mm and returns true.
// On reach failure (sqrt of negative, triangle inequality, etc.) leaves
// (x, y) untouched and returns false — caller holds last-valid pose.

inline bool fwdKin(float theta1, float theta2, float& x, float& y) {
  const float o1x = -kBaseMm * 0.5f;
  const float o5x = +kBaseMm * 0.5f;
  const float c1 = trig::cosFast(theta1);
  const float s1 = trig::sinFast(theta1);
  const float c2 = trig::cosFast(theta2);
  const float s2 = trig::sinFast(theta2);

  const float o2x = o1x + kL1Mm * c1;
  const float o2y =        kL1Mm * s1;
  const float o4x = o5x + kL4Mm * c2;
  const float o4y =        kL4Mm * s2;

  const float dx = o4x - o2x;
  const float dy = o4y - o2y;
  const float dSq = dx * dx + dy * dy;
  if (dSq <= 0.0f) return false;
  const float d = math::nrSqrt(dSq);

  const float maxReach = kL2Mm + kL3Mm;
  const float minReach = (kL2Mm > kL3Mm) ? (kL2Mm - kL3Mm) : (kL3Mm - kL2Mm);
  if (d > maxReach || d < minReach) return false;

  // Distance from O2 along O2->O4 to the foot of perpendicular from E.
  const float a = (kL2Mm * kL2Mm - kL3Mm * kL3Mm + dSq) / (2.0f * d);
  const float hSq = kL2Mm * kL2Mm - a * a;
  if (hSq < 0.0f) return false;
  const float h = math::nrSqrt(hSq);

  // Foot of perpendicular.
  const float fx = o2x + (a / d) * dx;
  const float fy = o2y + (a / d) * dy;

  // The two roots lie along the perpendicular to O2->O4 through F. The "upper
  // root" is the one with greater y — robust to which side of the symmetry
  // line we're on, unlike a sign-of-(dx) shortcut.
  const float perpx = -dy * (h / d);
  const float perpy =  dx * (h / d);
  const float ey_plus  = fy + perpy;
  const float ey_minus = fy - perpy;

  if (ey_plus >= ey_minus) { x = fx + perpx; y = ey_plus;  }
  else                     { x = fx - perpx; y = ey_minus; }
  return true;
}

// --- Pin joint E -> reported workspace coordinate (end-effector tip) ---------
// Apply the fixed +y end-effector offset and re-center on the workspace origin.
// Pure + host-tested. The caller keeps E (not this) for jacobianTranspose.
inline void reportPointFromE(float ex, float ey, float& rx, float& ry) {
  rx = ex;
  ry = (ey + kEndEffectorMm) - kReportOriginYMm;  // tip, re-centered
}

// --- Reported workspace coordinate -> pin joint E (exact inverse) -----------
// Undo reportPointFromE so the CPU side can run inverse kinematics on the
// reported tip. Paired here so the frame constants (kEndEffectorMm,
// kReportOriginYMm) live in exactly one place; reportPointFromE o invReportPoint
// is the identity (host-tested). The of-app debug overlay uses this to recover
// E before invKin.
inline void invReportPoint(float rx, float ry, float& ex, float& ey) {
  ex = rx;
  ey = (ry + kReportOriginYMm) - kEndEffectorMm;  // un-recenter, strip tip offset
}

// --- Jacobian transpose: cartesian force -> motor torques --------------------
//
// Derivation (closed kinematic chain, two circle constraints):
//   r1 = E - O2,  a1 = dO2/dtheta1 = L1 * (-sin t1, cos t1)
//   r2 = E - O4,  a2 = dO4/dtheta2 = L4 * (-sin t2, cos t2)
//   M  = [r1^T; r2^T]   (2x2)
//   J  = M^{-1} * diag(r1 . a1,  r2 . a2)
//   tau = J^T * F
//
// det(M) = r1x*r2y - r1y*r2x (a 2D cross product). It vanishes when the
// distal links are collinear — the workspace boundary singularity. We clamp
// to zero torque inside |det| < kSingularityDet and return false so the
// outer loop can fade force to zero rather than driving the motors blindly.

inline bool jacobianTranspose(float theta1, float theta2,
                              float ex, float ey,
                              float fx, float fy,
                              float& tau1, float& tau2) {
  const float o1x = -kBaseMm * 0.5f;
  const float o5x = +kBaseMm * 0.5f;
  const float c1 = trig::cosFast(theta1);
  const float s1 = trig::sinFast(theta1);
  const float c2 = trig::cosFast(theta2);
  const float s2 = trig::sinFast(theta2);

  const float o2x = o1x + kL1Mm * c1;
  const float o2y =        kL1Mm * s1;
  const float o4x = o5x + kL4Mm * c2;
  const float o4y =        kL4Mm * s2;

  const float r1x = ex - o2x, r1y = ey - o2y;
  const float r2x = ex - o4x, r2y = ey - o4y;

  const float r1_dot_a1 = kL1Mm * (-r1x * s1 + r1y * c1);
  const float r2_dot_a2 = kL4Mm * (-r2x * s2 + r2y * c2);

  const float det = r1x * r2y - r1y * r2x;
  if (det > -kSingularityDet && det < kSingularityDet) {
    tau1 = 0.0f;
    tau2 = 0.0f;
    return false;
  }
  const float inv = 1.0f / det;

  tau1 = r1_dot_a1 * inv * ( r2y * fx - r2x * fy);
  tau2 = r2_dot_a2 * inv * (-r1y * fx + r1x * fy);
  return true;
}

// --- Inverse kinematics: reported E -> motor angles -------------------------
//
// The of-app debug overlay needs theta1, theta2 from the live tip, but the wire
// only carries the FK-derived position. invKin is the inverse of fwdKin in the
// E-frame. It is host/of-app only — never on the AVR 1 kHz path — but compiles
// for AVR (unused inline -> not emitted).
//
// Two circle-circle intersections, one per side:
//   O2 in circle(O1,L1) cap circle(E,L2)   (two roots)
//   O4 in circle(O5,L4) cap circle(E,L3)   (two roots)
// That is up to 4 discrete (theta1,theta2) assemblies. ASSEMBLY-MODE INVARIANT
// (the branch decision a position-only round-trip would miss): the physical
// left elbow sits on the CCW side of the O1->E ray, the right elbow on the CW
// side of O5->E (mirror pair). The two circle roots are reflections across that
// ray, so they carry opposite cross signs; pick the sign per side. This flips
// only across a singularity (cross == 0, the workspace boundary), so it is
// stable. Verified by the angle round-trip + centerline known-answer host tests.

// Two-circle intersection. Writes both roots (a*, b*); false if the circles do
// not intersect (out of reach / one inside the other). Clamps a tiny negative
// h^2 (tangent case) to zero so a grazing solution still returns one point.
inline bool circleCircle(float c0x, float c0y, float r0,
                         float c1x, float c1y, float r1,
                         float& ax, float& ay, float& bx, float& by) {
  const float dx = c1x - c0x, dy = c1y - c0y;
  const float d2 = dx * dx + dy * dy;
  if (d2 <= 0.0f) return false;                       // concentric
  const float d = math::nrSqrt(d2);
  const float rsum = r0 + r1;
  const float rdif = (r0 > r1) ? (r0 - r1) : (r1 - r0);
  if (d > rsum || d < rdif) return false;             // no intersection
  const float a = (r0 * r0 - r1 * r1 + d2) / (2.0f * d);
  float h2 = r0 * r0 - a * a;
  if (h2 < 0.0f) h2 = 0.0f;                            // tangent: single point
  const float h = math::nrSqrt(h2);
  const float fx = c0x + a * dx / d, fy = c0y + a * dy / d;   // foot of perp
  const float px = -dy * (h / d),    py = dx * (h / d);       // +/- perp offset
  ax = fx + px; ay = fy + py;
  bx = fx - px; by = fy - py;
  return true;
}

inline bool invKin(float ex, float ey, float& theta1, float& theta2) {
  const float o1x = -kBaseMm * 0.5f;
  const float o5x = +kBaseMm * 0.5f;

  float l0x, l0y, l1x, l1y;   // two O2 candidates
  if (!circleCircle(o1x, 0.0f, kL1Mm, ex, ey, kL2Mm, l0x, l0y, l1x, l1y))
    return false;
  float r0x, r0y, r1x, r1y;   // two O4 candidates
  if (!circleCircle(o5x, 0.0f, kL4Mm, ex, ey, kL3Mm, r0x, r0y, r1x, r1y))
    return false;

  // ASSEMBLY-MODE INVARIANT (mirrored signed cross product). Over theta in
  // (0,pi)^2 fwdKin is many-to-one, so "matches E" is NOT enough — it admits a
  // flipped-elbow pose. The physical mode is: left elbow on the CCW side of the
  // O1->E ray, right elbow on the CW side of O5->E. The two circle roots are
  // reflections across that ray, so they carry opposite cross signs; pick the
  // sign that matches the physical assembly. This flips only across a
  // singularity (cross == 0, the workspace boundary), so it is stable.
  //   crossL = (E-O1) x (O2-O1),  O1 on the y=0 axis -> = (ex-o1x)*o2y - ey*(o2x-o1x)
  const float clA = (ex - o1x) * l0y - ey * (l0x - o1x);
  const float o2x = (clA > 0.0f) ? l0x : l1x;
  const float o2y = (clA > 0.0f) ? l0y : l1y;
  const float crA = (ex - o5x) * r0y - ey * (r0x - o5x);
  const float o4x = (crA < 0.0f) ? r0x : r1x;
  const float o4y = (crA < 0.0f) ? r0y : r1y;

  const float t1 = math::atan2safe(o2y, o2x - o1x);
  const float t2 = math::atan2safe(o4y, o4x - o5x);
  if (t1 <= 0.0f || t1 >= trig::kPi || t2 <= 0.0f || t2 >= trig::kPi) return false;

  // Boundary sanity: the picked assembly must actually reproduce E. 4 mm^2
  // absorbs LUT + nrSqrt round-off; a genuinely unreachable/degenerate pick fails.
  float vx, vy;
  if (!fwdKin(t1, t2, vx, vy)) return false;
  if ((vx - ex) * (vx - ex) + (vy - ey) * (vy - ey) > 4.0f) return false;

  theta1 = t1;
  theta2 = t2;
  return true;
}

}  // namespace welding

#endif  // WELDING_KINEMATICS_H
