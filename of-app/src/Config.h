#pragma once

// Single source of truth for CPU-side magic numbers. Anything tunable that
// is not exposed on the runtime UI lives here. Wire-format constants live in
// arduino/welding_common/protocol.h instead.
//
// Convention: every constant is `constexpr`, in welding::config namespace,
// and named with a unit suffix so misuse at the call site is obvious.

#include <cstddef>
#include <cstdint>

namespace welding {
namespace config {

// --- Workspace bounds (mm), in the REPORTED end-effector-tip frame ----------
// Shared by the renderer's workspace->screen transform and the cursor->mm
// mapping in ofApp (MouseSource). These define the of-app coordinate frame, and
// it MUST equal the frame the firmware reports on the wire: the torch tip
// re-centered on the centerline-pose origin (kReportOriginYMm in kinematics.h),
// i.e. reported = (E + kEndEffectorMm) - kReportOriginYMm. Both --source=serial
// (firmware) and --source=mouse (cursor->mm below) now live in THIS frame, so
// they agree on the rig.
//
// Values are the measured five-bar's reachable region in that reported frame,
// computed by sweeping welding::fwdKin over theta1,theta5 in (0,pi) and applying
// reportPointFromE (152.4 mm links, 87.0 mm base): the tip bounding box
//   rx in [-248.2, 248.2],  ry in [-228.6, 3.2]   (fits the q15 +/-256 range).
// Rounded outward by <1 mm for clean numbers. The centerline tip sits at ry=0
// (top of the band); the workspace extends downward (negative ry). This is NOT
// the old placeholder (+/-120, 0..180) — that was a guess in a different
// frame. test_kinematics.cpp pins these against the live fwdKin sweep so they
// cannot silently drift if the link dims change.
// (The base was re-measured 4.49 in -> 8.7 cm = 87.0 mm; the closer motor
// pivots widen and heighten the reachable region further, hence the larger
// bounds vs the earlier values +/-233 / [-219,6]. Still inside q15 +/-256.)
// ⚠ These kWorkspace* are the TRAINER frame ONLY (Renderer.cpp). The 4-bar
// linkage sim (--mode=linkage) is a DIFFERENT mechanism in a DIFFERENT frame —
// it uses kLinkageWorkspace* below. Do not merge the two; they were briefly
// shared early on and the sim happened to fit the old placeholder bounds.
inline constexpr float kWorkspaceMinX = -249.0f;
inline constexpr float kWorkspaceMaxX =  249.0f;
inline constexpr float kWorkspaceMinY = -229.0f;
inline constexpr float kWorkspaceMaxY =    4.0f;

// --- Linkage-sim workspace bounds (mm) --------------------------------------
// SEPARATE frame for the standalone 4-bar linkage simulator (--mode=linkage),
// which is an unrelated mechanism with its own geometry
// (kLinkA_mm.. below, joints around O2/O4 in y∈[40,166]). Kept at the original
// [-120,120]×[0,180] frame the sim's link lengths were verified against; the
// trainer's reported-tip reframing does NOT apply here. Consumed by
// LinkageRenderer's WorkspaceTransform and pinned by test_fourbar.
inline constexpr float kLinkageWorkspaceMinX = -120.0f;
inline constexpr float kLinkageWorkspaceMaxX =  120.0f;
inline constexpr float kLinkageWorkspaceMinY =    0.0f;
inline constexpr float kLinkageWorkspaceMaxY =  180.0f;

// --- Control rates (Hz) -----------------------------------------------------
inline constexpr double kRxHz          = 1000.0;  // RX drain + LP filter
inline constexpr double kOuterLoopHz   =  200.0;  // controller -> CMD_DOWN
inline constexpr double kHeartbeatHz   =   50.0;  // CPU liveness signal
inline constexpr double kRenderHz      =   60.0;  // not enforced by ControlThreads

// --- Low-pass filter --------------------------------------------------------
// One-pole IIR: y_n = alpha * x_n + (1 - alpha) * y_{n-1}.
//
// RATE-DEPENDENT — read before changing: the cutoff of a one-pole IIR is
// f_c ≈ f_s·α / (2π(1−α)), so α is only correct for the rate the filter is
// actually clocked at. The filter lives in ControlThreads::rxTick, pumped from
// ofApp::update() at the *render* rate (~60 Hz) because ControlThreads::start()
// is bypassed (ofApp.cpp:10-14) — NOT the 1 kHz the "kRxHz" constant implies.
//   α = 0.6 gives f_c ≈ 16 Hz AT 60 Hz   (0.6/(2π·0.4)·60 ≈ 14–16 Hz).
// Bench note: STATE_UP delivery to the of-app was MEASURED at ~60 Hz on the
// serial rig (frames/pump = 1 — loop() is paced by the I2C exchange, not the
// 1 kHz ISR), so this 60 Hz / 16 Hz figure is the true rate. Velocity noise is a
// SOURCE problem (a 1 ms-window finite diff sampled at ~60 Hz), so it is fixed in
// the firmware ISR (inner::VelocityEstimator), NOT by retuning α here.
// ⚠ At the original α=0.1 the 60 Hz pump gave τ≈150 ms (sluggish/unstable force);
//   α=0.6 restored ~16 Hz. If the reserved 1 kHz threaded path (start()) is ever
//   enabled, REVERT α to 0.1 — at 1 kHz α=0.6 barely filters (τ≈1 ms).
inline constexpr float kLpAlpha        = 0.6f;  // tuned for the ~60 Hz pump

// --- Reset / shape-switch force-feedback GAP -------------------------------
// After a reset or a shape switch, guidance force is held OFF until the handle
// comes within this radius of the active shape's start point (the trainee moves
// back across the gap, or Enter re-arms manually). Sized a bit larger than the
// bead width so re-arming feels forgiving but still "at the start."
inline constexpr float kStartCaptureRadiusMm = 12.0f;
// Re-tare re-home: "Re-tare to start" assigns the operator's current
// physical max-reach (pi/2) pose the meaning "20 mm above the active shape's
// start," then drops into the gap so the trainee descends onto the start. Also
// SEEDS the boot/default convention (centerline origin -> 20 mm above start) so a
// fresh boot already matches the tared layout. Must stay > kStartCaptureRadiusMm
// so the re-home lands OUTSIDE the capture zone and reset mode actually holds
// (else it would re-arm instantly).
inline constexpr float kTareAboveStartMm = 20.0f;
// Arm-in ramp: when guidance RE-engages (handle reaches the start, or
// Enter, or completion auto-reset, or E-stop release), force + ERM ease linearly
// from 0 to full over this window instead of stepping on. Pairs with the gap's
// active zero: silent approach → smooth ramp-in, no jolt at the start.
// 0 disables the ramp (instant-on). 0.5 s reads as a deliberate "grab" without
// feeling sluggish; shorter than the burn haptic's ~2 s in-ramp by design.
inline constexpr float kArmRampS = 0.5f;
// When the handle reaches within this radius of an OPEN path's end, the weld is
// "complete": guidance force releases (so the controller never shoves the handle
// off the end) and the completion marker shows. Closed paths (circle) have no end
// and never trigger this. Reset / switch / Enter clears the completed state.
inline constexpr float kEndReachedRadiusMm = 10.0f;
// A path is CLOSED (a loop, no endpoint) when its first and last vertices are
// within this distance (e.g. the full circle); closed paths skip end-release.
inline constexpr float kClosedLoopEpsMm = 1.0f;
// On completing an open weld, the green completion hoop pulses this many times,
// each shrinking over this period, then the trial AUTO-RESETS (trail cleared,
// back to RETURN-TO-START for the next rep) — no manual R needed.
inline constexpr float kCompletionHoopPeriodS = 0.5f;
inline constexpr int   kCompletionHoopCount   = 3;

// --- Outer-loop gains (defaults; UI sliders override at runtime) -----------
inline constexpr float kKpPerpDefault  = 0.30f;  // N / mm
inline constexpr float kKdPerpDefault  = 0.01f;  // N / (mm/s)
inline constexpr float kKpTanDefault   = 0.05f;  // N / (mm/s)
inline constexpr float kTargetSpeedMms = 20.0f;  // mm/s — tunable per path

// --- ERM signal shaping ------------------------------------------------------
// The ERM cue is carrier + a signed blow term, clamped to [0, kErmSafeCeilPwm]:
//   carrier = base hum + monotonic speed cue (always present)
//   blow    = a triggered one-shot envelope (signed; can be NEGATIVE)
// then pwm = clamp(carrier + blow, 0, ceil).
//
// WHY a NONZERO base above the floor: the ERM is unipolar (one H-bridge
// channel, DIR held LOW) so PWM is 0..255 with no inward "pull." To fake a
// bipolar "suction" on the blow-through, the carrier is biased to a base hum
// ABOVE the cogging floor; the blow envelope can then SUBTRACT below it (the
// dip drives the total to 0 — max buzz → sudden silence reads as a release/snap,
// the closest an eccentric mass can get to suction). Base sits below the old 28
// (quieter idle) but above the 20 floor so there is downward headroom.
//
// WHY the band [20, 70]: the ERM
// won't spin below the cogging floor and overheats above the thermal cap. The
// floor is a SUSTAINED-spin constraint on the carrier only — the blow dip may
// briefly hit 0 (a ~0.1 s transient; the mass just coasts). PER-COIL — confirm
// the 20 floor still reliably spins the mass on the bench; 20/255
// is ~8% duty, below the old 28 (~11%). The firmware passes pwm straight to
// analogWrite(D6) through the H-bridge; there is NO firmware clamp.
inline constexpr float kErmSafeFloorPwm = 20.0f;  // min SUSTAINED spin (carrier floor); dip may pass below
inline constexpr float kErmSafeCeilPwm  = 70.0f;  // thermal cap

inline constexpr float kErmBaseHum     =  24.0f;  // always-on hum; below old 28 (quieter) but above floor 20 (subtract room)

// Monotonic speed cue (supersedes the old detent bell). The buzz DECREASES
// as you move faster: max at standstill, fading to 0 at/above kErmSpeedCueSpan.
// WHY (preferred in bench testing): dwelling/too-slow is the dangerous failure
// (it burns through), so the cue nags when slow and quiets as you traverse —
// rather than rewarding a target speed with a peak (the old bell). The cue then
// hands off smoothly to the blow-through envelope when a dwell starts to burn.
inline constexpr float kErmSpeedCuePeak = 20.0f;  // PWM added at standstill (base+cue = 44 at v=0)
inline constexpr float kErmSpeedCueSpan = 40.0f;  // mm/s at which the speed cue fades to 0

// Blow-through (over-penetration) cue. Keyed to
// the HeatModel's burn proximity (0..1) under the torch — the exposure-dose
// distance to a blow-through, the SAME signal that draws the visible burn.
// Off-path error is corrected by force feedback alone; the ERM warns only
// of the heat defect (dwell/too-slow → burning a hole).
//
// The cue is no longer a static function of proximity — it is a TIME envelope:
//   approach  : while proximity rises thresh→sat, blow ramps 0→peak (proximity-driven)
//   hold      : at the burn (proximity≥sat) latch the one-shot; hold peak for kErmBlowHoldS
//   dip       : rapidly fall to kErmBlowDip (negative) over kErmBlowDipS — the "suction"
//   recover   : ramp dip→0 over kErmBlowRecoverS (total returns to the carrier)
//   refractory: suppress any further blow for kErmBlowRefractoryS after the latch
// Peak rides the ceiling (base+peak ≥ ceil); the dip is negative enough to drive
// the total to 0 (silence) regardless of the carrier. Significance comes from the
// peak→silence excursion (~70 units), not from raising the thermal cap.
inline constexpr float kErmBlowThresh      =   0.50f; // burn proximity (0..1) — below this, no blow cue
inline constexpr float kErmBlowSat         =   1.00f; // burn proximity — 1.0 = burning; latches the one-shot
inline constexpr float kErmBlowPeak        =  46.0f;  // base+peak = 70 (ceiling); rides the cap at the burn
inline constexpr float kErmBlowDip         = -70.0f;  // suction trough: drives total to 0 (silence) under clamp
inline constexpr float kErmBlowHoldS       =   0.20f; // s — peak hold at the burn
inline constexpr float kErmBlowDipS        =   0.1333f; // s — rapid drop to the dip (suction); 0.40/3, 3x steeper
inline constexpr float kErmBlowRecoverS    =   0.40f; // s — ramp dip→0 back to the carrier
inline constexpr float kErmBlowRefractoryS =   1.75f; // s — lockout: no new blow effect within this window; ALSO arms the visual burnout lockout in HeatModel so the two stay synced

// --- Renderer heat model (heat-affected-zone deposition) --------------------
// Each frame the arc deposits a constant energy kArcPower*dt, spread over the
// bead vertices within kHeatRadiusMm of the torch by a triangular falloff and
// normalized by Σw; then all cells decay by kHeatDecayPerFrame. A vertex stays
// within R for ~2R/v, so total deposited heat ∝ 1/v (the moving-heat-source
// law) — recovering the physical fact that a fast pass under-fuses and a dwell
// over-penetrates. Replaces the old single-vertex dt/v deposit, which
// integrated to an accidental 1/v² and left moving welds permanently cold.
//
// Calibration (host-tested, "calibration property"): with the constants below
// a traverse at/under ~30 mm/s lands in the "good" band and faster passes (40,
// 80 mm/s) read under-fused, with the kFusionMinNorm = 0.135 peak split (cutoff
// moved to 30 mm/s — measured peak(30)≈0.136 — so "too fast" trips just
// above the 20 mm/s kTargetSpeedMms); a dwell
// over-penetrates via the exposure latch (kBurnExposure), no longer a peak
// band. R is 4 mm (not the larger zone first sketched): the HAZ sizing kept the
// dwell-vs-traverse peak separation that the old normalized-peak burn threshold
// (0.85) needed; R≈8 was too flat (~3.6), R=4 gives ~7.5. The per-frame decay is
// frame-rate dependent; a time-based decay is the proper fix (TODO).
inline constexpr float kHeatRadiusMm      = 4.0f;    // mm — heat-affected-zone radius R
inline constexpr float kArcPower          = 0.63f;   // per-second arc energy spread over the HAZ
inline constexpr float kHeatDecayPerFrame = 0.98f;   // per-frame conductive cooling
inline constexpr float kHeatMax           = 0.30f;   // heat mapped to ramp top (supersedes the old 0.13)
// Spatial conduction (default off, 'C' toggles): explicit 1-D Laplacian
// mixing coefficient. Must be < 0.5 for stability; 0.15 visibly evens a heat
// spike toward its neighbors each frame without overshoot.
inline constexpr float kConduction        = 0.15f;

// --- Blow-through (over-penetration) designator -----------------------------
// Blow-through latch on the un-normalized exposure dose. On a dwell the
// torch sits on one vertex (falloff weight w≈1), so the dose accrues at
// ~kArcPower/s = 0.63/s; the latch therefore fires after ~kBurnExposure/0.63 s.
// 1.2 -> ~1.9 s on-vertex (~2 s with the slight off-vertex w<1), per operator
// "roughly around 2 s" (was 0.45 -> ~0.7 s). A target-speed traverse never
// dwells long enough to latch. INDEPENDENT of vertex density/path shape/conduction
// (which the old normalized-peak latch was not). Supersedes kBurnThroughNorm.
// NOTE: also stretches the ERM blow-through warning, which is keyed to the same
// burn proximity (exposure/kBurnExposure) — the haptic ramps in over ~2 s too.
inline constexpr float kBurnExposure      = 1.2f;
inline constexpr float kBurnHoleRadiusPx  = 15.0f;   // dark hole radius (live QA: 6->15, 2.5x larger marker)
inline constexpr float kBurnRimRadiusPx   = 30.0f;   // hot rim outer radius (live QA: 12->30, keeps 1:2 hole:rim)

// --- Weld-quality view (peak-heat classification) ---------------------------
// Per-segment fusion quality from latched PEAK normalized heat. Below
// kFusionMinNorm = under-fused; at/above = good fusion; over-penetration is
// flagged separately by the exposure latch (kBurnExposure), not a peak band.
// Set so the "too fast" cutoff lands just above 30 mm/s: measured
// peak(30)≈0.136, peak(35)≈0.130, so 0.135 makes 30 the last Good speed and any
// faster pass under-fused. The earlier 0.10 put the cutoff near 52 mm/s,
// far above the 20 mm/s target. The HAZ re-tune made moving welds reach the band.
inline constexpr float kFusionMinNorm     = 0.135f;  // below = under-fused (cutoff ≈ 30 mm/s)
// Peak normalized heat of an IDEAL (correct-speed) weld — the "perfect green"
// point of the quality-view gradient. The bead's green is brightest here
// and dims as peak heat runs toward under-fused (kFusionMinNorm, too fast/thin)
// OR toward over-penetration (too slow/deep), so the shade now encodes HOW close
// the trainee's speed was, instead of a flat pass/fail green. Set to the MEASURED
// peak a kTargetSpeedMms (20 mm/s) traverse reaches so the brightest green lands
// exactly at the prescribed speed; gradient half-width is (ideal-min)=0.072.
// VISUAL/BENCH-TUNABLE: nudge to match where a correct pass actually peaks.
inline constexpr float kFusionIdealNorm   = 0.207f;

// --- Spark particle system --------------------------------------------------
inline constexpr int   kMaxSparks        = 4000;     // hard cap (pre-allocated)
inline constexpr float kSparkGravityMms2 = 600.0f;   // +y accel in sim frame
inline constexpr float kSparkDrag        = 0.92f;    // per-frame velocity retention
inline constexpr float kSparkLifeS       = 0.85f;    // seconds before a spark dies
inline constexpr float kSparkSpeedScale  = 0.55f;    // ejection speed per source mm/s

// --- Audio engine -----------------------------------------------------------
// Crackle rate scales with travel speed (the audible speed cue); arc-hiss gain
// crossfades between idle and active. All values are tunable at mixdown time.
inline constexpr float kCrackleMinHz       = 4.0f;    // idle crackle trigger rate
inline constexpr float kCrackleMaxHz       = 45.0f;   // rate at/above the speed cap
inline constexpr float kCrackleSpeedCapMms = 80.0f;   // speed mapped to max rate
inline constexpr float kCracklePitchSpread = 0.40f;   // +/- playback-rate jitter range
inline constexpr float kHissGainAlpha      = 0.08f;   // one-pole smoothing for hiss gain
inline constexpr float kHissIdleGain       = 0.15f;   // hiss gain when stopped
inline constexpr float kHissActiveGain     = 0.85f;   // hiss gain when moving
inline constexpr float kHissActiveSpeedMms = 10.0f;   // speed above which arc is "active"
// Overall scale on the WELDING voices (arc hiss + crackle) only, applied at
// playback. The completion chime is a separate success cue and is NOT scaled.
// Set to 0.30 to bring the weld bed down to 30% of its prior level (user pref).
inline constexpr float kWeldVolumeScale    = 0.30f;

// --- SessionLog -------------------------------------------------------------
inline constexpr std::size_t kLogCapacityRows = 60'000;  // 10 min at 100 Hz
inline constexpr int         kLogDecimateN    = 10;      // 1 kHz -> 100 Hz

// --- CMD_DOWN payload limits -----------------------------------------------
// Outer loop force-magnitude clamp before handing to the sink. The wire layer
// also clamps when converting to q15, but doing it here keeps the diagnostic
// trail (test asserts) clean.
inline constexpr float kMaxForceN      = 5.0f;

// --- 4-bar linkage simulator ------------------------------------------------
// Default link lengths (mm). Verified to keep O2, O4, A, B, and coupler
// point P within the LINKAGE workspace bounds
// (kLinkageWorkspaceMinX=-120..MaxX=120, kLinkageWorkspaceMinY=0..MaxY=180)
// across the full crank rotation. Verified bounds (3600-sample sweep):
//   all-joint x in [-100, 60], y in [40, 166], P y_max ~166 with h=30.
// Margins from workspace edge: left 20, right 60, bottom 40, top ~14.
//
// Topology: O2=(-60,80), O4=(60,80); O4=O2+(d,0).
//   a = input  crank  O2->A = 40 mm
//   b = coupler bar   A->B  = 100 mm
//   c = output crank  O4->B = 80 mm
//   d = ground link   O2->O4 = 120 mm
//
// Grashof check: min+max < sum of other two => 40+120=160 < 100+80=180 ✓
// This is a crank-rocker (shortest link = a = crank -> fully rotatable input).
inline constexpr float kLinkA_mm = 40.0f;    // input crank length
inline constexpr float kLinkB_mm = 100.0f;   // coupler length
inline constexpr float kLinkC_mm = 80.0f;    // output crank length
inline constexpr float kLinkD_mm = 120.0f;   // ground link length

// Input ground pivot position in workspace mm (O2). O4 = O2 + (d, 0).
// O2_Y raised to 80 (workspace vertical midpoint) so B0 stays above y=0.
inline constexpr float kO2_X_mm = -60.0f;
inline constexpr float kO2_Y_mm =  80.0f;

// Coupler point defaults — point P rigidly attached to the coupler bar A->B.
//   s = fraction along A->B (0=A, 0.5=midpoint, 1=B)
//   h = perpendicular offset in mm (positive = left of A->B direction)
inline constexpr float kCouplerPointSDefault = 0.5f;   // midpoint of coupler
inline constexpr float kCouplerPointHDefaultMm = 30.0f; // 30 mm offset

// Assembly branch: +1 selects b0 on the first feasible frame (pickSeededRoot).
inline constexpr int kAssemblyBranchSign = +1;

// CouplerTrace ring buffer capacity (number of Vec2 points).
inline constexpr std::size_t kCouplerTraceMaxPoints = 2000;

// --- Linkage simulator slider ranges -----------------------------------------
// Number of theta samples used by feasibleThetaInterval in ofApp::update().
inline constexpr int kFeasibleThetaSamples = 360;

// Link-length slider range (mm). Wide enough to explore the design space while
// keeping the mechanism within workspace bounds for values near the defaults.
inline constexpr float kLinkMinMm = 10.0f;
inline constexpr float kLinkMaxMm = 200.0f;

// Coupler-point h (perpendicular offset) slider range (mm).
// ±80 mm keeps P within workspace across a wide sweep of configurations.
inline constexpr float kCouplerHMinMm = -80.0f;
inline constexpr float kCouplerHMaxMm =  80.0f;

// Theta slider range [0, 2π].
inline constexpr float kThetaMin = 0.0f;
inline constexpr float kThetaMax = 6.2831853f;  // 2*pi; literal to keep Config.h oF-free
inline constexpr float kThetaDefaultRad = 1.0471976f;  // initial crank angle; mid-range pose, avoids the toggle near 0

// Linkage rendering constants.
// Joint radius and link line width are in screen pixels.
inline constexpr float kJointRadiusPx    = 6.0f;    // filled circle at each joint
inline constexpr float kLinkLineWidthPx  = 2.5f;    // link bar line width
inline constexpr float kCouplerPtRadiusPx = 5.0f;   // marker at coupler point P
inline constexpr float kTraceLineWidthPx = 1.0f;    // coupler trace polyline width

// Colors as plain RGB ints [0..255] (Config.h is oF-free; no ofColor here).
// Use these to construct ofColor in LinkageRenderer.cpp.
struct RgbColor { int r, g, b; };

inline constexpr RgbColor kColorGround      = { 80,  90, 100};  // ground link O2->O4
inline constexpr RgbColor kColorInputCrank  = {220, 120,  30};  // input crank  O2->A
inline constexpr RgbColor kColorCoupler     = {180, 200,  80};  // coupler bar  A->B
inline constexpr RgbColor kColorOutputCrank = { 80, 160, 220};  // output crank B->O4
inline constexpr RgbColor kColorJoint       = {240, 240, 240};  // joint circles
inline constexpr RgbColor kColorCouplerPt   = {255,  60, 200};  // coupler point P
inline constexpr RgbColor kColorTrace       = {200,  60, 200};  // trace polyline

}  // namespace config
}  // namespace welding
