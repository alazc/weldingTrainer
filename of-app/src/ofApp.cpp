#include "ofApp.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "Config.h"
#include "FourBar.h"

// =============================================================================
// Threading model for v1: the control loop is driven SINGLE-
// THREADED from update(). ControlThreads::start() is NOT called. Reason:
// MouseSource::inject is main-thread-only and not thread-safe against a real
// RX thread, and with no Arduino the 3-thread live path buys nothing. The live
// path stays reserved for serial+hardware bringup.
// =============================================================================

namespace {
using namespace welding;

constexpr const char* kLogTag      = "ofApp";
constexpr const char* kSessionCsv  = "session.csv";

// Build OuterGains from the current slider values.
control::OuterGains gainsFromSliders(float kp_perp, float kd_perp,
                                     float kp_tan, float target,
                                     bool erm_enabled) {
  control::OuterGains g;
  g.kp_perp     = kp_perp;
  g.kd_perp     = kd_perp;
  g.kp_tan      = kp_tan;
  g.target_mms  = target;
  g.max_force_N = config::kMaxForceN;
  g.erm_enabled = erm_enabled;
  return g;
}
}  // namespace

//--------------------------------------------------------------
void ofApp::setup() {
  // --- High-DPI scaling (applies to BOTH modes; runs before any gui setup) --
  // The window is created at a fixed 1024x768; on a high-DPI display that is a
  // tiny fraction of the screen and oF's 8px bitmap font does not scale. Derive
  // a UI scale from the screen height, size the window to ~2/3 of the screen,
  // load a scaled TTF for the HUD, and scale the ofxGui panels (font + cell
  // size, which keeps mouse hit-testing correct). The bead/linkage geometry is
  // NOT scaled here — it already fits the window via the letterbox transform.
  {
    const int sw = ofGetScreenWidth();
    const int sh = ofGetScreenHeight();
    ui_scale_ = std::max(1.0f, std::min(4.0f, sh / 770.0f));
    const int ww = static_cast<int>(std::lround(sw * 0.85));
    const int wh = static_cast<int>(std::lround(sh * 0.85));
    ofSetWindowShape(ww, wh);
    ofSetWindowPosition((sw - ww) / 2, (sh - wh) / 2);

    hud_font_.load("fonts/hud.ttf",
                   static_cast<int>(std::lround(13 * ui_scale_)), true, true);

    // ofxGui global config — MUST precede every panel .setup() below.
    ofxGuiSetFont("fonts/hud.ttf", static_cast<int>(std::lround(9 * ui_scale_)));
    ofxGuiSetDefaultWidth(static_cast<int>(std::lround(240 * ui_scale_)));
    ofxGuiSetDefaultHeight(static_cast<int>(std::lround(18 * ui_scale_)));
  }

  // --- Linkage simulator mode -----------------------------------------------
  linkage_mode_ = (mode_spec_ == "linkage");
  if (linkage_mode_) {
    ofSetWindowTitle("Welding Sim: 4-Bar Linkage");
    ofSetVerticalSync(true);
    ofSetFrameRate(60);
    ofBackground(10, 12, 20);

    // Sliders seeded from Config defaults, ranges from Config range constants.
    lk_theta_.set("Theta (rad)",  config::kThetaDefaultRad,
                  config::kThetaMin, config::kThetaMax);
    lk_a_.set("Link A (mm)",      config::kLinkA_mm,
              config::kLinkMinMm, config::kLinkMaxMm);
    lk_b_.set("Link B (mm)",      config::kLinkB_mm,
              config::kLinkMinMm, config::kLinkMaxMm);
    lk_c_.set("Link C (mm)",      config::kLinkC_mm,
              config::kLinkMinMm, config::kLinkMaxMm);
    lk_d_.set("Link D (mm)",      config::kLinkD_mm,
              config::kLinkMinMm, config::kLinkMaxMm);
    lk_s_.set("Coupler s",        config::kCouplerPointSDefault,
              0.0f, 1.0f);
    lk_h_.set("Coupler h (mm)",   config::kCouplerPointHDefaultMm,
              config::kCouplerHMinMm, config::kCouplerHMaxMm);

    linkage_gui_.setup("Linkage");
    linkage_gui_.add(lk_theta_);
    linkage_gui_.add(lk_a_);
    linkage_gui_.add(lk_b_);
    linkage_gui_.add(lk_c_);
    linkage_gui_.add(lk_d_);
    linkage_gui_.add(lk_s_);
    linkage_gui_.add(lk_h_);

    linkage_renderer_.setup(ui_scale_);
    return;  // <-- do NOT fall through to trainer setup
  }

  // --- Trainer mode (existing, unchanged) -----------------------------------
  ofSetWindowTitle("Welding Trainer");
  ofSetVerticalSync(true);
  ofSetFrameRate(60);
  ofBackground(10, 12, 16);

  // --- HUD sliders, seeded from Config defaults -----------------------------
  kp_perp_.set("Kp_perp (N/mm)", config::kKpPerpDefault, 0.0f, 1.0f);
  kd_perp_.set("Kd_perp (N/mm/s)", config::kKdPerpDefault, 0.0f, 0.2f);
  kp_tan_.set("Kp_tan (N/mm/s)", config::kKpTanDefault, 0.0f, 0.5f);
  target_speed_.set("Target speed (mm/s)", config::kTargetSpeedMms, 0.0f, 120.0f);
  erm_enabled_.set("ERM haptics", true);
  gui_.setup("Gains");
  gui_.add(kp_perp_);
  gui_.add(kd_perp_);
  gui_.add(kp_tan_);
  gui_.add(target_speed_);
  gui_.add(erm_enabled_);
  gui_.add(retare_btn_.setup("Re-tare to start (T)"));   // re-home dot + reset
  retare_btn_.addListener(this, &ofApp::onRetarePressed);

  // Pantograph debug overlay ('D' toggle); shares the HUD font scale.
  pantograph_debug_.setup(ui_scale_);

  // --- Source + sink selection ----------------------------------------------
  // Default banner is overwritten below per resolved source.
  if (source_spec_.rfind("replay:", 0) == 0) {
    const std::string file = source_spec_.substr(std::string("replay:").size());
    auto replay = std::make_unique<input::LogReplaySource>();
    std::string err;
    if (replay->loadFromFile(file, &err)) {
      source_      = std::move(replay);
      source_kind_ = SourceKind::kReplay;
      banner_text_ = "DEMO: replay (" + file + ")";
    } else {
      ofLogError(kLogTag) << "replay load failed for '" << file << "': " << err
                          << " — falling back to mouse";
      auto m = std::make_unique<input::MouseSource>();
      mouse_       = m.get();
      source_      = std::move(m);
      source_kind_ = SourceKind::kMouse;
      banner_text_ = "DEMO: mouse (replay load failed)";
    }
    sink_ = std::make_unique<motor::NullSink>();

  } else if (source_spec_ == "serial" || source_spec_.rfind("serial:", 0) == 0) {
    // "serial"      -> first enumerated device (legacy). oF does NOT sort ports
    //                  on Windows (SetupDiEnumDeviceInfo order), so dev 0 is only
    //                  safe when a single adapter is present.
    // "serial:COM6" -> open that named port explicitly. Required once the
    //                  I2C secondary's USB is also plugged in (two FTDI ports),
    //                  since device 0 is then non-deterministic.
    std::string port;  // empty => legacy dev-0 path
    const auto colon = source_spec_.find(':');
    if (colon != std::string::npos) port = source_spec_.substr(colon + 1);
    const bool ok = port.empty() ? serial_.setup(0, 115200)
                                 : serial_.setup(port, 115200);
    if (ok) {
      source_      = std::make_unique<input::SerialSource>(serial_);
      sink_        = std::make_unique<motor::SerialSink>(serial_);
      source_kind_ = SourceKind::kSerial;
      banner_text_ = "SERIAL: connected (" +
                     (port.empty() ? std::string("dev 0") : port) + " @ 115200)";
    } else {
      ofLogWarning(kLogTag) << "serial setup(" << (port.empty() ? "0" : port)
                            << ",115200) failed — no input, NullSink";
      source_      = nullptr;  // controller handles a null source via rxTick poll
      sink_        = std::make_unique<motor::NullSink>();
      source_kind_ = SourceKind::kNone;
      banner_text_ = "SERIAL: no device";
    }

  } else {
    // Default: mouse.
    auto m = std::make_unique<input::MouseSource>();
    mouse_       = m.get();
    source_      = std::move(m);
    sink_        = std::make_unique<motor::NullSink>();
    source_kind_ = SourceKind::kMouse;
    banner_text_ = "DEMO: mouse";
  }

  // --- Session log (RAII flush on exit) -------------------------------------
  log_ = std::make_unique<log::SessionLog>(config::kLogCapacityRows,
                                           std::string(kSessionCsv));

  // --- Control stack (path wired below via setActivePath, the choke point) --
  const control::OuterGains g = gainsFromSliders(kp_perp_, kd_perp_,
                                                 kp_tan_, target_speed_,
                                                 erm_enabled_);
  ct_ = std::make_unique<control::ControlThreads>(source_.get(), sink_.get(),
                                                  nullptr, g);
  ct_->setSessionLog(log_.get());

  // --- Presentation ---------------------------------------------------------
  renderer_.setup();
  audio_.setup();  // graceful if false (silent run)

  // --- Shape library: cache the icon thumbnails, then load the default shape -
  // setActivePath() is the SINGLE place the active path is set: it points both
  // the controller (force) and the renderer (guide) at one Path. Boot enters the
  // reset/GAP state (await_start_) so the trainee starts at RETURN-TO-START with
  // guidance paused, just like after R / a shape switch — force arms once the
  // handle reaches the start marker (or on Enter), matching every other trial.
  buildThumbnails();
  setActivePath(catalog_.active().json_file, /*enter_gap=*/true);
}

//--------------------------------------------------------------
// setActivePath — the ONE choke point that swaps the active reference path.
// Points the controller (force) AND the renderer (guide) at the SAME Path so
// the line you feel always equals the line you see; re-primes the input filter
// (a switch teleports the handle); clears the prior trial's trail; and, when
// asked, drops into the force-off GAP until the handle returns to the start.
//--------------------------------------------------------------
void ofApp::setActivePath(const std::string& json_file, bool enter_gap) {
  const std::string full = ofToDataPath("paths/" + json_file, true);
  std::string err;
  auto p = path::PathLibrary::loadFromFile(full, &err);
  if (!p || p->empty()) {
    ofLogWarning(kLogTag) << "path load failed for '" << full << "': " << err
                          << " — keeping the previous path";
    return;  // never go null mid-session; keep whatever was active
  }
  path_ = std::move(p);
  ct_->setPath(path_.get());        // force
  renderer_.setPath(path_.get());   // visual guide — SAME object
  ct_->resetFilter();               // teleport → re-prime LP (no phantom spike)
  renderer_.reset();                // clear prior trail / heat / sparks
  assert(ct_->path() == renderer_.path());  // visual == force, by construction

  // Cache start + end vertices (polyline endpoints are exact). Start drives the
  // GAP capture; end drives the end-of-weld release. A path whose ends coincide
  // is a closed loop (the circle) with no endpoint to release at.
  const auto poly = path_->polyline(1);
  if (poly.empty()) {
    start_mm_ = end_mm_ = glm::vec2(0.0f, 0.0f);
  } else {
    start_mm_ = glm::vec2(poly.front().x, poly.front().y);
    end_mm_   = glm::vec2(poly.back().x,  poly.back().y);
  }
  is_closed_   = welding::ui::withinCapture(start_mm_.x, start_mm_.y,
                                            end_mm_.x, end_mm_.y,
                                            config::kClosedLoopEpsMm);
  reached_end_ = false;             // a fresh path is never already complete

  // Seed the re-home offset so a FRESH path already sits where 'T' would put it:
  // the firmware reports the centerline / pi-2 max-reach pose as
  // (0,0), and we want that pose to render 20 mm above this shape's start — the
  // same anchor retarePosition() uses. Since display = raw - tare and raw_ref =
  // (0,0), the seed is offset = -anchor. Deterministic (no live snapshot needed),
  // so it does not depend on where the handle is when of-app reads its first
  // frame; a manual 'T' afterwards corrects mid-trial sensor drift.
  if (ct_) ct_->setTare(-start_mm_.x,
                        -(start_mm_.y + config::kTareAboveStartMm));

  if (enter_gap) await_start_ = true;
}

//--------------------------------------------------------------
// buildThumbnails — load each catalog shape once and cache its polyline (mm)
// for the icon strip. The icon IS the shape's own path, so it can never drift
// from what the trainee feels/traces.
//--------------------------------------------------------------
void ofApp::buildThumbnails() {
  thumbs_.clear();
  thumbs_.reserve(catalog_.size());
  for (std::size_t i = 0; i < catalog_.size(); ++i) {
    const std::string full = ofToDataPath("paths/" + catalog_.at(i).json_file, true);
    std::string err;
    auto p = path::PathLibrary::loadFromFile(full, &err);
    std::vector<welding::ui::Pt> pts;
    if (p && !p->empty()) {
      for (const auto& v : p->polyline(48))
        pts.push_back(welding::ui::Pt{v.x, v.y});
    } else {
      ofLogWarning(kLogTag) << "thumbnail load failed for '" << full << "': " << err;
    }
    thumbs_.push_back(std::move(pts));
  }
}

//--------------------------------------------------------------
// layoutStrip — N shape cells + 1 reset cell, centered along the bottom, above
// the bottom HUD line. Shared by draw and the mouse hit-test so they agree.
//--------------------------------------------------------------
void ofApp::layoutStrip(std::vector<ofRectangle>& shape_cells,
                        ofRectangle& reset_cell) const {
  shape_cells.clear();
  const float cell = static_cast<float>(std::lround(54 * ui_scale_));
  const float gap  = static_cast<float>(std::lround(8  * ui_scale_));
  const std::size_t n = catalog_.size();
  const float total = (n + 1) * cell + n * gap;   // n shapes + reset, n gaps
  float x = (ofGetWidth() - total) * 0.5f;
  const float y = ofGetHeight() - cell - static_cast<float>(std::lround(36 * ui_scale_));
  for (std::size_t i = 0; i < n; ++i) {
    shape_cells.push_back(ofRectangle(x, y, cell, cell));
    x += cell + gap;
  }
  reset_cell = ofRectangle(x, y, cell, cell);
}

//--------------------------------------------------------------
void ofApp::drawShapeStrip() const {
  std::vector<ofRectangle> cells;
  ofRectangle reset_cell;
  layoutStrip(cells, reset_cell);

  ofPushStyle();
  for (std::size_t i = 0; i < cells.size() && i < thumbs_.size(); ++i) {
    const bool active = (i == catalog_.activeIndex());
    ofFill();
    ofSetColor(active ? ofColor(40, 70, 110) : ofColor(24, 28, 36));
    ofDrawRectangle(cells[i]);
    ofNoFill();
    ofSetColor(active ? ofColor(120, 180, 255) : ofColor(70, 80, 95));
    ofDrawRectangle(cells[i]);

    const welding::ui::Rect r{cells[i].x, cells[i].y, cells[i].width, cells[i].height};
    const auto fit = welding::ui::fitPolylineToRect(thumbs_[i], r,
                                                    10.0f * ui_scale_, /*flip_y=*/true);
    if (fit.size() >= 2) {
      ofSetColor(active ? ofColor(205, 228, 255) : ofColor(150, 170, 195));
      ofSetLineWidth(active ? 2.2f : 1.5f);
      ofPolyline pl;
      for (const auto& p : fit) pl.addVertex(p.x, p.y);
      pl.draw();
    }
  }

  // Reset cell (mirrors the [R] key).
  ofFill();
  ofSetColor(44, 24, 24);
  ofDrawRectangle(reset_cell);
  ofNoFill();
  ofSetColor(185, 95, 80);
  ofDrawRectangle(reset_cell);
  ofFill();
  ofSetColor(225, 150, 140);
  hud_font_.drawString("R",
      reset_cell.x + reset_cell.width * 0.5f - 4.0f * ui_scale_,
      reset_cell.y + reset_cell.height * 0.5f + 6.0f * ui_scale_);
  ofSetLineWidth(1.0f);
  ofPopStyle();
}

//--------------------------------------------------------------
bool ofApp::overStrip(float x, float y) const {
  std::vector<ofRectangle> cells;
  ofRectangle reset_cell;
  layoutStrip(cells, reset_cell);
  for (const auto& c : cells) if (c.inside(x, y)) return true;
  return reset_cell.inside(x, y);
}

//--------------------------------------------------------------
void ofApp::update() {
  // --- Linkage simulator mode -----------------------------------------------
  if (linkage_mode_) {
    const float a     = lk_a_.get();
    const float b     = lk_b_.get();
    const float c     = lk_c_.get();
    const float d     = lk_d_.get();
    float       theta = lk_theta_.get();

    // Clamp theta to the contiguous feasible interval that contains it.
    welding::linkage::ThetaInterval iv =
        welding::linkage::feasibleThetaInterval(
            a, b, c, d, theta, config::kFeasibleThetaSamples);
    if (iv.ok) {
      const float clamped = std::max(iv.lo, std::min(iv.hi, theta));
      if (clamped != theta) {
        theta = clamped;
        lk_theta_.set(theta);  // reflect the feasible-range limit in the slider
      }
    }

    const welding::linkage::Vec2 o2{config::kO2_X_mm, config::kO2_Y_mm};
    linkage_renderer_.update(theta, a, b, c, d, o2, lk_s_.get(), lk_h_.get());
    return;  // <-- do NOT fall through to trainer update
  }

  // --- Trainer mode (existing, unchanged) -----------------------------------
  const float t  = ofGetElapsedTimef();
  float       dt = ofGetLastFrameTime();
  if (dt < 1e-4f) dt = 1.0f / 60.0f;  // guard the first frame / paused clock

  // Mouse source: map the OS cursor to workspace mm via the shared transform.
  if (mouse_) {
    const float mx = static_cast<float>(ofGetMouseX());
    const float my = static_cast<float>(ofGetMouseY());
    const glm::vec2 mm = renderer_.screenToWorld(mx, my);
    mouse_->inject(t, mm.x, mm.y);

    // The cursor drives the torch, so hide it while it's over the weld area;
    // keep it visible over the gain panel AND over the shape-library strip
    // so both stay clickable. Toggle only on the boundary crossing.
    const bool over_ui = (show_gains_ && gui_.getShape().inside(mx, my)) || overStrip(mx, my);
    if (over_ui && cursor_hidden_) {
      ofShowCursor();
      cursor_hidden_ = false;
    } else if (!over_ui && !cursor_hidden_) {
      ofHideCursor();
      cursor_hidden_ = true;
    }
  }

  // Push current slider gains into the controller.
  ct_->setGains(gainsFromSliders(kp_perp_, kd_perp_, kp_tan_, target_speed_,
                                 erm_enabled_));

  // Drain input + publish snapshot. The frame count is a transport-rate
  // diagnostic: on the serial rig this reads ~1 (STATE_UP arrives at
  // ~60 Hz, loop()-paced by I2C), which is why velocity de-noising lives in the
  // firmware ISR, not here. Logged ~once a second.
  const std::size_t rx_frames = ct_->rxTick(t);
  if (ofGetFrameNum() % 60 == 0) {
    ofLogNotice("rxTick") << "frames/pump=" << rx_frames;
  }

  // Feed the burn proximity under the torch (exposure-dose distance to a
  // blow-through, the same signal as the visible burn) into the ERM blow-through
  // cue. Uses last frame's value (renderer_.update runs below); the
  // ~1-frame lag is immaterial for a slow-building "you're burning through"
  // warning. Off-path error is handled separately by the force loop.
  ct_->setOverpenetration(renderer_.currentBurnProximity());

  // Outer loop emits force only when armed: NOT E-stop-latched, NOT in the
  // reset/shape-switch GAP (await_start_, the "move back across the gap" state),
  // and NOT past the end of the weld (reached_end_, force released on completion).
  // Capture the commanded force AT this single gating site: the debug
  // overlay's torque must reflect what is actually applied, so when the loop is
  // NOT armed we zero it and record why, rather than re-inferring armed-ness
  // elsewhere (which could drift from this decision).
  const bool armed = !latched_ && !await_start_ && !reached_end_;
  if (armed) {
    // Arm-in ramp: the moment guidance re-engages out of the gap (or
    // E-stop release / completion auto-reset), ease force + ERM 0→full over
    // kArmRampS instead of stepping on. was_armed_ catches the not-armed→armed
    // edge and stamps arm_time_; the gain feeds outerLoopTick so the SCALED cmd
    // is what reaches the wire (and the captured last_cmd_ shows the ramp too).
    if (!was_armed_) arm_time_ = t;
    const float gain = welding::control::armRampGain(t - arm_time_, config::kArmRampS);
    last_cmd_ = ct_->outerLoopTick(t, gain);
    force_on_ = true;
  } else {
    // Actively command zero (force 0, ERM 0) — do NOT just skip. The firmware
    // latches its last CMD_DOWN and keeps applying it every inner-loop tick, so
    // skipping leaves residual force + ERM buzz through the RETURN-TO-START gap
    // (and E-stop / end-of-weld release). The explicit zero releases both.
    ct_->sendIdleCmd();
    last_cmd_ = welding::control::OuterCmd{};  // motors off -> zero force shown
    force_on_ = false;
    force_off_reason_ = latched_ ? "E-STOP" : (await_start_ ? "GAP" : "DONE");
  }
  was_armed_ = armed;  // edge for the arm-in ramp: re-stamp on next re-arm

  // CPU liveness heartbeat always ticks.
  ct_->heartbeatTick(t);

  // Read the latest coherent snapshot for render + audio.
  input::StateSnapshot s{};
  if (ct_->slot().read(s)) {
    last_snap_ = s;                 // for the 'D' debug overlay in draw()
    renderer_.update(s, dt, armed); // trail/heat/burns track only while armed
    last_speed_mms_ = std::hypot(s.vx, s.vy);
    if (!latched_) {
      // Welding voices ramp in/out with the arm state: silent through the
      // RETURN-TO-START gap + end-of-weld, faded in only while actually welding,
      // matching the force/ERM ramp. E-stop (latched_) still hard-stops.
      audio_.update(last_speed_mms_, dt, armed);
    }
    // GAP -> ARMED: re-engage guidance once the handle reaches the shape start.
    if (await_start_ &&
        welding::ui::withinCapture(s.x, s.y, start_mm_.x, start_mm_.y,
                                   config::kStartCaptureRadiusMm)) {
      await_start_ = false;
      audio_.playStart();   // brief "returned to start / armed" blip
    }
    // ARMED -> COMPLETE: reaching an OPEN path's end releases guidance force so
    // the controller never shoves the handle off the end. Closed loops (circle)
    // have no end. Not checked while heading to the start (await_start_).
    if (!is_closed_ && !await_start_ && !reached_end_ &&
        welding::ui::withinCapture(s.x, s.y, end_mm_.x, end_mm_.y,
                                   config::kEndReachedRadiusMm)) {
      reached_end_      = true;
      end_reached_time_ = t;   // start the 3-hoop completion animation
      audio_.playComplete();   // pleasant one-shot success chime
    }
  }

  // Completion auto-reset: after the hoop has pulsed kCompletionHoopCount times,
  // reload the same shape (clears the trail, re-enters the RETURN-TO-START GAP)
  // so the trainee can immediately do another rep without pressing R.
  if (reached_end_ &&
      (t - end_reached_time_) >=
          config::kCompletionHoopCount * config::kCompletionHoopPeriodS) {
    setActivePath(catalog_.active().json_file, /*enter_gap=*/true);
  }

  // Latch transition: stop audio exactly once.
  if (latched_ && !audio_stopped_) {
    audio_.stop();
    audio_stopped_ = true;
  }

  // Pantograph debug overlay: when the overlay is on, reconstruct the joint
  // state once per frame and push a sample for the time-series plots. The ERM
  // breakdown + path errors come straight off the captured OuterCmd (the
  // controller already computed them). Gated on the toggle so it costs nothing
  // when hidden; the buffer fills over a few seconds after you press 'D'.
  if (debug_view_) {
    last_jds_ = welding::debug::computeJointDebugState(
        last_snap_, last_cmd_.fx_N, last_cmd_.fy_N, force_on_);
    welding::debug::DebugSample smp;
    smp.tau1 = last_jds_.tau1;       smp.tau2 = last_jds_.tau2;
    smp.erm_pwm = last_cmd_.erm_pwm;
    smp.erm_base = last_cmd_.erm_base;
    smp.erm_speed = last_cmd_.erm_speed;
    smp.erm_blow = last_cmd_.erm_blow;
    smp.det = last_jds_.det;
    smp.speed = last_jds_.speed_mms; smp.target = target_speed_;
    smp.perp_err = last_cmd_.perp_err;
    smp.vel_err = last_cmd_.vel_err;
    smp.force_on = force_on_;        smp.pose_valid = last_jds_.pose_valid;
    debug_hist_.push(smp);
  }
}

//--------------------------------------------------------------
void ofApp::draw() {
  // --- Linkage simulator mode -----------------------------------------------
  if (linkage_mode_) {
    linkage_renderer_.draw();
    linkage_gui_.draw();

    // Mode label, bottom-left.
    ofPushStyle();
    ofSetColor(160, 220, 255);
    hud_font_.drawString("SIM: linkage  [C] clear trace",
                         static_cast<int>(std::lround(10 * ui_scale_)),
                         ofGetHeight() - static_cast<int>(std::lround(14 * ui_scale_)));
    ofPopStyle();
    return;  // <-- do NOT fall through to trainer draw
  }

  // --- Trainer mode (existing, unchanged) -----------------------------------
  renderer_.draw();  // bead + bloom + workspace bounds

  // --- HUD ------------------------------------------------------------------
  // Top status banner.
  ofPushStyle();
  const bool connected = (source_kind_ != SourceKind::kNone);
  if (connected) {
    ofSetColor(40, 160, 70);  // green
  } else {
    ofSetColor(170, 60, 50);  // red
  }
  const int mgn   = static_cast<int>(std::lround(10 * ui_scale_));
  const int bar_h = static_cast<int>(std::lround(26 * ui_scale_));
  const int row1  = static_cast<int>(std::lround(19 * ui_scale_));  // baseline in the bar
  ofDrawRectangle(0, 0, ofGetWidth(), bar_h);
  ofSetColor(235);
  hud_font_.drawString(banner_text_, mgn, row1);

  // Speedometer.
  char speedo[64];
  std::snprintf(speedo, sizeof(speedo), "|v| = %5.1f mm/s", last_speed_mms_);
  hud_font_.drawString(speedo,
      ofGetWidth() - static_cast<int>(hud_font_.stringWidth(speedo)) - mgn, row1);

  // Blow-through (over-penetration) count.
  char burns[48];
  std::snprintf(burns, sizeof(burns), "BURN-THROUGHS: %d", renderer_.burnThroughCount());
  ofSetColor(255, 140, 60);
  const int row2 = bar_h + static_cast<int>(std::lround(20 * ui_scale_));
  hud_font_.drawString(burns,
      ofGetWidth() - static_cast<int>(hud_font_.stringWidth(burns)) - mgn, row2);

  // Conduction state + per-frame heat-update time. [C] toggles it; the
  // microsecond readout is the conduction-on/off latency A/B.
  char heatline[72];
  std::snprintf(heatline, sizeof(heatline), "heat: %lld us (conduction %s)  [C]",
                renderer_.heatUpdateMicros(),
                renderer_.conductionEnabled() ? "ON" : "OFF");
  ofSetColor(150, 200, 255);
  const int row3 = bar_h + static_cast<int>(std::lround(42 * ui_scale_));
  hud_font_.drawString(heatline,
      ofGetWidth() - static_cast<int>(hud_font_.stringWidth(heatline)) - mgn, row3);

  // LOG FULL indicator.
  if (log_ && log_->overflow()) {
    ofSetColor(220, 50, 40);
    hud_font_.drawString("LOG FULL",
        ofGetWidth() - static_cast<int>(hud_font_.stringWidth("LOG FULL")) - mgn,
        ofGetHeight() - static_cast<int>(std::lround(12 * ui_scale_)));
  }

  // View mode + weld-quality legend, bottom-left, clear of the gain panel.
  const int vy = ofGetHeight() - static_cast<int>(std::lround(14 * ui_scale_));
  ofSetColor(200);
  hud_font_.drawString(renderer_.qualityView() ? "VIEW: weld quality  [Tab]"
                                               : "VIEW: live  [Tab]", mgn, vy);
  if (renderer_.qualityView()) {
    const int   ly  = vy - static_cast<int>(std::lround(20 * ui_scale_));
    const float gap = 12.0f * ui_scale_;
    float lx = static_cast<float>(mgn);
    ofSetColor(64, 90, 153);  hud_font_.drawString("under-fused", lx, ly);
    lx += hud_font_.stringWidth("under-fused") + gap;
    ofSetColor(51, 191, 77);  hud_font_.drawString("good", lx, ly);
    lx += hud_font_.stringWidth("good") + gap;
    ofSetColor(230, 77, 26);  hud_font_.drawString("over-penetration", lx, ly);
  }
  ofPopStyle();

  // Gain sliders ('G' hides/shows the panel).
  if (show_gains_) gui_.draw();

  // Shape-library icon strip (clickable; mirrors [ / ] / R keys).
  drawShapeStrip();

  // --- Reset / shape-switch GAP prompt --------------------------------------
  // Mark the active shape's start and tell the trainee guidance is paused until
  // they get there. Suppressed under the E-stop overlay (latched takes over).
  if (await_start_ && !latched_) {
    ofPushStyle();
    const glm::vec2 sp = renderer_.worldToScreen(start_mm_.x, start_mm_.y);
    const float r = 10.0f * ui_scale_;
    ofNoFill();
    ofSetColor(120, 200, 255);
    ofSetLineWidth(2.0f);
    ofDrawCircle(sp.x, sp.y, r);
    ofDrawLine(sp.x - r * 1.6f, sp.y, sp.x + r * 1.6f, sp.y);
    ofDrawLine(sp.x, sp.y - r * 1.6f, sp.x, sp.y + r * 1.6f);
    ofFill();
    ofSetLineWidth(1.0f);
    ofSetColor(150, 210, 255);
    hud_font_.drawString("RETURN TO START - guidance paused (move the handle to the marker)",
                         mgn, bar_h + static_cast<int>(std::lround(64 * ui_scale_)));
    ofPopStyle();
  }

  // --- End-of-weld completion marker ----------------------------------------
  // Reached the end of an open path: guidance has released. Two green hoops
  // shrink onto the end point (a "locked on / done" pulse), plus a label.
  if (reached_end_ && !latched_) {
    ofPushStyle();
    const glm::vec2 ep      = renderer_.worldToScreen(end_mm_.x, end_mm_.y);
    const float base        = 32.0f * ui_scale_;
    const float period      = config::kCompletionHoopPeriodS;
    const float elapsed     = ofGetElapsedTimef() - end_reached_time_;
    const int   pulse       = static_cast<int>(elapsed / period);       // 0,1,2
    const float ph          = std::fmod(elapsed, period) / period;      // 0..1
    const float rr          = base * (1.0f - ph) + 4.0f * ui_scale_;    // shrinks in
    ofNoFill();
    ofSetLineWidth(2.5f);
    ofSetColor(60, 230, 120, ofClamp(static_cast<int>(230.0f * (1.0f - ph)), 0, 255));
    ofDrawCircle(ep.x, ep.y, rr);
    ofFill();
    ofSetColor(60, 230, 120);
    ofDrawCircle(ep.x, ep.y, 4.0f * ui_scale_);   // solid center
    ofSetLineWidth(1.0f);
    char msg[96];
    std::snprintf(msg, sizeof(msg),
                  "WELD COMPLETE - guidance released, resetting (%d/%d)",
                  std::min(pulse + 1, config::kCompletionHoopCount),
                  config::kCompletionHoopCount);
    ofSetColor(120, 235, 160);
    hud_font_.drawString(msg, mgn, bar_h + static_cast<int>(std::lround(64 * ui_scale_)));
    ofPopStyle();
  }

  // --- Safety overlay -------------------------------------------------------
  if (latched_) {
    ofPushStyle();
    ofSetColor(0, 0, 0, 150);
    ofDrawRectangle(0, 0, ofGetWidth(), ofGetHeight());
    const std::string msg = "SAFETY TRIPPED - press Enter to re-arm";
    const float tw  = hud_font_.stringWidth(msg);
    const float tx  = (ofGetWidth() - tw) * 0.5f;
    const float ty  = ofGetHeight() * 0.5f;          // text baseline
    const float pad = 8.0f * ui_scale_;
    ofSetColor(60, 0, 0);                              // highlight box behind text
    ofDrawRectangle(tx - pad, ty - hud_font_.getAscenderHeight() - pad,
                    tw + 2 * pad, hud_font_.getLineHeight() + 2 * pad);
    ofSetColor(255, 80, 80);
    hud_font_.drawString(msg, tx, ty);
    ofPopStyle();
  }

  // --- Force-direction arrow ('A' toggle) -----------------------------------
  // Which way the handle is being nudged (the cartesian guidance force fx,fy at
  // the torch). Only while force is actually on; length scales with |F| up to
  // kMaxForceN. We probe the workspace->screen transform with the force as a
  // small mm offset so the arrow inherits the renderer's letterbox + y-flip
  // correctly. Toggled independently of the debug overlay.
  if (show_force_arrow_ && force_on_) {
    const float fmag = std::hypot(last_cmd_.fx_N, last_cmd_.fy_N);
    if (fmag > 1e-3f) {
      const glm::vec2 p0 = renderer_.worldToScreen(last_snap_.x, last_snap_.y);
      const glm::vec2 pf = renderer_.worldToScreen(last_snap_.x + last_cmd_.fx_N,
                                                   last_snap_.y + last_cmd_.fy_N);
      glm::vec2 dir = pf - p0;                       // screen-space force direction
      const float dlen = glm::length(dir);
      if (dlen > 1e-4f) {
        const float maxLen = 80.0f * ui_scale_;
        const float len = std::min(fmag / config::kMaxForceN, 1.0f) * maxLen;
        const glm::vec2 u = dir / dlen;              // unit dir
        const glm::vec2 tip = p0 + u * len;
        const glm::vec2 perp(-u.y, u.x);
        const float ah = 11.0f * ui_scale_;          // arrowhead size
        ofPushStyle();
        ofSetColor(90, 220, 255);
        ofSetLineWidth(std::max(2.0f, 2.5f * ui_scale_));
        ofDrawLine(p0.x, p0.y, tip.x, tip.y);
        ofDrawLine(tip.x, tip.y, tip.x - u.x * ah + perp.x * ah * 0.5f,
                                 tip.y - u.y * ah + perp.y * ah * 0.5f);
        ofDrawLine(tip.x, tip.y, tip.x - u.x * ah - perp.x * ah * 0.5f,
                                 tip.y - u.y * ah - perp.y * ah * 0.5f);
        ofPopStyle();
      }
    }
  }

  // --- Pantograph debug overlay ('D'), drawn last so it sits on top ---------
  // Arm pose + numerics + time-series plots (torque/ERM/singularity/speed) in a
  // right-side panel. last_jds_ + debug_hist_ are produced in update().
  if (debug_view_) {
    float ix, iy, iw, ih;
    welding::debug::debugInsetRect(static_cast<float>(ofGetWidth()),
                                   static_cast<float>(ofGetHeight()), ix, iy, iw, ih);
    pantograph_debug_.draw(last_jds_, ofRectangle(ix, iy, iw, ih), debug_hist_,
                           force_off_reason_);
  }

  // --- Keyboard-shortcut overlay ('L' toggle) -------------------------------
  // Single on-screen source of truth for the controls; keep README's table in
  // sync by hand. Dims the scene and lists every trainer-mode binding.
  if (show_help_) {
    struct Shortcut { const char* key; const char* action; };
    static const Shortcut kShortcuts[] = {
      {"Space", "E-STOP (latch, cut force + audio)"},
      {"Enter", "Arm / power motors (clear E-stop)"},
      {"Tab",   "Toggle live-heat vs weld-quality view"},
      {"C",     "Toggle heat conduction"},
      {"R",     "Reset trial (reload shape, return to start)"},
      {"[ / ]", "Previous / next shape"},
      {"T",     "Re-tare to start (re-home dot + reset)"},
      {"D",     "Toggle pantograph debug overlay"},
      {"A",     "Show / hide the force-direction arrow"},
      {"G",     "Show / hide the Gains panel"},
      {"L",     "Show / hide this help"},
    };
    const int n = static_cast<int>(sizeof(kShortcuts) / sizeof(kShortcuts[0]));

    ofPushStyle();
    ofSetColor(0, 0, 0, 170);  // dim the whole scene
    ofDrawRectangle(0, 0, ofGetWidth(), ofGetHeight());

    const float pad    = 22.0f * ui_scale_;
    const float lineH  = 22.0f * ui_scale_;
    const float keyW   = 78.0f * ui_scale_;
    const float titleH = 34.0f * ui_scale_;
    const float boxW   = 470.0f * ui_scale_;
    const float boxH   = titleH + pad + n * lineH + pad;
    const float bx     = (ofGetWidth()  - boxW) * 0.5f;
    const float by     = (ofGetHeight() - boxH) * 0.5f;

    ofSetColor(24, 26, 32, 235);
    ofDrawRectangle(bx, by, boxW, boxH);
    ofNoFill();
    ofSetColor(90, 110, 140);
    ofDrawRectangle(bx, by, boxW, boxH);
    ofFill();

    ofSetColor(235);
    hud_font_.drawString("KEYBOARD SHORTCUTS", bx + pad, by + titleH);
    ofSetColor(150);
    hud_font_.drawString("[L] to close",
        bx + boxW - pad - hud_font_.stringWidth("[L] to close"), by + titleH);

    float ty = by + titleH + pad + lineH;
    for (int i = 0; i < n; ++i) {
      ofSetColor(120, 200, 255);
      hud_font_.drawString(kShortcuts[i].key, bx + pad, ty);
      ofSetColor(220);
      hud_font_.drawString(kShortcuts[i].action, bx + pad + keyW, ty);
      ty += lineH;
    }
    ofPopStyle();
  }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
  // --- Linkage simulator mode -----------------------------------------------
  if (linkage_mode_) {
    if (key == 'c' || key == 'C') {
      linkage_renderer_.clearTrace();
    }
    return;  // <-- do NOT run trainer spacebar/enter/tab logic in linkage mode
  }

  // --- Trainer mode (existing, unchanged) -----------------------------------
  if (key == ' ') {
    // E-stop: latch, cut force (handled in update) + audio.
    latched_ = true;
    audio_.stop();
    audio_stopped_ = true;
  } else if (key == OF_KEY_RETURN) {
    // Enter POWERS the motors (sendArm + clears the E-stop latch) but is now
    // decoupled from trial state: it no
    // longer clears the RETURN-TO-START gap. At first launch the firmware boots
    // disarmed (polarity_ok=false, motors off) while comms look healthy, so the
    // operator presses Enter to power up — but that used to ALSO skip the gap and
    // make the start marker vanish. Now arming leaves you IN return-to-start with
    // the marker up; guidance force ramps in only once the handle reaches
    // the start. Force still additionally gates on the firmware's secondary-ready
    // + link-fresh + non-JOG build, so this can't surprise-engage.
    // Leaving the gap is now ONLY by reaching the start marker (or R/[ /]).
    latched_       = false;
    audio_stopped_ = false;
    if (sink_) sink_->sendArm();  // safe on NullSink (just counts)
  } else if (key == '\t') {
    renderer_.toggleQualityView();  // live heat glow <-> weld-quality map
  } else if (key == 'c' || key == 'C') {
    renderer_.toggleConduction();   // spatial heat conduction on/off
  } else if (key == 'r' || key == 'R') {
    // Reset this trial: reload the SAME shape (clears trail + heat + re-primes
    // the filter) and enter the GAP so the trainee walks back to the start.
    setActivePath(catalog_.active().json_file, /*enter_gap=*/true);
  } else if (key == '[') {
    setActivePath(catalog_.prev(), /*enter_gap=*/true);  // previous shape
  } else if (key == ']') {
    setActivePath(catalog_.next(), /*enter_gap=*/true);  // next shape
  } else if (key == 'd' || key == 'D') {
    debug_view_ = !debug_view_;     // pantograph debug overlay, trainer only
  } else if (key == 't' || key == 'T') {
    retarePosition();               // re-home dot 20 mm above start + reset
  } else if (key == 'g' || key == 'G') {
    show_gains_ = !show_gains_;      // hide/show the Gains panel
  } else if (key == 'l' || key == 'L') {
    show_help_ = !show_help_;        // keyboard-shortcut overlay
  } else if (key == 'a' || key == 'A') {
    show_force_arrow_ = !show_force_arrow_;  // force-direction arrow, independent of debug
  }
}

//--------------------------------------------------------------
// retarePosition — re-home the dot to 20 mm above the active shape's start
// (treating the operator's current physical max-reach / pi/2 pose as that spot),
// then drop into RETURN-TO-START so the trainee descends onto the start to arm.
// Same anchor setActivePath seeds at boot, so a 'T' press at the reference pose
// is idempotent with the fresh-boot layout.
//--------------------------------------------------------------
void ofApp::onRetarePressed() { retarePosition(); }

void ofApp::retarePosition() {
  if (!ct_) return;
  const float ax = start_mm_.x;
  const float ay = start_mm_.y + config::kTareAboveStartMm;  // +y is up -> above the top start
  if (!ct_->tareCurrentTo(ax, ay)) return;   // no snapshot yet -> nothing to tare
  await_start_ = true;   // RETURN-TO-START: force off; trainee descends to the start to arm
  reached_end_ = false;  // a re-home starts a fresh attempt even after a completed weld
  audio_.playStart();    // brief confirm blip (same cue as reaching the start)
}

//--------------------------------------------------------------
void ofApp::exit() {
  if (cursor_hidden_) ofShowCursor();  // restore the OS cursor on quit
  retare_btn_.removeListener(this, &ofApp::onRetarePressed);
  audio_.stop();
  if (log_) log_->flushToCsv(kSessionCsv);  // explicit; RAII also covers it
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {
  if (linkage_mode_) {
    linkage_renderer_.onResize(w, h);
    return;
  }
  renderer_.onResize(w, h);
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {}
void ofApp::mouseMoved(int x, int y) {}
void ofApp::mouseDragged(int x, int y, int button) {}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
  if (linkage_mode_) return;  // no shape strip in linkage mode
  std::vector<ofRectangle> cells;
  ofRectangle reset_cell;
  layoutStrip(cells, reset_cell);
  const float fx = static_cast<float>(x), fy = static_cast<float>(y);
  for (std::size_t i = 0; i < cells.size(); ++i) {
    if (cells[i].inside(fx, fy)) {
      setActivePath(catalog_.select(i), /*enter_gap=*/true);  // pick a shape
      return;
    }
  }
  if (reset_cell.inside(fx, fy)) {
    setActivePath(catalog_.active().json_file, /*enter_gap=*/true);  // reset cell
  }
}
void ofApp::mouseReleased(int x, int y, int button) {}
void ofApp::mouseEntered(int x, int y) {}
void ofApp::mouseExited(int x, int y) {}
void ofApp::gotMessage(ofMessage msg) {}
void ofApp::dragEvent(ofDragInfo dragInfo) {}
