#pragma once

#include "ofMain.h"
#include "ofxGui.h"

#include <memory>
#include <string>

#include "Renderer.h"
#include "AudioEngine.h"
#include "InputSource.h"
#include "MotorSink.h"
#include "ControlThreads.h"
#include "PathLibrary.h"
#include "ShapeCatalog.h"
#include "UiGeometry.h"
#include "SessionLog.h"
#include "LinkageRenderer.h"
#include "PantographDebugView.h"

#include <vector>

// The integration layer. Owns the full CPU stack (source -> control ->
// sink), the renderer, the audio engine, the session log, and the runtime
// gain-tuning HUD. The control loop runs SINGLE-THREADED from update()
// (ControlThreads::start() is deliberately NOT called in v1; see the threading
// note in ofApp.cpp). Spacebar latches a software E-stop; Enter re-arms.
//
// Adds "--mode=linkage" simulation mode. When active, the trainer stack
// (source_/sink_/ct_/log_/renderer_/audio_) is never created; only
// linkage_renderer_ and linkage_gui_ are used.
class ofApp : public ofBaseApp {
 public:
  void setup();
  void update();
  void draw();
  void exit();

  void keyPressed(int key);
  void keyReleased(int key);
  void mouseMoved(int x, int y);
  void mouseDragged(int x, int y, int button);
  void mousePressed(int x, int y, int button);
  void mouseReleased(int x, int y, int button);
  void mouseEntered(int x, int y);
  void mouseExited(int x, int y);
  void windowResized(int w, int h);
  void dragEvent(ofDragInfo dragInfo);
  void gotMessage(ofMessage msg);

  // Set by main() before ofRunApp; parsed in setup().
  void setSourceSpec(const std::string& spec) { source_spec_ = spec; }
  void setModeSpec(const std::string& s) { mode_spec_ = s; }

 private:
  // --- Shape library + reset (single source of truth for the active path) ---
  // setActivePath() is the ONE place the active reference path is swapped: it
  // loads the JSON, points BOTH the controller (force) and the renderer (visual
  // guide) at the same Path, re-primes the input filter, clears the trail, and
  // (when enter_gap) drops into the force-off GAP until the handle returns to
  // the new shape's start. Reset = setActivePath(active, true); shape switch =
  // setActivePath(other, true); boot = setActivePath(active, false).
  void setActivePath(const std::string& json_file, bool enter_gap);
  void buildThumbnails();   // sample each catalog shape's polyline once (icons)
  // Re-tare to start: re-home the dot to 20 mm above the active shape's
  // start (treating the current physical max-reach pose as that spot) and drop
  // into the RETURN-TO-START gap. Bound to the 'T' key and the Gains-panel button.
  void retarePosition();
  void onRetarePressed();   // ofxButton void listener -> retarePosition()
  // Strip layout: the shape-icon cells (one per catalog entry) plus the reset
  // cell, laid out along the bottom of the window. Shared by draw + hit-test.
  void layoutStrip(std::vector<ofRectangle>& shape_cells,
                   ofRectangle& reset_cell) const;
  void drawShapeStrip() const;
  bool overStrip(float x, float y) const;  // cursor over any strip cell?

  // Human-readable label of the active source, for the HUD banner.
  enum class SourceKind { kMouse, kReplay, kSerial, kNone };

  std::string source_spec_ = "mouse";
  std::string mode_spec_   = "trainer";
  bool        linkage_mode_ = false;

  SourceKind  source_kind_ = SourceKind::kMouse;
  std::string banner_text_;

  // --- CPU stack (declaration order matters: ct_ holds raw pointers into the
  //     source/sink/path/log, so those must be declared BEFORE ct_ and thus
  //     destroyed AFTER it) ---------------------------------------------------
  ofSerial                              serial_;       // serial mode only
  std::unique_ptr<welding::input::IInputSource> source_;
  welding::input::MouseSource*          mouse_ = nullptr;   // typed handle if mouse mode
  std::unique_ptr<welding::motor::IMotorSink>   sink_;
  std::unique_ptr<welding::path::Path>          path_;
  std::unique_ptr<welding::log::SessionLog>     log_;
  std::unique_ptr<welding::control::ControlThreads> ct_;

  // --- Presentation ---------------------------------------------------------
  welding::render::Renderer   renderer_;
  welding::audio::AudioEngine audio_;

  // --- HUD: runtime gain tuning ---------------------------------------------
  ofxPanel        gui_;
  ofParameter<float> kp_perp_;
  ofParameter<float> kd_perp_;
  ofParameter<float> kp_tan_;
  ofParameter<float> target_speed_;
  ofParameter<bool>  erm_enabled_;   // ERM haptic on/off toggle
  ofxButton          retare_btn_;    // "Re-tare to start" — re-home dot + reset
  bool  show_gains_ = true;   // 'G' toggles the Gains panel visibility
  bool  show_help_  = false;  // 'L' toggles the keyboard-shortcut overlay
  bool  show_force_arrow_ = true;  // 'A' toggles the force-direction arrow, independent of debug_view_

  // --- Shape library (selectable guide shapes) ------------------------------
  welding::shape::ShapeCatalog catalog_;   // ordered list + active index
  // Cached polyline (workspace mm) of every catalog shape, for the icon
  // thumbnails. Built once in setup(); fit to each cell rect at draw time.
  std::vector<std::vector<welding::ui::Pt>> thumbs_;

  // --- Force-feedback GAP (reset / shape-switch) ----------------------------
  // When true, guidance force is OFF (like latched_) and the HUD shows
  // "RETURN TO START". Cleared when the handle reaches the active shape's start
  // (within kStartCaptureRadiusMm) or on Enter. Reset and shape switch both set
  // it; the trainee moves back across this gap (no haptic pull during it).
  bool      await_start_ = false;
  glm::vec2 start_mm_{0.0f, 0.0f};  // active shape's start point (workspace mm)

  // --- Arm-in ramp ----------------------------------------------------------
  // On the gap→armed transition (handle reaches start / Enter / auto-reset /
  // E-stop release), force + ERM ease 0→full over kArmRampS rather than stepping
  // on. arm_time_ stamps the instant guidance armed; was_armed_ detects the edge.
  bool      was_armed_ = false;
  float     arm_time_  = 0.0f;       // ofGetElapsedTimef() when guidance last armed

  // --- End-of-weld release (open paths only) --------------------------------
  // When the handle reaches the end of an OPEN path, the weld is complete:
  // guidance force releases (the controller otherwise keeps nudging past the
  // end) and a green shrinking hoop marks completion. Closed paths (the circle)
  // have no end (start == end), so reached_end_ never trips for them.
  glm::vec2 end_mm_{0.0f, 0.0f};    // active shape's end point (workspace mm)
  bool      is_closed_   = false;   // start ~= end → a loop, no endpoint
  bool      reached_end_ = false;   // latched once the end is reached
  float     end_reached_time_ = 0.0f;  // ofGetElapsedTimef() when reached_end_ set;
                                       // drives the 3-hoop animation + auto-reset

  // --- Safety (software E-stop) ---------------------------------------------
  bool  latched_       = false;  // E-stop latched? no force while true
  bool  audio_stopped_ = false;  // ensure audio_.stop() fires once per latch

  // Latest snapshot speed for the speedometer HUD.
  float last_speed_mms_ = 0.0f;

  // --- Pantograph debug overlay ('D' toggle, trainer mode only) -------------
  // Reconstructs the joint world (theta1,theta2 + commanded tau1,tau2) from the
  // live reported tip + the captured outer-loop force, and draws it as a corner
  // inset over the weld scene. Default OFF every launch (no config persistence).
  welding::render::PantographDebugView pantograph_debug_;
  bool        debug_view_ = false;
  welding::control::OuterCmd last_cmd_{};      // force captured AT the gating site
  bool        force_on_ = false;               // outer loop emitting force this frame?
  std::string force_off_reason_;               // "E-STOP" / "GAP" / "DONE" when not
  welding::input::StateSnapshot last_snap_{};  // latest snapshot, for the overlay in draw()
  // Time-series plots: per-frame samples (torque/ERM/singularity/speed),
  // pushed in update() when the overlay is on, drawn from in draw().
  welding::debug::DebugHistory    debug_hist_{300};  // ~5 s @ 60 fps
  welding::debug::JointDebugState last_jds_{};        // reconstructed once per frame in update()

  // --- Cursor hiding (trainer mouse mode) -----------------------------------
  // In mouse mode the OS cursor IS the torch — its position is injected every
  // frame (see update()). We hide it while it's over the weld workspace so the
  // rendered torch is the only pointer, and restore it over the gain-tuning
  // panel so the sliders stay usable. Tracked here so ofHide/ShowCursor
  // fire only on the boundary crossing, not every frame.
  bool cursor_hidden_ = false;

  // --- High-DPI UI scaling (computed in setup() from the display) -----------
  // The window opens at a fraction of the screen and the HUD uses a TTF sized
  // by this factor, so text/panels stay legible on high-DPI displays. The
  // bead/linkage GEOMETRY is NOT scaled by this — it fits the window via the
  // workspace letterbox transform.
  float          ui_scale_ = 1.0f;   // ~1 at 96dpi; ~2.4 on a 2400px-tall 300% panel
  ofTrueTypeFont hud_font_;          // scaled HUD text (replaces fixed 8px bitmap font)

  // --- Linkage simulator (only used when linkage_mode_ == true) -------------
  // None of these are touched when linkage_mode_ == false.
  welding::render::LinkageRenderer linkage_renderer_;

  ofxPanel           linkage_gui_;
  ofParameter<float> lk_theta_;  // input crank angle (rad)
  ofParameter<float> lk_a_;      // link a length (mm)
  ofParameter<float> lk_b_;      // link b length (mm)
  ofParameter<float> lk_c_;      // link c length (mm)
  ofParameter<float> lk_d_;      // link d length (mm)
  ofParameter<float> lk_s_;      // coupler fraction along A->B [0,1]
  ofParameter<float> lk_h_;      // coupler perp offset (mm)
};
