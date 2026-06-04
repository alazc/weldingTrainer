#include "Renderer.h"

#include <algorithm>
#include <cmath>

#include "PathLibrary.h"

namespace welding {
namespace render {

namespace {

// New bead vertex is appended only after the pen has travelled this far since
// the last one. Keeps vertex/heat-cell count bounded when the pen is still.
constexpr float kVertexStepMm = 1.5f;

// Bead drawn as a triangle strip of this half-width (pixels) around the path.
constexpr float kBeadHalfWidthPx = 5.0f;

// Spark emission: count scales with speed, clamped to a few per frame.
constexpr float kSparkSpeedToCount = 0.20f;  // sparks per (mm/s)
constexpr int   kMaxSparksPerFrame = 12;

// Arc glow disc radius (pixels).
constexpr float kArcRadiusPx = 50.0f;

// --- Bloom shaders --------------------------------------------------------
// Self-contained inline GLSL (#version 150, GL_TEXTURE_2D / sampler2D so we
// can use normalized 0..1 texcoords). Separable 9-tap gaussian.
//
// A full-screen quad is drawn with ofTexture::draw, which feeds gl_MultiTexCoord
// style attributes through oF's default attribs; in the programmable pipeline
// oF supplies "texcoord" and "modelViewProjectionMatrix" / "position". We use
// the standard oF shader entry points.

const char* kVertSrc = R"GLSL(
#version 150
uniform mat4 modelViewProjectionMatrix;
in vec4 position;
in vec2 texcoord;
out vec2 vUv;
void main() {
  vUv = texcoord;
  gl_Position = modelViewProjectionMatrix * position;
}
)GLSL";

// Bright-pass + horizontal blur. Threshold keeps only the glowing bits so the
// bloom reads as light bleed rather than a uniform haze.
const char* kFragBrightH = R"GLSL(
#version 150
uniform sampler2D tex0;
uniform vec2  texelSize;   // 1.0 / resolution
uniform float threshold;
in  vec2 vUv;
out vec4 fragColor;

const float w0 = 0.2270270270;
const float w1 = 0.1945945946;
const float w2 = 0.1216216216;
const float w3 = 0.0540540541;
const float w4 = 0.0162162162;

vec3 brightPass(vec2 uv) {
  vec3 c = texture(tex0, uv).rgb;
  float l = dot(c, vec3(0.2126, 0.7152, 0.0722));
  float k = max(l - threshold, 0.0) / max(l, 0.0001);
  return c * k;
}

void main() {
  float dx = texelSize.x;
  vec3 sum = brightPass(vUv) * w0;
  sum += brightPass(vUv + vec2( dx, 0.0)) * w1;
  sum += brightPass(vUv + vec2(-dx, 0.0)) * w1;
  sum += brightPass(vUv + vec2( 2.0*dx, 0.0)) * w2;
  sum += brightPass(vUv + vec2(-2.0*dx, 0.0)) * w2;
  sum += brightPass(vUv + vec2( 3.0*dx, 0.0)) * w3;
  sum += brightPass(vUv + vec2(-3.0*dx, 0.0)) * w3;
  sum += brightPass(vUv + vec2( 4.0*dx, 0.0)) * w4;
  sum += brightPass(vUv + vec2(-4.0*dx, 0.0)) * w4;
  fragColor = vec4(sum, 1.0);
}
)GLSL";

// Vertical blur of the already bright-passed buffer.
const char* kFragBlurV = R"GLSL(
#version 150
uniform sampler2D tex0;
uniform vec2 texelSize;
in  vec2 vUv;
out vec4 fragColor;

const float w0 = 0.2270270270;
const float w1 = 0.1945945946;
const float w2 = 0.1216216216;
const float w3 = 0.0540540541;
const float w4 = 0.0162162162;

void main() {
  float dy = texelSize.y;
  vec3 sum = texture(tex0, vUv).rgb * w0;
  sum += texture(tex0, vUv + vec2(0.0,  dy)).rgb * w1;
  sum += texture(tex0, vUv + vec2(0.0, -dy)).rgb * w1;
  sum += texture(tex0, vUv + vec2(0.0,  2.0*dy)).rgb * w2;
  sum += texture(tex0, vUv + vec2(0.0, -2.0*dy)).rgb * w2;
  sum += texture(tex0, vUv + vec2(0.0,  3.0*dy)).rgb * w3;
  sum += texture(tex0, vUv + vec2(0.0, -3.0*dy)).rgb * w3;
  sum += texture(tex0, vUv + vec2(0.0,  4.0*dy)).rgb * w4;
  sum += texture(tex0, vUv + vec2(0.0, -4.0*dy)).rgb * w4;
  fragColor = vec4(sum, 1.0);
}
)GLSL";

}  // namespace

Renderer::Renderer() = default;

// =============================================================================
// setup / resize / shaders / fbos
// =============================================================================
void Renderer::setup() {
  heat_.setBurnExposure(welding::config::kBurnExposure);  // arm blow-through latch
  // Same constant the ERM blow-through envelope uses (ControlThreads), so the
  // visual burnout lockout and the haptic refractory stay in lock-step.
  heat_.setBurnRefractory(welding::config::kErmBlowRefractoryS);
  buildShaders();
  allocateFbos(ofGetWidth(), ofGetHeight());
}

void Renderer::onResize(int w, int h) {
  if (w <= 0 || h <= 0) return;
  allocateFbos(w, h);
}

void Renderer::allocateFbos(int w, int h) {
  fbo_w_ = w;
  fbo_h_ = h;

  // Use normalized GL_TEXTURE_2D so the bloom shaders can sample with 0..1
  // texcoords (sampler2D). Restore the prior ARB state afterward so we don't
  // surprise the rest of the app's texture loads.
  ofDisableArbTex();
  scene_.allocate(w, h, GL_RGBA);
  ping_.allocate(w, h, GL_RGBA);
  pong_.allocate(w, h, GL_RGBA);
  ofEnableArbTex();

  // Clear freshly-allocated FBOs to avoid garbage on the first frame.
  for (ofFbo* f : { &scene_, &ping_, &pong_ }) {
    f->begin();
    ofClear(0, 0, 0, 255);
    f->end();
  }

  recomputeTransform();
}

void Renderer::buildShaders() {
  shaders_ok_ = true;
  shaders_ok_ &= bright_blur_h_.setupShaderFromSource(GL_VERTEX_SHADER,   kVertSrc);
  shaders_ok_ &= bright_blur_h_.setupShaderFromSource(GL_FRAGMENT_SHADER, kFragBrightH);
  bright_blur_h_.bindDefaults();
  shaders_ok_ &= bright_blur_h_.linkProgram();

  shaders_ok_ &= blur_v_.setupShaderFromSource(GL_VERTEX_SHADER,   kVertSrc);
  shaders_ok_ &= blur_v_.setupShaderFromSource(GL_FRAGMENT_SHADER, kFragBlurV);
  blur_v_.bindDefaults();
  shaders_ok_ &= blur_v_.linkProgram();

  if (!shaders_ok_) {
    ofLogError("welding::render::Renderer")
        << "bloom shader build failed; falling back to no-bloom composite";
  }
}

// =============================================================================
// workspace mm -> screen pixels (aspect-preserving letterbox)
// =============================================================================
void Renderer::recomputeTransform() {
  using namespace welding::config;
  const float ws_w = kWorkspaceMaxX - kWorkspaceMinX;
  const float ws_h = kWorkspaceMaxY - kWorkspaceMinY;
  if (ws_w <= 0.0f || ws_h <= 0.0f || fbo_w_ <= 0 || fbo_h_ <= 0) {
    vp_x_ = vp_y_ = 0.0f;
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

glm::vec2 Renderer::toScreen(float mm_x, float mm_y) const {
  using namespace welding::config;
  const float ws_w = kWorkspaceMaxX - kWorkspaceMinX;
  const float ws_h = kWorkspaceMaxY - kWorkspaceMinY;
  const float u = (ws_w > 0.0f) ? (mm_x - kWorkspaceMinX) / ws_w : 0.0f;
  const float v = (ws_h > 0.0f) ? (mm_y - kWorkspaceMinY) / ws_h : 0.0f;
  // +y mm is "into" the workspace; flip to y-down screen (top = far edge).
  return glm::vec2(vp_x_ + u * vp_w_,
                   vp_y_ + (1.0f - v) * vp_h_);
}

// Inverse of toScreen: screen pixels -> workspace mm. Used by ofApp to map the
// OS cursor to a workspace position so the bead lands under the cursor.
glm::vec2 Renderer::screenToWorld(float px, float py) const {
  using namespace welding::config;
  const float ws_w = kWorkspaceMaxX - kWorkspaceMinX;
  const float ws_h = kWorkspaceMaxY - kWorkspaceMinY;
  const float u = (vp_w_ > 0.0f) ? (px - vp_x_) / vp_w_ : 0.0f;
  // Undo the y flip from toScreen.
  const float v = (vp_h_ > 0.0f) ? 1.0f - (py - vp_y_) / vp_h_ : 0.0f;
  return glm::vec2(kWorkspaceMinX + u * ws_w,
                   kWorkspaceMinY + v * ws_h);
}

// Thin outline of the workspace rectangle so the user sees the reachable
// limits. Drawn in screen space via the shared transform.
void Renderer::drawWorkspaceBounds() const {
  using namespace welding::config;
  const glm::vec2 tl = toScreen(kWorkspaceMinX, kWorkspaceMaxY);  // top-left
  const glm::vec2 br = toScreen(kWorkspaceMaxX, kWorkspaceMinY);  // bottom-right
  ofPushStyle();
  ofNoFill();
  ofSetLineWidth(1.5f);
  ofSetColor(70, 90, 110);
  ofDrawRectangle(tl.x, tl.y, br.x - tl.x, br.y - tl.y);
  ofPopStyle();
}

void Renderer::setPath(const welding::path::Path* p) {
  constexpr int kGuideSegments = 32;  // samples per primitive (linear stays collinear)
  ref_path_ = p;
  guide_mm_.clear();
  if (!p || p->empty()) return;
  const std::vector<welding::path::Vec2> pts = p->polyline(kGuideSegments);
  guide_mm_.reserve(pts.size());
  for (const auto& q : pts) guide_mm_.emplace_back(q.x, q.y);
}

void Renderer::drawReferencePath() const {
  const std::size_t n = guide_mm_.size();
  if (n < 2) return;

  // Project the path to screen space.
  std::vector<glm::vec2> sp(n);
  for (std::size_t i = 0; i < n; ++i)
    sp[i] = toScreen(guide_mm_[i].x, guide_mm_[i].y);

  // Per-vertex unit normal (perpendicular to the local tangent), used to
  // expand the path into a band.
  std::vector<glm::vec2> nrm(n);
  for (std::size_t i = 0; i < n; ++i) {
    const glm::vec2 prev = sp[i > 0 ? i - 1 : i];
    const glm::vec2 next = sp[i + 1 < n ? i + 1 : i];
    glm::vec2 t = next - prev;
    const float L = std::sqrt(t.x * t.x + t.y * t.y);
    t = (L > 1e-4f) ? t / L : glm::vec2(1.0f, 0.0f);
    nrm[i] = glm::vec2(-t.y, t.x);
  }

  // A "ghosted" swipe: a soft translucent band whose alpha fades to zero at
  // the edges, so it reads as a faint glow over the work rather than a hard
  // line like the workspace boundary.
  constexpr float kHalfW = 40.0f;                      // px, half band width (live QA: 10->40, 4x thicker correct-path guide)
  const ofFloatColor core(0.62f, 0.78f, 1.0f, 0.32f);  // translucent cool-blue
  const ofFloatColor edge(0.62f, 0.78f, 1.0f, 0.0f);   // transparent at edges

  ofPushStyle();
  ofEnableBlendMode(OF_BLENDMODE_ALPHA);
  // One feathered half-band per side (a separate strip each, so the two halves
  // don't get stitched together): opaque centre rail -> transparent edge rail.
  for (int side = -1; side <= 1; side += 2) {
    ofMesh half;
    half.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
    for (std::size_t i = 0; i < n; ++i) {
      const glm::vec2 c = sp[i];
      const glm::vec2 e = c + nrm[i] * (kHalfW * static_cast<float>(side));
      half.addVertex(glm::vec3(c, 0.0f)); half.addColor(core);
      half.addVertex(glm::vec3(e, 0.0f)); half.addColor(edge);
    }
    half.draw();
  }
  ofPopStyle();
}

// =============================================================================
// update — advance the simulation from a fresh snapshot
// =============================================================================
void Renderer::update(const welding::input::StateSnapshot& s, float dt_s,
                      bool weld_active) {
  const float speed = std::hypot(s.vx, s.vy);
  pen_speed_ = speed;
  const glm::vec2 now_mm(s.x, s.y);

  // Trail / heat / sparks are deposited ONLY while guidance is armed. In
  // an idle state (RETURN-TO-START gap, end-of-weld, E-stop) we still track the
  // torch cursor (pen_mm_/has_pen_ below) and age existing sparks, but we lay
  // down no new bead vertices, accumulate no dwell heat (so no burnouts), and
  // emit no sparks — and we skip decay so a finished bead freezes as the result.
  if (weld_active) {
    // Append a bead/heat vertex on first sample, or once the pen has moved far
    // enough since the last vertex. Otherwise keep accumulating onto the
    // current (most-recent) cell so dwell heat builds up while standing still.
    bool append = false;
    if (!has_pen_ || heat_.size() == 0) {
      append = true;
    } else {
      const Vec2 last = heat_.posAt(heat_.size() - 1);
      const float dx = last.x - now_mm.x;
      const float dy = last.y - now_mm.y;
      if (std::sqrt(dx * dx + dy * dy) >= kVertexStepMm) append = true;
    }
    if (append) {
      heat_.appendVertex(now_mm.x, now_mm.y);
    }

    // Dwell heat then conductive decay, once per frame.
    heat_.accumulate(now_mm.x, now_mm.y, dt_s);
    if (heat_.conductionEnabled()) heat_.conduct(dt_s);
    heat_.decayAll();

    // Sparks: more sparks the faster the pen moves, clamped per frame.
    int count = static_cast<int>(speed * kSparkSpeedToCount);
    count = std::min(count, kMaxSparksPerFrame);
    if (count > 0) {
      sparks_.emit(s.x, s.y, speed, count,
                   welding::config::kSparkSpeedScale,
                   []{ return ofRandomuf(); });
    }
  }

  pen_mm_  = now_mm;
  has_pen_ = true;

  sparks_.update(dt_s);  // always age existing sparks (none emitted when idle)
}

// =============================================================================
// drawScene — bead + arc + sparks into scene_
// =============================================================================
void Renderer::drawScene() {
  ofClear(0, 0, 0, 255);

  // --- bead: triangle strip with per-vertex heat color --------------------
  // Vertex positions live in heat_; read them back via posAt.
  if (heat_.size() >= 2) {
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
    const std::size_t n = heat_.size();
    for (std::size_t i = 0; i < n; ++i) {
      const Vec2 vi = heat_.posAt(i);
      const glm::vec2 p = toScreen(vi.x, vi.y);

      // Path tangent (forward difference, central where possible).
      const Vec2 vprev = (i > 0)     ? heat_.posAt(i - 1) : vi;
      const Vec2 vnext = (i + 1 < n) ? heat_.posAt(i + 1) : vi;
      glm::vec2 a = toScreen(vprev.x, vprev.y);
      glm::vec2 b = toScreen(vnext.x, vnext.y);
      glm::vec2 dir = b - a;
      const float len = glm::length(dir);
      dir = (len > 1e-4f) ? dir / len : glm::vec2(1.0f, 0.0f);
      const glm::vec2 normal(-dir.y, dir.x);  // perpendicular

      // Live view: transient heat glow. Quality view: permanent peak-heat
      // classification (under-fused / good / over-penetration).
      const Color c = quality_view_
          ? fusionQualityColor(heat_.burnedAt(i), heat_.peakAt(i),
                               welding::config::kFusionMinNorm,
                               welding::config::kFusionIdealNorm)
          : heat_.colorAt(i);
      const ofFloatColor col(c.r, c.g, c.b, 1.0f);

      mesh.addVertex(glm::vec3(p + normal * kBeadHalfWidthPx, 0.0f));
      mesh.addColor(col);
      mesh.addVertex(glm::vec3(p - normal * kBeadHalfWidthPx, 0.0f));
      mesh.addColor(col);
    }
    mesh.draw();
  }

  // --- blow-through markers: hot rim + dark hole at each burned vertex -------
  // Latched (heat_.burnedAt), so the hole persists after the spot cools. The
  // rim is additive (glows + feeds the bloom); the hole is punched dark on top.
  ofPushStyle();
  for (std::size_t i = 0; i < heat_.size(); ++i) {
    if (!heat_.burnedAt(i)) continue;
    const Vec2 vi = heat_.posAt(i);
    const glm::vec2 p = toScreen(vi.x, vi.y);
    ofEnableBlendMode(OF_BLENDMODE_ADD);
    ofSetColor(255, 90, 20);                        // hot oxidized rim
    ofDrawCircle(p.x, p.y, welding::config::kBurnRimRadiusPx);
    ofEnableBlendMode(OF_BLENDMODE_DISABLED);
    ofSetColor(8, 6, 4);                            // punched-through hole
    ofDrawCircle(p.x, p.y, welding::config::kBurnHoleRadiusPx);
  }
  ofPopStyle();

  // --- arc glow + sparks: additive ----------------------------------------
  ofPushStyle();
  ofEnableBlendMode(OF_BLENDMODE_ADD);

  if (has_pen_) {
    const glm::vec2 c = toScreen(pen_mm_.x, pen_mm_.y);
    // Soft radial gradient disc via a triangle fan: bright center -> dark rim.
    ofMesh glow;
    glow.setMode(OF_PRIMITIVE_TRIANGLE_FAN);
    glow.addVertex(glm::vec3(c, 0.0f));
    glow.addColor(ofFloatColor(1.4f, 1.6f, 1.9f, 1.0f));  // hot bluish-white core (HDR -> bloom)
    const int kSegments = 32;
    for (int i = 0; i <= kSegments; ++i) {
      const float a = (static_cast<float>(i) / kSegments) * TWO_PI;
      glow.addVertex(glm::vec3(c.x + std::cos(a) * kArcRadiusPx,
                               c.y + std::sin(a) * kArcRadiusPx, 0.0f));
      glow.addColor(ofFloatColor(0.0f, 0.0f, 0.0f, 1.0f));  // dark rim -> fades
    }
    glow.draw();
  }

  // sparks as small additive points
  const auto& parts = sparks_.particles();
  if (!parts.empty()) {
    ofMesh pts;
    pts.setMode(OF_PRIMITIVE_POINTS);
    for (const Spark& sp : parts) {
      const float fade = (sp.life > 0.0f)
                         ? std::max(0.0f, 1.0f - sp.age / sp.life) : 0.0f;
      const glm::vec2 p = toScreen(sp.x, sp.y);
      pts.addVertex(glm::vec3(p, 0.0f));
      pts.addColor(ofFloatColor(1.0f, 0.85f, 0.5f, 1.0f) * fade);
    }
    glPointSize(4.0f);
    pts.draw();
  }

  ofPopStyle();
}

// =============================================================================
// draw — scene -> bloom -> screen
// =============================================================================
void Renderer::draw() {
  if (fbo_w_ <= 0 || fbo_h_ <= 0) return;

  // 1. scene into scene_
  scene_.begin();
  drawScene();
  scene_.end();

  const ofFloatColor white(1.0f);

  if (shaders_ok_) {
    const glm::vec2 texel(1.0f / static_cast<float>(fbo_w_),
                          1.0f / static_cast<float>(fbo_h_));

    // 2a. bright-pass + horizontal blur: scene_ -> ping_
    ping_.begin();
    ofClear(0, 0, 0, 255);
    bright_blur_h_.begin();
    bright_blur_h_.setUniformTexture("tex0", scene_.getTexture(), 0);
    bright_blur_h_.setUniform2f("texelSize", texel.x, texel.y);
    bright_blur_h_.setUniform1f("threshold", 0.22f);
    scene_.draw(0, 0, fbo_w_, fbo_h_);
    bright_blur_h_.end();
    ping_.end();

    // 2b. vertical blur: ping_ -> pong_
    pong_.begin();
    ofClear(0, 0, 0, 255);
    blur_v_.begin();
    blur_v_.setUniformTexture("tex0", ping_.getTexture(), 0);
    blur_v_.setUniform2f("texelSize", texel.x, texel.y);
    ping_.draw(0, 0, fbo_w_, fbo_h_);
    blur_v_.end();
    pong_.end();
  }

  // 3. composite to screen: scene, then blurred bloom additively on top.
  ofPushStyle();
  ofSetColor(white);
  ofEnableBlendMode(OF_BLENDMODE_DISABLED);
  scene_.draw(0, 0, fbo_w_, fbo_h_);

  if (shaders_ok_) {
    ofEnableBlendMode(OF_BLENDMODE_ADD);
    pong_.draw(0, 0, fbo_w_, fbo_h_);
  }
  ofPopStyle();

  // Workspace boundary outline on top of the composited scene.
  drawReferencePath();
  drawWorkspaceBounds();
}

// =============================================================================
// reset — clear all simulation state (path switch / new run)
// =============================================================================
void Renderer::reset() {
  heat_.clear();
  sparks_.clear();
  has_pen_   = false;
  pen_speed_ = 0.0f;
  pen_mm_    = glm::vec2(0.0f, 0.0f);
}

}  // namespace render
}  // namespace welding
