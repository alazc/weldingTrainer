#include "PathLibrary.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace welding {
namespace path {

namespace {

constexpr float kPi    = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;

inline Vec2 sub(const Vec2& a, const Vec2& b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 add(const Vec2& a, const Vec2& b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 scl(const Vec2& a, float s)       { return {a.x * s,   a.y * s  }; }
inline float dot(const Vec2& a, const Vec2& b){ return a.x*b.x + a.y*b.y;   }
inline float len(const Vec2& a)               { return std::sqrt(dot(a,a)); }
inline Vec2  norm(const Vec2& a) {
  const float l = len(a);
  return (l > 1e-9f) ? Vec2{a.x / l, a.y / l} : Vec2{1.0f, 0.0f};
}

// Signed perpendicular distance from `p` to the line through `pt` along
// tangent `t`. Positive when `p` is on the LEFT of the tangent (matches
// the controller's convention).
inline float signedPerp(const Vec2& p, const Vec2& pt, const Vec2& t) {
  const Vec2 d = sub(p, pt);
  return t.x * d.y - t.y * d.x;
}

// Wrap angle to [-pi, pi).
inline float wrapPi(float a) {
  while (a >=  kPi) a -= kTwoPi;
  while (a <  -kPi) a += kTwoPi;
  return a;
}

}  // namespace

// =============================================================================
// LinearPrimitive
// =============================================================================
PathPoint LinearPrimitive::nearestPoint(float px, float py) const {
  const Vec2 p{px, py};
  const Vec2 d = sub(p1_, p0_);
  const float L2 = dot(d, d);
  float t = 0.0f;
  if (L2 > 1e-12f) {
    t = dot(sub(p, p0_), d) / L2;
    t = std::clamp(t, 0.0f, 1.0f);
  }
  const Vec2 pt = add(p0_, scl(d, t));
  const Vec2 tn = norm(d);
  PathPoint out{};
  out.position = pt;
  out.tangent  = tn;
  out.distance = signedPerp(p, pt, tn);
  out.t        = t;
  return out;
}

void LinearPrimitive::sample(std::vector<Vec2>& out, int segments) const {
  if (segments < 1) segments = 1;
  for (int i = 0; i <= segments; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(segments);
    out.push_back(add(p0_, scl(sub(p1_, p0_), t)));
  }
}

// =============================================================================
// BezierPrimitive (cubic)
// =============================================================================
namespace {

Vec2 bezierEval(const Vec2& p0, const Vec2& p1,
                const Vec2& p2, const Vec2& p3, float t) {
  const float u  = 1.0f - t;
  const float b0 = u*u*u;
  const float b1 = 3*u*u*t;
  const float b2 = 3*u*t*t;
  const float b3 = t*t*t;
  return {b0*p0.x + b1*p1.x + b2*p2.x + b3*p3.x,
          b0*p0.y + b1*p1.y + b2*p2.y + b3*p3.y};
}

// First derivative: B'(t) = 3 (1-t)^2 (p1-p0) + 6 (1-t) t (p2-p1) + 3 t^2 (p3-p2)
Vec2 bezierDeriv(const Vec2& p0, const Vec2& p1,
                 const Vec2& p2, const Vec2& p3, float t) {
  const float u = 1.0f - t;
  const Vec2 a = scl(sub(p1, p0), 3*u*u);
  const Vec2 b = scl(sub(p2, p1), 6*u*t);
  const Vec2 c = scl(sub(p3, p2), 3*t*t);
  return add(add(a, b), c);
}

// Second derivative: B''(t) = 6 (1-t)(p2 - 2 p1 + p0) + 6 t (p3 - 2 p2 + p1)
Vec2 bezierDeriv2(const Vec2& p0, const Vec2& p1,
                  const Vec2& p2, const Vec2& p3, float t) {
  const Vec2 a = scl({p2.x - 2*p1.x + p0.x, p2.y - 2*p1.y + p0.y}, 6*(1-t));
  const Vec2 b = scl({p3.x - 2*p2.x + p1.x, p3.y - 2*p2.y + p1.y}, 6*t);
  return add(a, b);
}

}  // namespace

PathPoint BezierPrimitive::nearestPoint(float px, float py) const {
  const Vec2 p{px, py};

  // Step 1: 16-seed sweep finds the basin of attraction. Pure Newton starting
  // from t=0.5 lands in the wrong minimum on S-curves where two lobes of the
  // curve are roughly equidistant from p.
  constexpr int kSeeds   = 16;
  constexpr int kMaxIter = 8;
  constexpr float kTol   = 1e-7f;

  float best_t = 0.0f;
  float best_d2 = std::numeric_limits<float>::infinity();
  for (int i = 0; i <= kSeeds; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(kSeeds);
    const Vec2 q  = bezierEval(p0_, p1_, p2_, p3_, t);
    const Vec2 dv = sub(q, p);
    const float d2 = dot(dv, dv);
    if (d2 < best_d2) { best_d2 = d2; best_t = t; }
  }

  // Step 2: Newton refinement on f(t) = (B(t)-p) . B'(t) = 0.
  // f'(t) = |B'(t)|^2 + (B(t)-p) . B''(t).
  float t = best_t;
  for (int it = 0; it < kMaxIter; ++it) {
    const Vec2 b   = bezierEval (p0_, p1_, p2_, p3_, t);
    const Vec2 db  = bezierDeriv(p0_, p1_, p2_, p3_, t);
    const Vec2 d2b = bezierDeriv2(p0_, p1_, p2_, p3_, t);
    const Vec2 r   = sub(b, p);
    const float f  = dot(r, db);
    const float fp = dot(db, db) + dot(r, d2b);
    if (std::fabs(fp) < 1e-12f) break;
    const float dt = f / fp;
    t -= dt;
    t = std::clamp(t, 0.0f, 1.0f);
    if (std::fabs(dt) < kTol) break;
  }

  // Endpoint check: Newton can wander away from a true endpoint minimum.
  for (float te : {0.0f, 1.0f}) {
    const Vec2 b = bezierEval(p0_, p1_, p2_, p3_, te);
    const Vec2 r = sub(b, p);
    const float d2 = dot(r, r);
    if (d2 < best_d2 - 1e-9f) {
      best_d2 = d2;
      t = te;
    } else if (te == 1.0f) {
      const Vec2 bn = bezierEval(p0_, p1_, p2_, p3_, t);
      const Vec2 rn = sub(bn, p);
      if (dot(rn, rn) < best_d2) best_d2 = dot(rn, rn);
    }
  }

  const Vec2 pt = bezierEval (p0_, p1_, p2_, p3_, t);
  const Vec2 tg = norm(bezierDeriv(p0_, p1_, p2_, p3_, t));
  PathPoint out{};
  out.position = pt;
  out.tangent  = tg;
  out.distance = signedPerp(p, pt, tg);
  out.t        = t;
  return out;
}

void BezierPrimitive::sample(std::vector<Vec2>& out, int segments) const {
  if (segments < 1) segments = 1;
  for (int i = 0; i <= segments; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(segments);
    out.push_back(bezierEval(p0_, p1_, p2_, p3_, t));
  }
}

// =============================================================================
// ArcPrimitive
// =============================================================================
PathPoint ArcPrimitive::nearestPoint(float px, float py) const {
  const Vec2 p{px, py};
  const Vec2 d = sub(p, center_);
  const float L = len(d);

  // Angle of p relative to center, in [-pi, pi).
  const float a_p = std::atan2(d.y, d.x);

  // Total signed sweep — preserves the "increasing parameter" direction
  // even when end_rad < start_rad (CW sweep).
  const float sweep = end_rad_ - start_rad_;
  const float sweep_abs = std::fabs(sweep);
  const float sign = (sweep >= 0.0f) ? 1.0f : -1.0f;

  // Express a_p in the local frame [0, sweep_abs] along the sweep
  // direction. If outside, clamp to the nearer endpoint.
  float u = wrapPi(a_p - start_rad_) * sign;  // u in [-pi, pi]
  // Map u into [0, sweep_abs] with clamp; if u is in the "outside" sector,
  // pick whichever endpoint (0 or sweep_abs) is angularly closest.
  if (u < 0.0f || u > sweep_abs) {
    const float d_to_0     = std::fabs(wrapPi(u));
    const float d_to_sweep = std::fabs(wrapPi(u - sweep_abs));
    u = (d_to_0 <= d_to_sweep) ? 0.0f : sweep_abs;
  }

  // Convert u back to absolute angle.
  const float a_use = start_rad_ + sign * u;
  const Vec2 pt{center_.x + radius_ * std::cos(a_use),
                center_.y + radius_ * std::sin(a_use)};

  // Tangent in direction of increasing parameter: derivative of arc
  // parametrization s -> center + r * [cos(start + sign*s), sin(start + sign*s)]
  // is r * sign * [-sin, cos]. Normalize and drop the r.
  const Vec2 tg{-sign * std::sin(a_use), sign * std::cos(a_use)};

  PathPoint out{};
  out.position = pt;
  out.tangent  = tg;  // already unit length (|[-sin, cos]| = 1)
  out.distance = signedPerp(p, pt, tg);
  // t = fraction of the sweep used.
  out.t = (sweep_abs > 1e-9f) ? (u / sweep_abs) : 0.0f;
  // For very-close-to-center queries (L ~ 0), position picked above is on
  // the nearer endpoint — already correct via the clamp. Just guard L for
  // signed-distance sanity (uninvolved here since signedPerp is structural).
  (void)L;
  return out;
}

void ArcPrimitive::sample(std::vector<Vec2>& out, int segments) const {
  if (segments < 1) segments = 1;
  const float sweep = end_rad_ - start_rad_;
  for (int i = 0; i <= segments; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(segments);
    const float a = start_rad_ + sweep * t;
    out.push_back(Vec2{center_.x + radius_ * std::cos(a),
                       center_.y + radius_ * std::sin(a)});
  }
}

// =============================================================================
// Path
// =============================================================================
PathPoint Path::nearestPoint(float px, float py) const {
  PathPoint best{};
  float best_d2 = std::numeric_limits<float>::infinity();
  for (std::size_t i = 0; i < primitives_.size(); ++i) {
    PathPoint cand = primitives_[i]->nearestPoint(px, py);
    const float dx = cand.position.x - px;
    const float dy = cand.position.y - py;
    const float d2 = dx*dx + dy*dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = cand;
      best.primitive_index = i;
    }
  }
  return best;
}

std::vector<Vec2> Path::polyline(int segments_per_primitive) const {
  std::vector<Vec2> out;
  for (std::size_t i = 0; i < primitives_.size(); ++i) {
    const std::size_t before = out.size();
    primitives_[i]->sample(out, segments_per_primitive);
    // The first sample of every primitive after the first coincides with the
    // previous primitive's last sample (G0 continuity is assumed for all
    // trainer paths); drop that duplicated joint vertex.
    if (i > 0 && out.size() > before) {
      out.erase(out.begin() + static_cast<std::ptrdiff_t>(before));
    }
  }
  return out;
}

// =============================================================================
// Minimal JSON parser
// =============================================================================
//
// Just enough JSON for the path schema: objects, arrays, numbers, strings,
// true/false/null. No escape sequences beyond \" and \\ (we never emit any
// others in our own paths). Errors carry "line N col M: <reason>".

namespace {

struct JsonValue;
using JsonObject = std::vector<std::pair<std::string, std::shared_ptr<JsonValue>>>;
using JsonArray  = std::vector<std::shared_ptr<JsonValue>>;

struct JsonValue {
  enum Type { NUL, BOOL, NUM, STR, ARR, OBJ } type = NUL;
  bool        b = false;
  double      n = 0.0;
  std::string s;
  JsonArray   a;
  JsonObject  o;
};

class JsonParser {
 public:
  explicit JsonParser(const std::string& src) : src_(src) {}

  std::shared_ptr<JsonValue> parse(std::string* err) {
    try {
      skipWs();
      auto v = parseValue();
      skipWs();
      if (pos_ != src_.size()) fail("trailing garbage");
      return v;
    } catch (const std::runtime_error& e) {
      if (err) *err = e.what();
      return nullptr;
    }
  }

 private:
  const std::string& src_;
  std::size_t        pos_  = 0;
  std::size_t        line_ = 1;
  std::size_t        col_  = 1;

  [[noreturn]] void fail(const std::string& msg) {
    std::ostringstream e;
    e << "line " << line_ << " col " << col_ << ": " << msg;
    throw std::runtime_error(e.str());
  }

  char peek() {
    if (pos_ >= src_.size()) fail("unexpected end of input");
    return src_[pos_];
  }
  char get() {
    if (pos_ >= src_.size()) fail("unexpected end of input");
    char c = src_[pos_++];
    if (c == '\n') { ++line_; col_ = 1; } else { ++col_; }
    return c;
  }
  bool eof() const { return pos_ >= src_.size(); }

  void skipWs() {
    while (!eof()) {
      char c = src_[pos_];
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        get();
      } else if (c == '/' && pos_ + 1 < src_.size() && src_[pos_ + 1] == '/') {
        // Line comment (extension beyond strict JSON; pragmatic for hand-
        // edited path files).
        while (!eof() && src_[pos_] != '\n') get();
      } else {
        break;
      }
    }
  }

  void expect(char c) {
    skipWs();
    if (eof() || src_[pos_] != c) {
      std::ostringstream e;
      e << "expected '" << c << "'";
      fail(e.str());
    }
    get();
  }

  bool match(char c) {
    skipWs();
    if (!eof() && src_[pos_] == c) { get(); return true; }
    return false;
  }

  std::shared_ptr<JsonValue> parseValue() {
    skipWs();
    if (eof()) fail("expected value");
    char c = src_[pos_];
    if (c == '{') return parseObject();
    if (c == '[') return parseArray();
    if (c == '"') return parseString();
    if (c == 't' || c == 'f') return parseBool();
    if (c == 'n') return parseNull();
    if (c == '-' || (c >= '0' && c <= '9')) return parseNumber();
    fail(std::string("unexpected character '") + c + "'");
  }

  std::shared_ptr<JsonValue> parseObject() {
    auto v = std::make_shared<JsonValue>();
    v->type = JsonValue::OBJ;
    expect('{');
    skipWs();
    if (match('}')) return v;
    while (true) {
      skipWs();
      if (eof() || src_[pos_] != '"') fail("expected string key");
      auto key = parseString();
      expect(':');
      auto val = parseValue();
      v->o.emplace_back(key->s, val);
      skipWs();
      if (match(',')) continue;
      expect('}');
      break;
    }
    return v;
  }

  std::shared_ptr<JsonValue> parseArray() {
    auto v = std::make_shared<JsonValue>();
    v->type = JsonValue::ARR;
    expect('[');
    skipWs();
    if (match(']')) return v;
    while (true) {
      v->a.push_back(parseValue());
      skipWs();
      if (match(',')) continue;
      expect(']');
      break;
    }
    return v;
  }

  std::shared_ptr<JsonValue> parseString() {
    expect('"');
    std::string s;
    while (true) {
      if (eof()) fail("unterminated string");
      char c = get();
      if (c == '"') break;
      if (c == '\\') {
        if (eof()) fail("unterminated escape");
        char e = get();
        switch (e) {
          case '"':  s += '"';  break;
          case '\\': s += '\\'; break;
          case '/':  s += '/';  break;
          case 'n':  s += '\n'; break;
          case 't':  s += '\t'; break;
          case 'r':  s += '\r'; break;
          default:
            fail(std::string("unsupported escape '\\") + e + "'");
        }
      } else {
        s += c;
      }
    }
    auto v = std::make_shared<JsonValue>();
    v->type = JsonValue::STR;
    v->s    = std::move(s);
    return v;
  }

  std::shared_ptr<JsonValue> parseNumber() {
    const std::size_t start = pos_;
    if (!eof() && src_[pos_] == '-') get();
    bool any_digit = false;
    while (!eof() && src_[pos_] >= '0' && src_[pos_] <= '9') { get(); any_digit = true; }
    if (!eof() && src_[pos_] == '.') {
      get();
      while (!eof() && src_[pos_] >= '0' && src_[pos_] <= '9') { get(); any_digit = true; }
    }
    if (!eof() && (src_[pos_] == 'e' || src_[pos_] == 'E')) {
      get();
      if (!eof() && (src_[pos_] == '+' || src_[pos_] == '-')) get();
      while (!eof() && src_[pos_] >= '0' && src_[pos_] <= '9') get();
    }
    if (!any_digit) fail("malformed number");
    auto v = std::make_shared<JsonValue>();
    v->type = JsonValue::NUM;
    v->n    = std::strtod(src_.c_str() + start, nullptr);
    return v;
  }

  std::shared_ptr<JsonValue> parseBool() {
    if (src_.compare(pos_, 4, "true") == 0) {
      for (int i = 0; i < 4; ++i) get();
      auto v = std::make_shared<JsonValue>();
      v->type = JsonValue::BOOL;
      v->b = true;
      return v;
    }
    if (src_.compare(pos_, 5, "false") == 0) {
      for (int i = 0; i < 5; ++i) get();
      auto v = std::make_shared<JsonValue>();
      v->type = JsonValue::BOOL;
      v->b = false;
      return v;
    }
    fail("expected true/false");
  }

  std::shared_ptr<JsonValue> parseNull() {
    if (src_.compare(pos_, 4, "null") == 0) {
      for (int i = 0; i < 4; ++i) get();
      auto v = std::make_shared<JsonValue>();
      v->type = JsonValue::NUL;
      return v;
    }
    fail("expected null");
  }
};

// --- Schema mappers --------------------------------------------------------
const JsonValue* objGet(const JsonValue& obj, const std::string& key) {
  if (obj.type != JsonValue::OBJ) return nullptr;
  for (const auto& kv : obj.o) {
    if (kv.first == key) return kv.second.get();
  }
  return nullptr;
}

bool jsonAsVec2(const JsonValue* v, Vec2& out, std::string* err,
                const std::string& field) {
  if (!v || v->type != JsonValue::ARR || v->a.size() != 2 ||
      v->a[0]->type != JsonValue::NUM ||
      v->a[1]->type != JsonValue::NUM) {
    if (err) *err = field + ": expected [x, y] number pair";
    return false;
  }
  out.x = static_cast<float>(v->a[0]->n);
  out.y = static_cast<float>(v->a[1]->n);
  return true;
}

bool jsonAsNum(const JsonValue* v, float& out, std::string* err,
               const std::string& field) {
  if (!v || v->type != JsonValue::NUM) {
    if (err) *err = field + ": expected number";
    return false;
  }
  out = static_cast<float>(v->n);
  return true;
}

}  // namespace

std::unique_ptr<Path> PathLibrary::loadFromString(const std::string& json,
                                                  std::string* err) {
  std::string parse_err;
  auto root = JsonParser(json).parse(&parse_err);
  if (!root) {
    if (err) *err = parse_err;
    return nullptr;
  }
  if (root->type != JsonValue::OBJ) {
    if (err) *err = "root must be an object";
    return nullptr;
  }
  auto out = std::make_unique<Path>();

  if (const JsonValue* name = objGet(*root, "name")) {
    if (name->type == JsonValue::STR) out->setName(name->s);
  }

  const JsonValue* prims = objGet(*root, "primitives");
  if (!prims || prims->type != JsonValue::ARR) {
    if (err) *err = "missing or non-array field 'primitives'";
    return nullptr;
  }

  for (std::size_t i = 0; i < prims->a.size(); ++i) {
    const JsonValue& prim = *prims->a[i];
    if (prim.type != JsonValue::OBJ) {
      if (err) *err = "primitive " + std::to_string(i) + ": not an object";
      return nullptr;
    }
    const JsonValue* type = objGet(prim, "type");
    if (!type || type->type != JsonValue::STR) {
      if (err) *err = "primitive " + std::to_string(i) +
                      ": missing string field 'type'";
      return nullptr;
    }
    const std::string& t = type->s;
    const std::string ctx = "primitive " + std::to_string(i) + " (" + t + ")";

    if (t == "linear") {
      Vec2 p0, p1;
      if (!jsonAsVec2(objGet(prim, "p0"), p0, err, ctx + ".p0")) return nullptr;
      if (!jsonAsVec2(objGet(prim, "p1"), p1, err, ctx + ".p1")) return nullptr;
      out->addPrimitive(std::make_unique<LinearPrimitive>(p0, p1));
    } else if (t == "bezier") {
      Vec2 p0, p1, p2, p3;
      if (!jsonAsVec2(objGet(prim, "p0"), p0, err, ctx + ".p0")) return nullptr;
      if (!jsonAsVec2(objGet(prim, "p1"), p1, err, ctx + ".p1")) return nullptr;
      if (!jsonAsVec2(objGet(prim, "p2"), p2, err, ctx + ".p2")) return nullptr;
      if (!jsonAsVec2(objGet(prim, "p3"), p3, err, ctx + ".p3")) return nullptr;
      out->addPrimitive(std::make_unique<BezierPrimitive>(p0, p1, p2, p3));
    } else if (t == "arc") {
      Vec2 c; float r=0, s=0, e=0;
      if (!jsonAsVec2(objGet(prim, "center"),    c, err, ctx + ".center"))    return nullptr;
      if (!jsonAsNum (objGet(prim, "radius"),    r, err, ctx + ".radius"))    return nullptr;
      if (!jsonAsNum (objGet(prim, "start_rad"), s, err, ctx + ".start_rad")) return nullptr;
      if (!jsonAsNum (objGet(prim, "end_rad"),   e, err, ctx + ".end_rad"))   return nullptr;
      if (r <= 0.0f) {
        if (err) *err = ctx + ".radius: must be > 0";
        return nullptr;
      }
      // ArcPrimitive::nearestPoint wraps the query angle to [-pi, pi], so it only
      // resolves the nearest point correctly for sweeps up to a half turn. A
      // |sweep| > pi mis-clamps the far half of the arc to an endpoint, which on
      // a path would command a large wrong perpendicular force. Reject it at load
      // time and tell the author to split the arc (e.g. a full circle = two
      // pi-arcs). Epsilon admits an exact-pi half-circle.
      if (std::fabs(e - s) > kPi + 1e-4f) {
        if (err) *err = ctx + ": |end_rad - start_rad| must be <= pi "
                              "(split larger arcs into multiple primitives)";
        return nullptr;
      }
      out->addPrimitive(std::make_unique<ArcPrimitive>(c, r, s, e));
    } else {
      if (err) *err = ctx + ": unknown primitive type";
      return nullptr;
    }
  }

  return out;
}

std::unique_ptr<Path> PathLibrary::loadFromFile(const std::string& path,
                                                std::string* err) {
  std::ifstream f(path);
  if (!f) {
    if (err) *err = "could not open file: " + path;
    return nullptr;
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  return loadFromString(ss.str(), err);
}

}  // namespace path
}  // namespace welding
