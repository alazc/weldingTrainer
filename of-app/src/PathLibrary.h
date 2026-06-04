#pragma once

// PathLibrary owns the reference path the trainee is asked to trace. A path
// is an ordered list of analytic primitives (linear / cubic Bezier / arc).
// At runtime the controller asks `nearestPoint(x, y)` to get the closest
// point on the path along with its tangent direction; the controller then
// drives perpendicular force toward that point and modulates ERM by the
// tangential-velocity error.
//
// JSON schema (all coordinates in workspace mm, angles in radians):
//
//   {
//     "name": "S-curve demo",
//     "primitives": [
//       { "type": "linear", "p0": [10, 20], "p1": [120, 20] },
//       { "type": "bezier",
//         "p0": [120, 20], "p1": [180, 20],
//         "p2": [180, 80], "p3": [220, 80] },
//       { "type": "arc",
//         "center": [220, 120], "radius": 40,
//         "start_rad": -1.5707963, "end_rad": 1.5707963 }
//     ]
//   }
//
// nearestPoint() returns (position, unit tangent, primitive index, parameter
// within that primitive). The unit tangent points in the direction of
// increasing path parameter — for arcs that means in the direction from
// start_rad toward end_rad, which can be either CW or CCW.

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace welding {
namespace path {

struct Vec2 {
  float x = 0.0f, y = 0.0f;
};

// Result of a nearest-point query.
struct PathPoint {
  Vec2  position;         // closest point on the path, in workspace mm
  Vec2  tangent;          // unit tangent in direction of increasing parameter
  float distance = 0.0f;  // signed perpendicular distance (sign chosen so
                          // that +distance is on the LEFT of the tangent)
  std::size_t primitive_index = 0;
  float       t              = 0.0f;  // parameter within that primitive [0,1]
                                      // (for arc: fraction of angular sweep)
};

class PathPrimitive {
 public:
  virtual ~PathPrimitive() = default;
  // Closest point on this primitive to (px, py). Returns position + tangent +
  // parameter; the caller fills in primitive_index and signed distance.
  virtual PathPoint nearestPoint(float px, float py) const = 0;
  // Append `segments + 1` points sampled uniformly in t∈[0,1] to `out`
  // (for arc: across the angular sweep). `segments` is clamped to >= 1.
  // Used to draw the path as a polyline.
  virtual void sample(std::vector<Vec2>& out, int segments) const = 0;
};

class Path {
 public:
  PathPoint nearestPoint(float px, float py) const;

  // Concatenated polyline across all primitives, in workspace mm. The shared
  // joint vertex between consecutive primitives is emitted once. Empty path
  // → empty vector. `segments_per_primitive` is clamped to >= 1.
  // Assumes G0 continuity: the start of each primitive equals the end of the
  // previous one; non-continuous paths will have their first point dropped.
  std::vector<Vec2> polyline(int segments_per_primitive) const;

  const std::string& name() const { return name_; }
  std::size_t        size() const { return primitives_.size(); }
  bool               empty() const { return primitives_.empty(); }

  // Take ownership of an already-constructed primitive. Used by the loader
  // and by tests that build paths without going through JSON.
  void addPrimitive(std::unique_ptr<PathPrimitive> p) {
    primitives_.push_back(std::move(p));
  }
  void setName(std::string n) { name_ = std::move(n); }

 private:
  std::string                                 name_;
  std::vector<std::unique_ptr<PathPrimitive>> primitives_;
};

// --- Concrete primitives ---------------------------------------------------
class LinearPrimitive : public PathPrimitive {
 public:
  LinearPrimitive(Vec2 p0, Vec2 p1) : p0_(p0), p1_(p1) {}
  PathPoint nearestPoint(float px, float py) const override;
  void sample(std::vector<Vec2>& out, int segments) const override;
 private:
  Vec2 p0_, p1_;
};

class BezierPrimitive : public PathPrimitive {
 public:
  BezierPrimitive(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3)
      : p0_(p0), p1_(p1), p2_(p2), p3_(p3) {}
  PathPoint nearestPoint(float px, float py) const override;
  void sample(std::vector<Vec2>& out, int segments) const override;
 private:
  Vec2 p0_, p1_, p2_, p3_;
};

class ArcPrimitive : public PathPrimitive {
 public:
  ArcPrimitive(Vec2 center, float radius, float start_rad, float end_rad)
      : center_(center), radius_(radius),
        start_rad_(start_rad), end_rad_(end_rad) {}
  PathPoint nearestPoint(float px, float py) const override;
  void sample(std::vector<Vec2>& out, int segments) const override;
 private:
  Vec2  center_;
  float radius_;
  float start_rad_, end_rad_;
};

// --- Loader -----------------------------------------------------------------
// On failure, the unique_ptr is null and (if non-null) *err carries a
// "line N col M: <reason>" diagnostic.
class PathLibrary {
 public:
  static std::unique_ptr<Path> loadFromFile  (const std::string& path,
                                              std::string* err = nullptr);
  static std::unique_ptr<Path> loadFromString(const std::string& json,
                                              std::string* err = nullptr);
};

}  // namespace path
}  // namespace welding
