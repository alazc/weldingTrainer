#pragma once

// ShapeCatalog — the trainer's selectable "shape library": an ordered list of
// reference-path shapes plus which one is active. Pure logic with zero
// openFrameworks dependency (mirrors the model/draw split: PathLibrary / RenderModel
// are engineering-unit code host-tested with cl.exe; the oF layer is thin).
//
// The catalog deliberately holds only DISPLAY NAME + JSON FILENAME, never opens a
// file. File I/O needs ofToDataPath() (oF-only), so ofApp owns one catalog
// and does the load itself in its setActivePath() choke point. Keeping the
// catalog oF-free is what lets "select shape 3", "cycle next", and "every shipped
// shape loads and fits the workspace" be unit tests.
//
// Default order matches the UI icon strip, left to right:
//   0 vertical line  1 line-into-arc  2 small circle

#include <cstddef>
#include <string>
#include <vector>

namespace welding {
namespace shape {

struct ShapeEntry {
  std::string display_name;  // HUD/tooltip label
  std::string json_file;     // filename only (e.g. "linear.json"); resolved via
                             // ofToDataPath("paths/" + json_file) at load time
};

class ShapeCatalog {
 public:
  // Default catalog: the three shipped shapes in icon-strip order.
  ShapeCatalog();
  // Explicit list (used by tests to build small fixtures).
  explicit ShapeCatalog(std::vector<ShapeEntry> entries);

  std::size_t       size()        const { return entries_.size(); }
  bool              empty()       const { return entries_.empty(); }
  std::size_t       activeIndex() const { return active_; }

  // Active / indexed entry. On an empty catalog active() returns a static empty
  // entry (no UB); at() clamps the index into range.
  const ShapeEntry& active() const;
  const ShapeEntry& at(std::size_t i) const;

  // Set the active shape. `i` is clamped into [0, size()-1]. Returns the active
  // entry's json_file (empty string if the catalog is empty). No-op on empty.
  const std::string& select(std::size_t i);

  // Advance / retreat the active index, wrapping at the ends. Returns the active
  // entry's json_file. No-op (empty string) on an empty catalog.
  const std::string& next();
  const std::string& prev();

 private:
  std::vector<ShapeEntry> entries_;
  std::size_t             active_ = 0;
};

}  // namespace shape
}  // namespace welding
