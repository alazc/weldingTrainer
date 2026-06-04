#include "ShapeCatalog.h"

namespace welding {
namespace shape {

namespace {
// Returned by accessors when the catalog is empty, so callers never deref past
// the end. Static so the const& outlives the call.
const ShapeEntry& emptyEntry() {
  static const ShapeEntry kEmpty{};
  return kEmpty;
}
const std::string& emptyFile() {
  static const std::string kEmpty;
  return kEmpty;
}
}  // namespace

ShapeCatalog::ShapeCatalog()
    : entries_{
          {"Vertical line", "linear.json"},
          {"Line into arc", "line-arc.json"},
          {"Small circle",  "small-circle.json"},
      } {}

ShapeCatalog::ShapeCatalog(std::vector<ShapeEntry> entries)
    : entries_(std::move(entries)) {}

const ShapeEntry& ShapeCatalog::active() const {
  if (entries_.empty()) return emptyEntry();
  return entries_[active_];
}

const ShapeEntry& ShapeCatalog::at(std::size_t i) const {
  if (entries_.empty()) return emptyEntry();
  if (i >= entries_.size()) i = entries_.size() - 1;
  return entries_[i];
}

const std::string& ShapeCatalog::select(std::size_t i) {
  if (entries_.empty()) return emptyFile();
  if (i >= entries_.size()) i = entries_.size() - 1;
  active_ = i;
  return entries_[active_].json_file;
}

const std::string& ShapeCatalog::next() {
  if (entries_.empty()) return emptyFile();
  active_ = (active_ + 1) % entries_.size();
  return entries_[active_].json_file;
}

const std::string& ShapeCatalog::prev() {
  if (entries_.empty()) return emptyFile();
  active_ = (active_ + entries_.size() - 1) % entries_.size();
  return entries_[active_].json_file;
}

}  // namespace shape
}  // namespace welding
