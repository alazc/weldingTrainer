#include "InputSource.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace welding {
namespace input {

// =============================================================================
// MouseSource
// =============================================================================

void MouseSource::inject(double now_sec, float x_mm, float y_mm) {
  ++inject_count_;
  if (!has_prev_) {
    has_prev_ = true;
    prev_t_   = now_sec;
    prev_x_   = x_mm;
    prev_y_   = y_mm;
    // First sample: report position, no velocity. We deliberately do not
    // emit a snapshot on the first inject — we need a second point before
    // velocity is meaningful. The consumer's first `poll` returns false.
    return;
  }
  const double dt = now_sec - prev_t_;
  if (dt > 1e-6) {
    latest_.x  = x_mm;
    latest_.y  = y_mm;
    latest_.vx = static_cast<float>((x_mm - prev_x_) / dt);
    latest_.vy = static_cast<float>((y_mm - prev_y_) / dt);
    latest_.max_loop_us = 0;
    latest_.status      = 0;
    fresh_ = true;
  }
  prev_t_ = now_sec;
  prev_x_ = x_mm;
  prev_y_ = y_mm;
}

bool MouseSource::poll(double /*now_sec*/, StateSnapshot& out) {
  if (!fresh_) return false;
  out = latest_;
  fresh_ = false;
  return true;
}

// =============================================================================
// LogReplaySource
// =============================================================================

namespace {

// Tiny CSV row split. Trims surrounding whitespace and splits on commas.
// Returns the number of cells. Embedded quotes are not supported (we control
// the writer, and the format is fixed numeric).
std::size_t splitCsv(const std::string& line, std::vector<std::string>& out) {
  out.clear();
  std::string cell;
  cell.reserve(16);
  for (char c : line) {
    if (c == ',') {
      out.push_back(cell);
      cell.clear();
    } else if (c != '\r') {
      cell.push_back(c);
    }
  }
  out.push_back(cell);
  // Trim each cell.
  for (auto& s : out) {
    std::size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    s = s.substr(a, b - a);
  }
  return out.size();
}

bool parseFloat(const std::string& s, float& out) {
  if (s.empty()) return false;
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str()) return false;
  // Allow trailing whitespace only (already trimmed by splitCsv, defensive).
  while (*end != '\0') {
    if (!std::isspace(static_cast<unsigned char>(*end))) return false;
    ++end;
  }
  out = static_cast<float>(v);
  return true;
}

bool parseUint8(const std::string& s, uint8_t& out) {
  if (s.empty()) return false;
  char* end = nullptr;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (end == s.c_str()) return false;
  if (v < 0 || v > 255) return false;
  while (*end != '\0') {
    if (!std::isspace(static_cast<unsigned char>(*end))) return false;
    ++end;
  }
  out = static_cast<uint8_t>(v);
  return true;
}

}  // namespace

bool LogReplaySource::loadFromString(const std::string& csv,
                                     std::string* err) {
  rows_.clear();
  next_idx_  = 0;
  t0_        = -1.0;
  exhausted_ = false;
  ever_emitted_ = false;

  std::istringstream in(csv);
  std::string        line;
  std::size_t        lineno = 0;
  bool               saw_header = false;
  double             t0_ms = 0.0;

  while (std::getline(in, line)) {
    ++lineno;
    // Skip blank lines.
    bool only_ws = true;
    for (char c : line) {
      if (!std::isspace(static_cast<unsigned char>(c))) { only_ws = false; break; }
    }
    if (only_ws) continue;

    std::vector<std::string> cells;
    splitCsv(line, cells);

    if (!saw_header) {
      // Header sanity-check. We do not require exact column names, but the
      // shape (6 cells, first cell looks like "t_ms") catches the most common
      // misuse: passing a binary log or wrong CSV.
      if (cells.size() != 6 || cells[0] != "t_ms") {
        if (err) {
          std::ostringstream e;
          e << "line " << lineno
            << ": expected header 't_ms,x_mm,y_mm,vx_mms,vy_mms,status' (got "
            << cells.size() << " cells)";
          *err = e.str();
        }
        rows_.clear();
        return false;
      }
      saw_header = true;
      continue;
    }

    if (cells.size() != 6) {
      if (err) {
        std::ostringstream e;
        e << "line " << lineno << ": expected 6 cells, got " << cells.size();
        *err = e.str();
      }
      rows_.clear();
      return false;
    }

    float   t_ms = 0.0f;
    Row     r{};
    uint8_t status = 0;
    if (!parseFloat (cells[0], t_ms)   ||
        !parseFloat (cells[1], r.x)    ||
        !parseFloat (cells[2], r.y)    ||
        !parseFloat (cells[3], r.vx)   ||
        !parseFloat (cells[4], r.vy)   ||
        !parseUint8 (cells[5], status)) {
      if (err) {
        std::ostringstream e;
        e << "line " << lineno << ": malformed numeric field";
        *err = e.str();
      }
      rows_.clear();
      return false;
    }
    r.status = status;

    // Rebase t_ms to start at 0 (per CSV spec, first row is t=0, but we
    // tolerate writers that emit absolute timestamps).
    if (rows_.empty()) t0_ms = t_ms;
    r.t_sec = (t_ms - t0_ms) / 1000.0;

    // Monotonicity: t must be non-decreasing.
    if (!rows_.empty() && r.t_sec < rows_.back().t_sec - 1e-9) {
      if (err) {
        std::ostringstream e;
        e << "line " << lineno << ": t_ms is non-monotonic";
        *err = e.str();
      }
      rows_.clear();
      return false;
    }

    rows_.push_back(r);
  }

  if (!saw_header) {
    if (err) *err = "empty input (no header)";
    return false;
  }
  // Allow zero data rows — the source simply exhausts on first poll.
  return true;
}

bool LogReplaySource::loadFromFile(const std::string& path,
                                   std::string* err) {
  std::ifstream f(path);
  if (!f) {
    if (err) *err = "could not open file: " + path;
    return false;
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  return loadFromString(ss.str(), err);
}

void LogReplaySource::reset() {
  next_idx_     = 0;
  t0_           = -1.0;
  exhausted_    = false;
  ever_emitted_ = false;
}

bool LogReplaySource::poll(double now_sec, StateSnapshot& out) {
  if (rows_.empty()) {
    exhausted_ = true;
    return false;
  }
  if (t0_ < 0.0) {
    t0_ = now_sec;
  }
  // 1 us tolerance absorbs the inevitable double-precision drift in
  // (now_sec - t0_): e.g. 5.1 - 5.0 evaluates to 0.09999...4, which would
  // otherwise spuriously hold back a row scheduled for 0.1 s.
  const double elapsed = (now_sec - t0_) + 1e-6;

  // Advance through every row whose timestamp is <= elapsed. The most
  // recent such row is the one we emit. If no row passes the bar, the
  // consumer gets nothing this tick.
  bool advanced = false;
  while (next_idx_ < rows_.size() && rows_[next_idx_].t_sec <= elapsed) {
    ++next_idx_;
    advanced = true;
  }
  if (!advanced && ever_emitted_) {
    return false;  // no new sample since last poll
  }
  if (next_idx_ == 0) {
    return false;  // not yet time for the first sample
  }

  const Row& r = rows_[next_idx_ - 1];
  out.x  = r.x;
  out.y  = r.y;
  out.vx = r.vx;
  out.vy = r.vy;
  out.max_loop_us = 0;
  out.status      = r.status;
  ever_emitted_   = true;

  if (next_idx_ >= rows_.size()) {
    exhausted_ = true;
  }
  return true;
}

}  // namespace input
}  // namespace welding
