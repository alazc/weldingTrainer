#include "SessionLog.h"

#include "Config.h"

#include <cstdio>
#include <fstream>
#include <utility>

namespace welding {
namespace log {

SessionLog::SessionLog(std::size_t capacity_rows)
    : capacity_(capacity_rows) {
  rows_.reserve(capacity_);
}

SessionLog::SessionLog(std::size_t capacity_rows, std::string flush_path)
    : capacity_(capacity_rows),
      flush_path_(std::move(flush_path)) {
  rows_.reserve(capacity_);
}

SessionLog::~SessionLog() {
  // RAII flush only if a path was set at construction AND we haven't already
  // drained the buffer via an explicit flush. The two-condition guard means:
  //   * The "I just want an in-memory buffer for tests" caller pays nothing.
  //   * The "ofApp owns one per session" caller is durable even if shutdown
  //     skipped the explicit flushToCsv() call (panic exit, exception).
  //   * Calling flushToCsv() explicitly then letting the dtor run is idempotent.
  if (!flush_path_.empty() && !already_flushed_ && rowCount() > 0) {
    flushToCsv(flush_path_);
  }
}

void SessionLog::append(double t_sec, const input::StateSnapshot& s) {
  // Decimate to ~100 Hz. The counter is atomic so observers (HUD) can read
  // a coherent value; the increment is single-producer so the modulus check
  // is race-free against the RX thread's own subsequent appends.
  const int n = decimate_count_.fetch_add(1, std::memory_order_relaxed);
  if (n % config::kLogDecimateN != 0) return;

  if (overflow_.load(std::memory_order_relaxed)) return;
  if (rows_.size() >= capacity_) {
    overflow_.store(true, std::memory_order_relaxed);
    return;
  }

  if (!have_t0_) {
    t0_sec_   = t_sec;
    have_t0_  = true;
  }

  LogRow r;
  r.t_sec  = t_sec - t0_sec_;
  r.x      = s.x;
  r.y      = s.y;
  r.vx     = s.vx;
  r.vy     = s.vy;
  r.status = s.status;
  rows_.push_back(r);
  row_count_.store(rows_.size(), std::memory_order_relaxed);
}

void SessionLog::clear() {
  rows_.clear();
  row_count_.store(0, std::memory_order_relaxed);
  decimate_count_.store(0, std::memory_order_relaxed);
  overflow_.store(false, std::memory_order_relaxed);
  have_t0_         = false;
  already_flushed_ = false;
}

bool SessionLog::flushToCsv(const std::string& path) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) return false;

  // Header must match the CSV format exactly — LogReplaySource's loader keys off it.
  out << "t_ms,x_mm,y_mm,vx_mms,vy_mms,status\n";

  // Per-row format: t_ms as integer (ms granularity is what 100 Hz buys us),
  // positions/velocities as %.3f / %.3f, status as integer. snprintf into a
  // stack buffer keeps the formatting out of the iostream locale path, which
  // is both faster and predictable across MSVC default locales.
  char buf[96];
  for (const auto& r : rows_) {
    const long long t_ms = static_cast<long long>(r.t_sec * 1000.0 + 0.5);
    const int n = std::snprintf(buf, sizeof(buf),
                                "%lld,%.3f,%.3f,%.3f,%.3f,%u\n",
                                t_ms, r.x, r.y, r.vx, r.vy,
                                static_cast<unsigned>(r.status));
    if (n <= 0 || n >= static_cast<int>(sizeof(buf))) return false;
    out.write(buf, n);
    if (!out) return false;
  }
  out.flush();
  if (!out) return false;

  already_flushed_ = true;
  return true;
}

}  // namespace log
}  // namespace welding
