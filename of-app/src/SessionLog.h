#pragma once

// In-RAM session log. The RX thread appends decimated StateSnapshots
// (1 kHz -> 100 Hz) into a pre-allocated vector; on session end a flush
// writes them to a CSV. On overflow we set a flag
// rather than reallocate (a real-time thread doing malloc would blow
// every downstream deadline).
//
// Construction modes:
//   * SessionLog(capacity) — pure in-memory accumulator; destructor does
//                            not flush. Use this for tests or any caller
//                            that controls its own persistence story.
//   * SessionLog(capacity, flush_path) — RAII-guarded: destructor flushes
//                            to that path if at least one row was appended
//                            and no prior flush call already drained the
//                            buffer. This is the "ofApp owns one of these
//                            per session" idiom.
//
// Threading model: append() is intended for a single RX-thread caller;
// rowCount() and overflow() use atomics so any thread (HUD, main) can
// observe progress safely. flushToCsv() must be called from a thread
// that is NOT racing append() — typically after ControlThreads::stop().

#include "InputSource.h"

#include <atomic>
#include <cstddef>
#include <string>
#include <vector>

namespace welding {
namespace log {

struct LogRow {
  double  t_sec   = 0.0;
  float   x       = 0.0f;
  float   y       = 0.0f;
  float   vx      = 0.0f;
  float   vy      = 0.0f;
  uint8_t status  = 0;
};

class SessionLog {
 public:
  // Pure in-memory mode.
  explicit SessionLog(std::size_t capacity_rows);
  // RAII-flush mode.
  SessionLog(std::size_t capacity_rows, std::string flush_path);
  ~SessionLog();

  SessionLog(const SessionLog&)            = delete;
  SessionLog& operator=(const SessionLog&) = delete;

  // Called from the RX thread (1 kHz). Internally decimates by
  // config::kLogDecimateN so we land at ~100 Hz of stored rows.
  // No-op once overflow has tripped. `t_sec` is the caller's monotonic
  // wall-clock; the writer rebases to t_sec_at_first_append in the CSV.
  void append(double t_sec, const input::StateSnapshot& s);

  // Approximate (relaxed) row count. Safe to call from any thread.
  std::size_t rowCount() const {
    return row_count_.load(std::memory_order_relaxed);
  }
  bool overflow() const {
    return overflow_.load(std::memory_order_relaxed);
  }
  std::size_t capacity() const { return capacity_; }

  // Reset decimation counter and trash any accumulated rows. Test helper —
  // not meant for live use (clearing the vector mid-session would race the
  // RX thread).
  void clear();

  // Write the current rows to a CSV in the format:
  //   t_ms,x_mm,y_mm,vx_mms,vy_mms,status
  //   0,100.000,...
  // Returns true on success. Marks the log as flushed so the RAII destructor
  // does not double-write.
  bool flushToCsv(const std::string& path);

  // Direct read of the row buffer (test/HUD inspection). NOT thread-safe
  // vs concurrent append() — caller is responsible.
  const std::vector<LogRow>& rows() const { return rows_; }

 private:
  std::vector<LogRow>      rows_;
  std::size_t              capacity_       = 0;
  std::atomic<int>         decimate_count_{0};
  std::atomic<std::size_t> row_count_     {0};
  std::atomic<bool>        overflow_      {false};

  std::string              flush_path_;
  bool                     already_flushed_ = false;
  bool                     have_t0_         = false;
  double                   t0_sec_          = 0.0;
};

}  // namespace log
}  // namespace welding
