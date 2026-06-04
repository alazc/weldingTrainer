#pragma once

// CPU-side wrapper around the shared wire format defined in
// arduino/welding_common/protocol.h. Adds:
//
//   * streaming frame reassembly (USB CDC reads do not respect framing)
//   * typed encoders for the CPU->Arduino direction
//   * a typed decoder for STATE_UP (the only Arduino->CPU message with data)
//
// The reassembler is intentionally simple: a flat byte buffer that
// accumulates until it sees the next 0x00 delimiter, then attempts to
// decode. The 0x00 delimiter is also the resync point — if any earlier
// frame was malformed, the buffer is discarded on the next 0x00 and the
// stream auto-recovers.

#include "../../arduino/welding_common/protocol.h"

#include <cstddef>
#include <cstdint>
#include <functional>

namespace welding {
namespace protocol {

class FrameReassembler {
public:
  using FrameCallback =
      std::function<void(uint8_t msg_type, uint8_t seq,
                         const uint8_t* data, std::size_t len)>;

  void setCallback(FrameCallback cb) { cb_ = std::move(cb); }

  // Push received bytes from a serial read(). Fires the callback once per
  // valid frame recognized. Malformed frames are counted and dropped.
  void feed(const uint8_t* bytes, std::size_t n);

  std::size_t framesAccepted()  const { return frames_accepted_;  }
  std::size_t crcErrors()       const { return crc_errors_;       }
  std::size_t oversizeDrops()   const { return oversize_drops_;   }
  std::size_t resyncEvents()    const { return resync_events_;    }

  void reset();

private:
  static constexpr std::size_t kMaxFrame = 64;
  uint8_t buf_[kMaxFrame] = {};
  std::size_t buf_idx_ = 0;
  bool overflowed_ = false;
  FrameCallback cb_;
  std::size_t frames_accepted_ = 0;
  std::size_t crc_errors_      = 0;
  std::size_t oversize_drops_  = 0;
  std::size_t resync_events_   = 0;
};

// --- Typed encoders (CPU -> Arduino) ----------------------------------------
// Each returns total bytes written including the trailing 0x00 delimiter.
// out_cap should be at least 32.

std::size_t buildCmdDown(uint8_t seq, float fx_N, float fy_N,
                         uint8_t erm_pwm, uint8_t flags,
                         uint8_t* out, std::size_t out_cap);
std::size_t buildHeartbeat(uint8_t seq, uint8_t* out, std::size_t out_cap);
std::size_t buildArm(uint8_t seq, uint8_t* out, std::size_t out_cap);

// --- Typed decoder (Arduino -> CPU) -----------------------------------------
// Returns true on success.
bool decodeStateUp(const uint8_t* data, std::size_t len, StateUp& out);

}  // namespace protocol
}  // namespace welding
