#include "ProtocolIO.h"

#include <cstring>

namespace welding {
namespace protocol {

void FrameReassembler::reset() {
  buf_idx_ = 0;
  overflowed_ = false;
}

void FrameReassembler::feed(const uint8_t* bytes, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) {
    const uint8_t b = bytes[i];
    if (b == 0x00) {
      // End of frame.
      if (overflowed_) {
        ++oversize_drops_;
        buf_idx_ = 0;
        overflowed_ = false;
        ++resync_events_;
        continue;
      }
      if (buf_idx_ == 0) {
        // 0x00 with empty buffer = sync byte at stream start, or two
        // delimiters back-to-back. Either way, no frame to decode.
        continue;
      }
      uint8_t msg_type = 0, seq = 0;
      uint8_t data[64] = {};
      int data_len = parseFrame(buf_, buf_idx_,
                                msg_type, seq, data, sizeof(data));
      if (data_len < 0) {
        ++crc_errors_;
      } else {
        ++frames_accepted_;
        if (cb_) cb_(msg_type, seq, data,
                     static_cast<std::size_t>(data_len));
      }
      buf_idx_ = 0;
    } else {
      // Accumulate.
      if (buf_idx_ < kMaxFrame) {
        buf_[buf_idx_++] = b;
      } else {
        // Frame would exceed our max; mark and wait for the next 0x00 to
        // resync. We deliberately do NOT clear the buffer mid-frame — we
        // need a 0x00 to know where the next frame starts.
        overflowed_ = true;
      }
    }
  }
}

// --- Encoders --------------------------------------------------------------

std::size_t buildCmdDown(uint8_t seq, float fx_N, float fy_N,
                         uint8_t erm_pwm, uint8_t flags,
                         uint8_t* out, std::size_t out_cap) {
  CmdDown c{};
  c.fx_q15  = floatToQ15(fx_N, kForceMaxN);
  c.fy_q15  = floatToQ15(fy_N, kForceMaxN);
  c.erm_pwm = erm_pwm;
  c.flags   = flags;
  uint8_t data[sizeof(CmdDown)];
  std::memcpy(data, &c, sizeof(c));
  const std::size_t need = cobsEncodeBound(2 + sizeof(c) + 1) + 1;
  if (out_cap < need) return 0;
  return buildFrame(MSG_CMD_DOWN, seq, data, sizeof(data), out);
}

std::size_t buildHeartbeat(uint8_t seq, uint8_t* out, std::size_t out_cap) {
  const std::size_t need = cobsEncodeBound(3) + 1;
  if (out_cap < need) return 0;
  return buildFrame(MSG_HEARTBEAT, seq, nullptr, 0, out);
}

std::size_t buildArm(uint8_t seq, uint8_t* out, std::size_t out_cap) {
  const std::size_t need = cobsEncodeBound(3) + 1;
  if (out_cap < need) return 0;
  return buildFrame(MSG_ARM, seq, nullptr, 0, out);
}

// --- Decoder ---------------------------------------------------------------

bool decodeStateUp(const uint8_t* data, std::size_t len, StateUp& out) {
  if (len != sizeof(StateUp)) return false;
  std::memcpy(&out, data, sizeof(out));
  return true;
}

}  // namespace protocol
}  // namespace welding
