#ifndef WELDING_PROTOCOL_H
#define WELDING_PROTOCOL_H

// Wire protocol shared between the AVR firmware and the CPU openFrameworks
// app. Both sides include this header directly; the CPU side wraps it in
// ProtocolIO for stream reassembly.
//
// Frame:   [ COBS(payload + crc8) ] [ 0x00 ]
// Payload: [ msg_type:1 ] [ seq:1 ] [ data:N ]
// CRC8:    Computed over msg_type + seq + data; polynomial 0x07 (CCITT),
//          init 0x00, no reflection, no xor-out.
//
// The trailing 0x00 is the frame delimiter — COBS guarantees the encoded
// region contains no 0x00 bytes, so the receiver can always resync on the
// next 0x00 it sees. This is the "free resync" property of COBS framing.
//
// Q15 mapping (locked in plan):
//   Position: int16 ~ +/- 256 mm,   7.8125 um / LSB
//   Velocity: int16 ~ +/- 512 mm/s, 15.625 mm/s / LSB
//   Force:    int16 ~ +/- 10 N,     305 uN / LSB  (kForceMaxN below)
//
// The motor inner loop converts (fx_q15, fy_q15) -> Newtons -> joint torques
// via Jacobian transpose -> PWM. Force in Newtons keeps units consistent with
// the dynamics simulation on the CPU side. The exact +/-10 N range is a
// placeholder; tune to the motor's stall torque after the bringup measurement.

#include <stdint.h>
#include <stddef.h>
#include <string.h>   // memcpy — inter-board pack/unpack helpers

namespace welding {
namespace protocol {

// --- Message types -----------------------------------------------------------
// Single-byte tags; values picked away from 0x00 to make resync-on-zero
// unambiguous even if a corrupted frame leaks one type byte.

enum MsgType : uint8_t {
  MSG_STATE_UP  = 0x11,  // Arduino -> CPU, ~1 kHz
  MSG_CMD_DOWN  = 0x21,  // CPU -> Arduino, ~200 Hz
  MSG_HEARTBEAT = 0x31,  // CPU -> Arduino, ~50 Hz (independent of CMD_DOWN)
  MSG_ARM       = 0x41,  // CPU -> Arduino, on user re-arm after E-stop
  MSG_SECONDARY_STATE = 0x51,  // secondary -> main (inter-board): theta_left + status
  MSG_SECONDARY_CMD   = 0x61,  // main -> secondary (inter-board): left motor PWM + flags
};

// --- Q15 fixed-point conversion ---------------------------------------------

constexpr float kPosMaxMm   = 256.0f;
constexpr float kVelMaxMmS  = 512.0f;
constexpr float kForceMaxN  = 10.0f;
// Inter-board q15 angle scale. Covers the (0, pi) workspace with headroom;
// ~1.2e-4 rad / LSB. Only the primary<->secondary link uses this; the PC link is
// unchanged.
constexpr float kAngleMaxRad = 4.0f;
constexpr float kQ15Max     = 32767.0f;

inline int16_t floatToQ15(float v, float scale_max) {
  float r = v * (kQ15Max / scale_max);
  if (r >  kQ15Max) r =  kQ15Max;
  if (r < -kQ15Max) r = -kQ15Max;
  return (int16_t)r;
}

inline float q15ToFloat(int16_t q, float scale_max) {
  return (float)q * (scale_max / kQ15Max);
}

// --- Payload structs ---------------------------------------------------------
//
// On wire, these are packed little-endian byte sequences. AVR (atmega328p)
// and x86 are both little-endian, so a raw memcpy of the struct works on
// both — provided we silence padding. We do that by using only fields that
// are naturally aligned at their declared sizes.

struct StateUp {
  int16_t  x_q15;        // position x, q15 scaled by kPosMaxMm
  int16_t  y_q15;        // position y
  int16_t  vx_q15;       // velocity x, q15 scaled by kVelMaxMmS
  int16_t  vy_q15;       // velocity y
  uint16_t max_loop_us;  // peak inner-loop time over last 100 ms window
  uint8_t  status;       // bit 0: SAFE_LATCHED, bit 1: polarity_ok,
                         // bit 2: loop_overrun, bits 3-7: reserved
  uint8_t  _pad;         // explicit pad to keep size = 12 on both targets
};
static_assert(sizeof(StateUp) == 12, "StateUp wire size must be 12 bytes");

struct CmdDown {
  int16_t fx_q15;
  int16_t fy_q15;
  uint8_t erm_pwm;       // 0..255 ERM motor duty
  uint8_t flags;         // bit 0: ARM_PERMIT, bit 1: JOG_MODE, ...
};
static_assert(sizeof(CmdDown) == 6, "CmdDown wire size must be 6 bytes");

// --- Inter-board payloads (primary <-> secondary link) ------------------------
// Little-endian packed, like StateUp/CmdDown. The secondary is a thin node:
// it reports its joint angle and applies a signed PWM the primary computes.

struct SecondaryState {        // secondary -> main
  int16_t theta_left_q15;     // left joint angle, q15 scaled by kAngleMaxRad
  uint8_t status;             // bit0: secondary ready (non-JOG build); bit1: fault
  uint8_t _pad;               // explicit pad -> size 4 on both targets
};
static_assert(sizeof(SecondaryState) == 4, "SecondaryState wire size must be 4 bytes");

struct SecondaryCmd {          // main -> secondary
  int16_t left_pwm;           // signed PWM for the left motor (-255..255)
  uint8_t flags;              // bit0: ENABLE (secondary may drive its motor)
  uint8_t _pad;
};
static_assert(sizeof(SecondaryCmd) == 4, "SecondaryCmd wire size must be 4 bytes");

constexpr uint8_t kSecondaryReady  = 0x01;  // SecondaryState.status bit0
constexpr uint8_t kSecondaryFault  = 0x02;  // SecondaryState.status bit1
constexpr uint8_t kSecondaryEnable = 0x01;  // SecondaryCmd.flags bit0

// HEARTBEAT and ARM have no data — just msg_type + seq + crc.

// --- CRC8 (poly 0x07, init 0x00) --------------------------------------------
// Table-free implementation. ~8 cycles per byte on AVR. For STATE_UP at 1 kHz
// over ~14 bytes, this is ~100 cycles / iteration — negligible against the
// 16 000 cycles available per millisecond.

inline uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t c = 0;
  for (size_t i = 0; i < len; ++i) {
    c ^= data[i];
    for (int b = 0; b < 8; ++b) {
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
  }
  return c;
}

// --- COBS encode / decode ---------------------------------------------------
//
// Reference: Cheshire & Baker, "Consistent Overhead Byte Stuffing" (1999).
// In:  arbitrary bytes (may contain 0x00).
// Out: byte stream with NO 0x00 bytes, length <= in_len + (in_len/254) + 1.
//
// Encoder writes the encoded bytes; the caller appends the 0x00 delimiter.
// Decoder reads up to the first 0x00 (or in_len bytes, whichever comes
// first) and writes the decoded payload.

// Worst-case output size for a buffer of in_len bytes (NOT including the
// trailing 0x00 delimiter). Used to size send buffers.
inline size_t cobsEncodeBound(size_t in_len) {
  return in_len + (in_len / 254) + 1;
}

// Returns number of bytes written to `out`. Does not write the 0x00
// delimiter; the caller is responsible for that.
inline size_t cobsEncode(const uint8_t* in, size_t in_len, uint8_t* out) {
  size_t out_idx = 0;
  size_t code_idx = out_idx++;  // reserve slot for the first code byte
  uint8_t code = 1;
  for (size_t i = 0; i < in_len; ++i) {
    if (in[i] == 0) {
      out[code_idx] = code;
      code_idx = out_idx++;
      code = 1;
    } else {
      out[out_idx++] = in[i];
      ++code;
      if (code == 0xFF) {
        out[code_idx] = code;
        code_idx = out_idx++;
        code = 1;
      }
    }
  }
  out[code_idx] = code;
  return out_idx;
}

// Returns decoded length, or 0 on malformed input (run extends past buffer,
// or empty input). `in_len` must NOT include the trailing 0x00 delimiter.
inline size_t cobsDecode(const uint8_t* in, size_t in_len, uint8_t* out) {
  if (in_len == 0) return 0;
  size_t in_idx = 0;
  size_t out_idx = 0;
  while (in_idx < in_len) {
    uint8_t code = in[in_idx++];
    if (code == 0) return 0;  // illegal: 0x00 inside a COBS block
    // Copy (code - 1) data bytes verbatim.
    for (uint8_t k = 1; k < code; ++k) {
      if (in_idx >= in_len) return 0;  // truncated
      out[out_idx++] = in[in_idx++];
    }
    // Emit a 0x00 separator unless code was 0xFF (the "no zero in this run"
    // marker) or unless we're at end of input.
    if (code != 0xFF && in_idx < in_len) {
      out[out_idx++] = 0;
    }
  }
  return out_idx;
}

// --- Frame helpers -----------------------------------------------------------
//
// buildFrame: writes [COBS(type | seq | data | crc8)] + 0x00 to `out`.
// Returns total bytes written. `out` must have capacity
// cobsEncodeBound(2 + data_len + 1) + 1.
//
// parseFrame: takes a single frame (the COBS region, NOT including the 0x00
// delimiter). Validates CRC. On success returns the data length (may be 0
// for HEARTBEAT/ARM). On failure returns -1.

inline size_t buildFrame(uint8_t msg_type, uint8_t seq,
                         const uint8_t* data, size_t data_len,
                         uint8_t* out) {
  // Assemble the unencoded payload + CRC on the stack.
  uint8_t payload[64];
  if (data_len + 3 > sizeof(payload)) return 0;
  payload[0] = msg_type;
  payload[1] = seq;
  for (size_t i = 0; i < data_len; ++i) payload[2 + i] = data[i];
  payload[2 + data_len] = crc8(payload, 2 + data_len);
  const size_t n = cobsEncode(payload, 3 + data_len, out);
  out[n] = 0x00;
  return n + 1;
}

// Returns data length on success, -1 on any failure (malformed COBS,
// truncated payload, CRC mismatch).
inline int parseFrame(const uint8_t* frame, size_t frame_len,
                      uint8_t& msg_type, uint8_t& seq,
                      uint8_t* out_data, size_t out_cap) {
  uint8_t buf[64];
  const size_t n = cobsDecode(frame, frame_len, buf);
  if (n < 3) return -1;  // need at least type + seq + crc
  const uint8_t got_crc = buf[n - 1];
  const uint8_t want_crc = crc8(buf, n - 1);
  if (got_crc != want_crc) return -1;
  msg_type = buf[0];
  seq      = buf[1];
  const size_t data_len = n - 3;
  if (data_len > out_cap) return -1;
  for (size_t i = 0; i < data_len; ++i) out_data[i] = buf[2 + i];
  return (int)data_len;
}

// --- Inter-board I2C pack / unpack (NO COBS) --------------------------------
//
// The primary<->secondary link is hardware I2C/TWI, which self-frames each
// transaction — `requestFrom`/`endTransmission` define the byte boundaries — so
// the COBS delimiter framing the PC link needs is redundant here. The wire form
// is simply the little-endian struct followed by a crc8 over those struct bytes:
//
//   pack:   [ sizeof(T) struct bytes (memcpy) ][ crc8 over them ]   -> sizeof(T)+1
//   unpack: accept iff n == sizeof(T)+1 AND crc8(in, sizeof(T)) == in[sizeof(T)]
//
// `unpack*` returns false on a wrong length (short/long read) OR a CRC mismatch
// — that bool IS the controller's accept/reject safety decision (host-tested). A
// rejected frame must NOT refresh the caller's cache: the inter-board watchdog
// (kLinkWatchdogMs) then cuts force on the stale timestamp. memcpy + crc8 keeps
// the same wire convention as StateUp/CmdDown (NOT explicit byte-packing).

constexpr size_t kSecondaryStateLen = sizeof(SecondaryState) + 1;  // 4 + crc8 = 5
constexpr size_t kSecondaryCmdLen   = sizeof(SecondaryCmd)   + 1;  // 4 + crc8 = 5

inline size_t packSecondaryState(const SecondaryState& s, uint8_t* out) {
  memcpy(out, &s, sizeof(SecondaryState));
  out[sizeof(SecondaryState)] = crc8(out, sizeof(SecondaryState));
  return kSecondaryStateLen;
}

inline bool unpackSecondaryState(const uint8_t* in, size_t n, SecondaryState* s) {
  if (n != kSecondaryStateLen) return false;                       // wrong length
  if (crc8(in, sizeof(SecondaryState)) != in[sizeof(SecondaryState)]) return false;
  memcpy(s, in, sizeof(SecondaryState));
  return true;
}

inline size_t packSecondaryCmd(const SecondaryCmd& c, uint8_t* out) {
  memcpy(out, &c, sizeof(SecondaryCmd));
  out[sizeof(SecondaryCmd)] = crc8(out, sizeof(SecondaryCmd));
  return kSecondaryCmdLen;
}

inline bool unpackSecondaryCmd(const uint8_t* in, size_t n, SecondaryCmd* c) {
  if (n != kSecondaryCmdLen) return false;                         // wrong length
  if (crc8(in, sizeof(SecondaryCmd)) != in[sizeof(SecondaryCmd)]) return false;
  memcpy(c, in, sizeof(SecondaryCmd));
  return true;
}

}  // namespace protocol
}  // namespace welding

#endif  // WELDING_PROTOCOL_H
