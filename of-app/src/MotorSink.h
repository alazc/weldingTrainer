#pragma once

// IMotorSink is the symmetric seam to IInputSource: it accepts engineering-
// unit commands from the controller and is responsible for putting them on
// the wire. Two impls:
//
//   * SerialSink — encodes CMD_DOWN / HEARTBEAT / ARM frames and writes
//                  them to an ofSerial port. ofSerial is forward-declared,
//                  impl lives in SerialSink.cpp and is not in the host
//                  test build.
//   * NullSink   — discards everything, but records per-method call counts
//                  and the last payload. Used by demo mode (--source=mouse)
//                  where there is no Arduino to talk to, and by host tests
//                  to verify the controller's CMD_DOWN cadence and values.
//
// Sequence-number bookkeeping is the sink's responsibility — each send-method
// auto-increments its own per-message-type counter. This matches how the
// Arduino side validates seq order (per msg_type), not globally.

#include <cstddef>
#include <cstdint>

class ofSerial;  // production-only; impl in SerialSink.cpp

namespace welding {
namespace motor {

class IMotorSink {
 public:
  virtual ~IMotorSink() = default;

  // Force in Newtons; ERM duty as 0..255; flags is the protocol::CmdDown
  // flag byte (bit 0 = ARM_PERMIT, bit 1 = JOG_MODE, ...).
  virtual void sendCmdDown(float fx_N, float fy_N,
                           uint8_t erm_pwm, uint8_t flags) = 0;
  virtual void sendHeartbeat() = 0;
  virtual void sendArm()       = 0;
};

// --- NullSink ---------------------------------------------------------------
// Records call counts and the last payload of each kind so tests can assert
// without round-tripping through a serial port.
class NullSink : public IMotorSink {
 public:
  void sendCmdDown(float fx_N, float fy_N,
                   uint8_t erm_pwm, uint8_t flags) override;
  void sendHeartbeat() override;
  void sendArm() override;

  std::size_t cmdCount()       const { return cmd_count_; }
  std::size_t heartbeatCount() const { return hb_count_;  }
  std::size_t armCount()       const { return arm_count_; }

  // Snapshot of the last CMD_DOWN payload (for test assertions). Returns
  // sentinel zeros if no CMD_DOWN has been sent yet.
  float   lastFx()      const { return last_fx_; }
  float   lastFy()      const { return last_fy_; }
  uint8_t lastErmPwm()  const { return last_erm_; }
  uint8_t lastFlags()   const { return last_flags_; }

 private:
  std::size_t cmd_count_ = 0;
  std::size_t hb_count_  = 0;
  std::size_t arm_count_ = 0;
  float   last_fx_    = 0.0f;
  float   last_fy_    = 0.0f;
  uint8_t last_erm_   = 0;
  uint8_t last_flags_ = 0;
};

// --- SerialSink -------------------------------------------------------------
class SerialSink : public IMotorSink {
 public:
  explicit SerialSink(ofSerial& port);
  ~SerialSink() override;

  void sendCmdDown(float fx_N, float fy_N,
                   uint8_t erm_pwm, uint8_t flags) override;
  void sendHeartbeat() override;
  void sendArm()       override;

 private:
  struct Impl;
  Impl* impl_;
};

}  // namespace motor
}  // namespace welding
