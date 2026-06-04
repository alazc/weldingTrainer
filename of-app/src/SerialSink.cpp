// Single source of ofSerial dependency for the motor-out direction.

#include "MotorSink.h"
#include "ProtocolIO.h"

#include "ofSerial.h"

namespace welding {
namespace motor {

struct SerialSink::Impl {
  ofSerial* port = nullptr;
  // Per-message-type sequence counters. The Arduino-side validator
  // tracks seq per msg_type, so we mirror that here.
  uint8_t   seq_cmd = 0;
  uint8_t   seq_hb  = 0;
  uint8_t   seq_arm = 0;
};

SerialSink::SerialSink(ofSerial& port) : impl_(new Impl()) {
  impl_->port = &port;
}

SerialSink::~SerialSink() { delete impl_; }

void SerialSink::sendCmdDown(float fx_N, float fy_N,
                             uint8_t erm_pwm, uint8_t flags) {
  if (!impl_ || !impl_->port) return;
  uint8_t buf[32];
  const std::size_t n = protocol::buildCmdDown(
      impl_->seq_cmd++, fx_N, fy_N, erm_pwm, flags, buf, sizeof(buf));
  if (n > 0) impl_->port->writeBytes(buf, static_cast<int>(n));
}

void SerialSink::sendHeartbeat() {
  if (!impl_ || !impl_->port) return;
  uint8_t buf[16];
  const std::size_t n = protocol::buildHeartbeat(
      impl_->seq_hb++, buf, sizeof(buf));
  if (n > 0) impl_->port->writeBytes(buf, static_cast<int>(n));
}

void SerialSink::sendArm() {
  if (!impl_ || !impl_->port) return;
  uint8_t buf[16];
  const std::size_t n = protocol::buildArm(
      impl_->seq_arm++, buf, sizeof(buf));
  if (n > 0) impl_->port->writeBytes(buf, static_cast<int>(n));
}

}  // namespace motor
}  // namespace welding
