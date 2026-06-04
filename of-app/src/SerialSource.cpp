// This is the single file that brings ofSerial into the input layer.

#include "InputSource.h"
#include "ProtocolIO.h"

#include "ofSerial.h"

#include <cstring>

namespace welding {
namespace input {

struct SerialSource::Impl {
  ofSerial*                         port = nullptr;
  protocol::FrameReassembler        reass;
  StateSnapshot                     latest{};
  bool                              fresh = false;
  uint8_t                           rx_buf[256] = {};
};

SerialSource::SerialSource(ofSerial& port) : impl_(new Impl()) {
  impl_->port = &port;

  impl_->reass.setCallback(
      [this](uint8_t msg_type, uint8_t /*seq*/,
             const uint8_t* data, std::size_t len) {
        if (msg_type != protocol::MSG_STATE_UP) return;
        protocol::StateUp s{};
        if (!protocol::decodeStateUp(data, len, s)) return;
        impl_->latest.x  = protocol::q15ToFloat(s.x_q15,  protocol::kPosMaxMm);
        impl_->latest.y  = protocol::q15ToFloat(s.y_q15,  protocol::kPosMaxMm);
        impl_->latest.vx = protocol::q15ToFloat(s.vx_q15, protocol::kVelMaxMmS);
        impl_->latest.vy = protocol::q15ToFloat(s.vy_q15, protocol::kVelMaxMmS);
        impl_->latest.max_loop_us = s.max_loop_us;
        impl_->latest.status      = s.status;
        impl_->fresh = true;
      });
}

SerialSource::~SerialSource() { delete impl_; }

bool SerialSource::poll(double /*now_sec*/, StateSnapshot& out) {
  if (!impl_ || !impl_->port) return false;
  const int avail = impl_->port->available();
  if (avail > 0) {
    const int n = impl_->port->readBytes(
        impl_->rx_buf,
        std::min<int>(avail, static_cast<int>(sizeof(impl_->rx_buf))));
    if (n > 0) {
      impl_->reass.feed(impl_->rx_buf, static_cast<std::size_t>(n));
    }
  }
  if (!impl_->fresh) return false;
  out = impl_->latest;
  impl_->fresh = false;
  return true;
}

std::size_t SerialSource::framesAccepted() const {
  return impl_ ? impl_->reass.framesAccepted() : 0;
}
std::size_t SerialSource::crcErrors() const {
  return impl_ ? impl_->reass.crcErrors() : 0;
}

}  // namespace input
}  // namespace welding
