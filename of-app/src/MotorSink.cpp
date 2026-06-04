#include "MotorSink.h"

namespace welding {
namespace motor {

void NullSink::sendCmdDown(float fx_N, float fy_N,
                           uint8_t erm_pwm, uint8_t flags) {
  ++cmd_count_;
  last_fx_    = fx_N;
  last_fy_    = fy_N;
  last_erm_   = erm_pwm;
  last_flags_ = flags;
}

void NullSink::sendHeartbeat() { ++hb_count_; }
void NullSink::sendArm()       { ++arm_count_; }

}  // namespace motor
}  // namespace welding
