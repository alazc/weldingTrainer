// welding_secondary.ino -- secondary Hapkit (LEFT joint) of the primary/secondary
// pantograph. Thin node: reports theta_left to the main and drives the left
// motor with the PWM the main returns. No kinematics, no PC link, no ERM.
//
// Link: hardware I2C/TWI, this board is TARGET 0x08. A4=SDA, A5=SCL, shared GND,
// 4.7 kOhm pull-ups to +5V (ONE pair on the bus). The controller polls us: it reads
// a 5-byte SecondaryState (struct + crc8) and writes a 5-byte SecondaryCmd. No
// COBS -- I2C self-frames each transaction. Because the link is off D0/D1, the
// secondary's hardware Serial is now FREE: it can be flashed and run a Serial
// Monitor while the link runs.
//
// This board gains its FIRST async ISRs (TWI onReceive/onRequest). They are kept
// trivial -- onReceive copies raw bytes + sets a flag; onRequest writes a buffer
// loop() already published. loop() owns ALL multi-byte decode/encode and crosses
// the ISR boundary atomically (ATOMIC snapshot on RX; single-byte index flip on
// TX). We do NOT lean on CRC to paper over a torn frame.
//
// Bringup: build once with WELDING_JOG_MODE to verify the left motor/encoder
// sign (BRINGUP.md), set kEncLeftSign/kMotorLeftSign in config.h, then rebuild
// without it for normal operation.

#include <Arduino.h>
#include <Wire.h>
#include <util/atomic.h>

#include "kinematics.h"   // trig (pulled in by inner_loop.h)
#include "protocol.h"
#include "config.h"
#include "inner_loop.h"

using namespace welding;

// Compile-time polarity-bringup gate (mirrors the main). Uncomment to jog.
// #define WELDING_JOG_MODE

// Left-joint MR analog sensor: analogRead(A2) -> flip-unwrap -> fit. The
// secondary is a super-loop (no Timer ISR), so it reads + converts inline in loop().
inner::MrUnwrapper g_mrL;
float g_tare_offset_l = 0.0f;

// ----------------------------------------------------------------------------
// ISR <-> loop() shared state. Single-byte volatiles only at the boundary; all
// multi-byte (de)serialization happens in loop().
// ----------------------------------------------------------------------------

// RX path: onReceive copies the raw transaction bytes here and raises a flag.
// loop() ATOMIC-snapshots this before decoding, so a back-to-back write from the
// controller cannot overwrite the buffer mid-decode.
constexpr uint8_t kRxCap = 8;                 // >= kSecondaryCmdLen (5), with slack
volatile uint8_t  g_rx_buf[kRxCap];
volatile uint8_t  g_rx_len   = 0;
volatile uint8_t  g_rx_ready = 0;

// TX path: double-buffered SecondaryState. loop() fills the INACTIVE half then
// flips g_active (a single-byte store = atomic publish). onRequest reads only
// g_send_buf[g_active], so the controller never clocks out a torn frame.
volatile uint8_t g_send_buf[2][protocol::kSecondaryStateLen];
volatile uint8_t g_active = 0;

// loop()-owned command cache (decoded from a CRC-valid frame only).
int16_t  g_cmd_left_pwm = 0;
bool     g_enabled      = false;
uint32_t g_last_cmd_ms  = 0;

// ----------------------------------------------------------------------------
// TWI ISRs -- trivial by design. No decode, no millis(), no motor writes here.
// ----------------------------------------------------------------------------

void onReceiveISR(int n) {
  (void)n;
  uint8_t i = 0;
  while (Wire.available() && i < kRxCap) g_rx_buf[i++] = (uint8_t)Wire.read();
  while (Wire.available()) Wire.read();        // drain extra -> next frame clean
  g_rx_len   = i;
  g_rx_ready = 1;                              // single-byte flag write (atomic)
}

void onRequestISR() {
  // Send the published half only. const_cast drops the volatile qualifier for
  // the Wire.write(buf, len) overload; those bytes are stable because loop()
  // writes the OTHER half before flipping g_active.
  Wire.write(const_cast<uint8_t*>(g_send_buf[g_active]), protocol::kSecondaryStateLen);
}

// ----------------------------------------------------------------------------
// Motor output. The secondary owns its safety clamp (defense in depth -- never
// trusts the primary's PWM blindly) and applies its own per-board motor sign.
// ----------------------------------------------------------------------------

inline void leftMotorOff() { analogWrite(config::kPinMotPwm, 0); }

inline void driveLeftMotor(int16_t signed_pwm) {
  const inner::ClampResult r =
      inner::clampSignedPwm((float)signed_pwm * (float)config::kMotorLeftSign,
                            config::kPwmMaxClamp);
  digitalWrite(config::kPinMotDir, r.dir ? HIGH : LOW);
  analogWrite(config::kPinMotPwm, r.pwm);
}

// Build a SecondaryState into the inactive send half, then publish it atomically.
void publishSecondaryState(float theta_left) {
  protocol::SecondaryState s;
  s.theta_left_q15 = protocol::floatToQ15(theta_left, protocol::kAngleMaxRad);
#ifdef WELDING_JOG_MODE
  s.status = 0;                          // not ready while jogging
#else
  s.status = protocol::kSecondaryReady;   // normal build = bringup acknowledged
#endif
  s._pad = 0;
  const uint8_t next = g_active ? 0 : 1;                       // the inactive half
  protocol::packSecondaryState(s, const_cast<uint8_t*>(g_send_buf[next]));
  g_active = next;                                             // atomic publish
}

void setup() {
  pinMode(config::kPinMotPwm, OUTPUT);
  pinMode(config::kPinMotDir, OUTPUT);
  pinMode(config::kPinMrSensor, INPUT);
  leftMotorOff();

  // MR sensor boot tare: operator holds the end-effector at the centerline
  // (theta = pi/2) during power-on so the left joint tares to pi/2 (BRINGUP).
  {
    const int raw0 = analogRead(config::kPinMrSensor);
    g_mrL.prime(raw0);
    const long u0 = g_mrL.update(raw0, config::kMrFlipThreshold, config::kMrCountsPerRev);
    g_tare_offset_l = inner::mrTareOffsetRad(u0, config::kMrSlopeDegPerCount,
                                             config::kEncLeftSign, config::kThetaTareRefRad);
  }

  // Initialize the send buffer to NOT-ready (status=0) BEFORE registering the
  // ISRs, so the very first onRequest serves a valid-but-not-ready frame. The
  // controller holds its motors off until our first real publish from loop().
  {
    protocol::SecondaryState boot;
    boot.theta_left_q15 = 0;
    boot.status = 0;            // bit0 (ready) clear -> controller keeps force off
    boot._pad = 0;
    protocol::packSecondaryState(boot, const_cast<uint8_t*>(g_send_buf[0]));
    protocol::packSecondaryState(boot, const_cast<uint8_t*>(g_send_buf[1]));
    g_active = 0;
  }

  Serial.begin(115200);   // now FREE (link moved off D0/D1 to I2C) -- bench debug

  Wire.begin(config::kSecondaryI2cAddr);   // I2C target 0x08
  Wire.onReceive(onReceiveISR);
  Wire.onRequest(onRequestISR);

  g_last_cmd_ms = millis();

  // Boot self-test ping (saltation_tele pattern): verify the motor channel in
  // isolation before joining the boards.
  analogWrite(config::kPinMotPwm, config::kJogPwm);
  delay(80);
  leftMotorOff();
}

void loop() {
  // 1. read local (left) MR analog sensor -> theta_left (flip-unwrap + fit)
  const int  raw = analogRead(config::kPinMrSensor);
  const long u   = g_mrL.update(raw, config::kMrFlipThreshold, config::kMrCountsPerRev);
  const float theta_left =
      inner::mrCountsToThetaRad(u, config::kMrSlopeDegPerCount,
                                config::kEncLeftSign, g_tare_offset_l);

  // 2. publish state for the controller to clock out, rate-limited to ~100 Hz
  static uint32_t last_pub = 0;
  const uint32_t now = millis();
  if ((uint32_t)(now - last_pub) >= config::kLinkSendIntervalMs) {
    last_pub = now;
    publishSecondaryState(theta_left);
  }

  // 3. consume a received command if the ISR flagged one. ATOMIC-snapshot the
  // raw bytes first so onReceive cannot overwrite them mid-decode.
  if (g_rx_ready) {
    uint8_t local[kRxCap];
    uint8_t n;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      n = g_rx_len;
      for (uint8_t i = 0; i < n && i < kRxCap; ++i) local[i] = g_rx_buf[i];
      g_rx_ready = 0;
    }
    protocol::SecondaryCmd cmd;
    if (protocol::unpackSecondaryCmd(local, n, &cmd)) {
      g_cmd_left_pwm = cmd.left_pwm;
      g_enabled      = (cmd.flags & protocol::kSecondaryEnable) != 0;
      g_last_cmd_ms  = now;
    }
    // else: wrong-length / corrupt / torn -> ignore. We do NOT refresh
    // g_last_cmd_ms, so the watchdog below ages the stale command out to off.
  }

  // 4. drive, or fail safe
#ifdef WELDING_JOG_MODE
  // Polarity bringup: ignore commands, cycle the motor, watch the encoder.
  driveLeftMotor((int16_t)inner::jogModePwm(now, 0));
#else
  const bool link_stale =
      inner::watchdogStale(now, g_last_cmd_ms, config::kLinkWatchdogMs);
  if (!g_enabled || link_stale) {
    leftMotorOff();
  } else {
    driveLeftMotor(g_cmd_left_pwm);
  }
#endif
}
