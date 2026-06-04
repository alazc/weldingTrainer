// welding_trainer.ino  --  inner loop on Arduino Uno.
//
// Responsibilities of this translation unit (everything else lives in the
// header library so the host build can test it):
//   * encoder ISR wiring via Encoder.h
//   * Timer1 CTC @ 1 kHz drives the ISR-resident inner control loop
//   * watchdog: SAFE_LATCHED when the CPU's HEARTBEAT goes stale
//   * JOG_MODE polarity bringup (compile-time gate in config.h)
//   * main loop polls Serial, parses CMD_DOWN / HEARTBEAT / ARM,
//     emits STATE_UP frames, rolls the loop-time window
//   * primary of the two-board split: exchanges theta_left / left-motor PWM with
//     the secondary as I2C CONTROLLER on A4/A5 (config.h); the one hardware UART is
//     owned by the PC link, so TWI is the only free hardware serial peripheral
//
// Pin / timer allocation is documented in config.h. Read that header first
// if you are debugging why a motor will not move.

#include <Arduino.h>
#include <Wire.h>
#include <util/atomic.h>

#include "kinematics.h"
#include "protocol.h"
#include "config.h"
#include "inner_loop.h"

using namespace welding;

// ----------------------------------------------------------------------------
// Local (right) MR analog sensor + the secondary link. The right joint is read
// via analogRead(A2) in loop() (the ~104 us conversion must NOT sit in the
// 1 kHz ISR), flip-unwrapped + fitted, and cached in g_theta_right — the
// SAME pattern theta_left already uses, so the ISR reads BOTH joints from a
// cache and does zero sensor I/O. The secondary link is hardware I2C (this board
// is the controller, A4=SDA/A5=SCL); the PC owns the hardware UART.
// ----------------------------------------------------------------------------

inner::MrUnwrapper g_mrR;          // right-joint flip-unwrap state (loop() only)
float g_tare_offset_r = 0.0f;      // boot-tare offset (loop() only)

// ----------------------------------------------------------------------------
// State shared between the Timer1 ISR (inner loop) and the main loop
// (serial RX/TX, window rollover). All shared scalars are volatile; reads
// of multi-byte values from the main loop go through ATOMIC_BLOCK because
// the AVR cannot read a uint32_t or a 4-byte float atomically.
// ----------------------------------------------------------------------------

volatile float    g_x_mm = 0.0f, g_y_mm = 0.0f;
volatile float    g_vx_mms = 0.0f, g_vy_mms = 0.0f;
volatile uint16_t g_max_loop_window_us = 0;  // peak this 100 ms
volatile uint16_t g_max_loop_us = 0;         // peak the previous window
volatile uint8_t  g_status = 0;
volatile bool     g_tx_due = false;
volatile uint8_t  g_tx_seq = 0;

// CMD_DOWN snapshot (set by main loop, read by ISR).
volatile float    g_cmd_fx_n = 0.0f;
volatile float    g_cmd_fy_n = 0.0f;
volatile uint8_t  g_cmd_erm  = 0;
volatile uint32_t g_last_heartbeat_ms = 0;

// Polarity-ok latch lives in RAM, not EEPROM, in v1. Rationale: every fresh
// power-up should re-run JOG_MODE before closed-loop force engages. EEPROM
// persistence is captured as a follow-up in BRINGUP.md.
volatile bool g_polarity_ok = false;

// --- Sensor cache (written by loop() under ATOMIC_BLOCK, read by the ISR) ----
volatile float    g_theta_right     = 0.0f;  // radians, local MR sensor

// --- Inter-board cache (written by loop(), read by the ISR) ------------------
volatile float    g_theta_left      = 0.0f;  // radians, decoded from secondary
volatile uint8_t  g_secondary_status = 0;     // cached SecondaryState.status
volatile uint32_t g_last_secondary_ms = 0;    // inter-board watchdog
volatile int16_t  g_left_pwm_cmd    = 0;     // left motor PWM to send (set by ISR)
volatile bool     g_link_enable     = false; // ENABLE bit to send (set by ISR)

// ----------------------------------------------------------------------------
// Timer1 CTC setup: prescaler 8, OCR1A = 1999 -> ISR every 1 ms. The compare
// match interrupt vector runs welding_innerLoop(). We pick Timer1 instead of
// Timer2 so that Timer2 stays available for D11 PWM (the ERM driver) and
// Timer0 stays available for millis() / D5 PWM.
// ----------------------------------------------------------------------------

void timer1InnerLoopBegin() {
  noInterrupts();
  TCCR1A = 0;                              // no OC1A/OC1B pin coupling
  TCCR1B = (1 << WGM12) | (1 << CS11);     // CTC, prescaler 8
  OCR1A  = config::kTimer1OcrFor1kHz;
  TIMSK1 = (1 << OCIE1A);
  TCNT1  = 0;
  interrupts();
}

// ----------------------------------------------------------------------------
// Motor / ERM output helpers. Keep these tiny — they are called from the
// inner-loop ISR every millisecond.
// ----------------------------------------------------------------------------

inline void writeMotor(uint8_t pwm_pin, uint8_t dir_pin,
                       int8_t sign, float signed_pwm) {
  // Per-motor sign flip lets us correct DIR-pin polarity post-bringup
  // without rewiring. Multiply BEFORE the clamp so a negative sign on a
  // positive command still saturates symmetrically.
  signed_pwm *= (float)sign;
  const inner::ClampResult r =
      inner::clampSignedPwm(signed_pwm, config::kPwmMaxClamp);
  digitalWrite(dir_pin, r.dir ? HIGH : LOW);
  analogWrite(pwm_pin, r.pwm);
}

inline void motorsOff() {
  analogWrite(config::kPinMotPwm, 0);  // local (right) motor
  analogWrite(config::kPinErmPwm, 0);
}

// ----------------------------------------------------------------------------
// Inner control loop. Called from ISR at 1 kHz.
//
// Sequence:
//   1. read encoders -> joint angles
//   2. fwd kinematics -> (x, y), finite-diff velocity
//   3. watchdog check; latch SAFE on stale heartbeat
//   4. JOG_MODE OR (closed-loop force via Jacobian transpose)
//   5. write PWM
//   6. update loop-time max for the current window
//   7. signal main loop that a STATE_UP frame is due
// ----------------------------------------------------------------------------

void innerLoop() {
  const uint32_t t0 = micros();

  // --- 1. angles: both joints read from caches (no sensor I/O in the ISR) ----
  const float theta2 = g_theta_right;  // right joint (doc theta5), local MR sensor
  const float theta1 = g_theta_left;   // left joint (doc theta1), from secondary.
  // Both are written by loop() under ATOMIC_BLOCK, so these ISR reads are
  // consistent (the ISR cannot itself be interrupted mid-read). The ~104 us
  // analogRead for theta2 lives in loop(), off the 1 kHz budget.

  // --- 2. fwd kinematics + windowed finite-diff velocity -------------------
  // Velocity is differenced over a kVelWindowTicks window (not one 1 kHz tick)
  // so the MR position quantization does not blow up into velocity noise — the
  // fix MUST be here, the PC only sees ~60 Hz frames (see config::kVelWindowTicks
  // / inner::VelocityEstimator). last_x/last_y still hold the last good pose so a
  // failed fwdKin leaves (x,y) frozen rather than snapping.
  static inner::VelocityEstimator<config::kVelWindowTicks> vel_est;
  static float last_x = 0.0f, last_y = 0.0f;
  float x = last_x, y = last_y;
  const bool reach_ok = fwdKin(theta1, theta2, x, y);
  if (reach_ok) {
    float vx = 0.0f, vy = 0.0f;
    vel_est.update(x, y, (float)config::kInnerLoopHz, vx, vy);
    g_vx_mms = vx;
    g_vy_mms = vy;
    g_x_mm = x;
    g_y_mm = y;
    last_x = x;
    last_y = y;
  }
  // If fwdKin failed (out-of-reach / singularity) we leave the last good
  // position in place; the CPU side sees an unchanging (x,y) and stalls the
  // bead, which is the user-visible "frozen briefly" behavior in the
  // failure-modes table.

  // --- 3. watchdog ---------------------------------------------------------
  const uint32_t now_ms = millis();
  if (inner::watchdogStale(now_ms, g_last_heartbeat_ms,
                           config::kWatchdogTimeoutMs)) {
    g_status |= config::kStatusSafeLatched;
  }

  // --- 3b. inter-board freshness + secondary readiness ----------------------
  const bool link_fresh =
      !inner::watchdogStale(now_ms, g_last_secondary_ms, config::kLinkWatchdogMs);
  const bool secondary_ready =
      (g_secondary_status & protocol::kSecondaryReady) != 0;

  // --- 4. compute output ---------------------------------------------------
#ifdef WELDING_JOG_MODE
  // Main jogs only its local (right) motor; ERM off; secondary disabled.
  const float pr = (float)inner::jogModePwm(now_ms, 0);
  writeMotor(config::kPinMotPwm, config::kPinMotDir, config::kMotorRightSign, pr);
  analogWrite(config::kPinErmPwm, 0);
  g_left_pwm_cmd = 0;
  g_link_enable  = false;
  g_status &= (uint8_t)~config::kStatusPolarityOk;  // never engage closed-loop
#else
  if ((g_status & config::kStatusSafeLatched) || !g_polarity_ok ||
      !secondary_ready || !link_fresh) {
    // Any fault -> both motors off + ERM off + secondary disabled.
    motorsOff();
    g_left_pwm_cmd = 0;
    g_link_enable  = false;
  } else {
    // Jacobian transpose: cartesian force (N) -> joint torque (N*mm).
    // tau1 = left joint, tau2 = right joint.
    float tau1 = 0.0f, tau2 = 0.0f;
    jacobianTranspose(theta1, theta2, g_x_mm, g_y_mm,
                      g_cmd_fx_n, g_cmd_fy_n, tau1, tau2);
    // Right motor: drive locally. Capstan + sqrt law.
    const float pr = inner::torqueToSignedPwmCapstan(
        tau2, config::kCapstanRpOverRs, config::kMotorTorqueConstNm);
    writeMotor(config::kPinMotPwm, config::kPinMotDir, config::kMotorRightSign, pr);
    analogWrite(config::kPinErmPwm, g_cmd_erm);
    // Left motor: queue the (range-limited) PWM for the secondary.
    float pl = inner::torqueToSignedPwmCapstan(
        tau1, config::kCapstanRpOverRs, config::kMotorTorqueConstNm);
    if (pl >  255.0f) pl =  255.0f;
    if (pl < -255.0f) pl = -255.0f;
    g_left_pwm_cmd = (int16_t)pl;
    g_link_enable  = true;
  }
#endif

  // --- 5. loop-time instrumentation ---------------------------------------
  const uint16_t dt = (uint16_t)(micros() - t0);
  if (dt > g_max_loop_window_us) g_max_loop_window_us = dt;

  // --- 6. flag the main loop to send STATE_UP -----------------------------
  g_tx_due = true;
}

// The 1 kHz control loop does software-float fwdKin/Jacobianᵀ — too long to run
// with interrupts masked. Masked, it starves the interrupt-driven TWI controller
// mid-transaction: the secondary I2C read returns short and the write times out,
// so g_theta_left freezes and the left joint goes dead (observed on the bench:
// the link_probe sketch read clean ONLY with no Timer1 ISR). Fix: re-enable
// interrupts for the duration of innerLoop() so TWI (and USART) ISRs are
// serviced, with a static guard so a tick that overruns into the next
// compare-match is DROPPED rather than re-entering innerLoop (the drop still
// shows up in g_max_loop_us). in_isr is checked/set with interrupts still masked
// at ISR entry, so the guard itself is race-free.
ISR(TIMER1_COMPA_vect) {
  static volatile bool in_isr = false;
  if (in_isr) return;       // re-entrant overrun tick -> drop it
  in_isr = true;
  sei();                    // allow nested TWI/USART ISRs during the float loop
  innerLoop();
  cli();
  in_isr = false;
}

// ----------------------------------------------------------------------------
// Serial RX: byte-at-a-time COBS frame reassembly. The 0x00 delimiter ends
// the current frame; the buffer absorbs up to one MTU worth of bytes.
// On overflow we reset the index to drop the malformed frame and let the
// next 0x00 boundary resync (free property of COBS framing).
// ----------------------------------------------------------------------------

constexpr size_t kRxBufCap = 64;
uint8_t g_rx_buf[kRxBufCap];
size_t  g_rx_idx = 0;

void handleFrame(const uint8_t* frame, size_t len) {
  uint8_t msg_type = 0, seq = 0;
  uint8_t data[16];
  const int n = protocol::parseFrame(frame, len, msg_type, seq,
                                     data, sizeof(data));
  if (n < 0) return;  // CRC or COBS failure -> silent drop, resync

  if (msg_type == protocol::MSG_HEARTBEAT) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      g_last_heartbeat_ms = millis();
    }
  } else if (msg_type == protocol::MSG_CMD_DOWN &&
             n == (int)sizeof(protocol::CmdDown)) {
    protocol::CmdDown cmd;
    memcpy(&cmd, data, sizeof(cmd));
    float fx = protocol::q15ToFloat(cmd.fx_q15, protocol::kForceMaxN);
    float fy = protocol::q15ToFloat(cmd.fy_q15, protocol::kForceMaxN);
    // Clamp to the cable-slip ceiling before the Jacobian ever sees it.
    inner::clampForceMagN(fx, fy, config::kForceCmdMaxN);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      g_cmd_fx_n = fx;
      g_cmd_fy_n = fy;
      g_cmd_erm  = cmd.erm_pwm;
      g_last_heartbeat_ms = millis();  // CMD_DOWN counts as liveness too
    }
  } else if (msg_type == protocol::MSG_ARM) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      g_status &= (uint8_t)~config::kStatusSafeLatched;
      g_last_heartbeat_ms = millis();
      // Lift polarity_ok via the ARM flag only after JOG_MODE has been run
      // and the operator removed -DWELDING_JOG_MODE from the build. ARM
      // here marks "operator has acknowledged bringup".
      g_polarity_ok = true;
    }
  }
}

void serialPoll() {
  while (Serial.available()) {
    const int b = Serial.read();
    if (b < 0) break;
    if (b == 0x00) {
      if (g_rx_idx > 0) handleFrame(g_rx_buf, g_rx_idx);
      g_rx_idx = 0;
    } else if (g_rx_idx < kRxBufCap) {
      g_rx_buf[g_rx_idx++] = (uint8_t)b;
    } else {
      // Frame too long -> drop and wait for next 0x00.
      g_rx_idx = 0;
    }
  }
}

// ----------------------------------------------------------------------------
// Inter-board link (I2C controller). Once per send interval: read the secondary's
// SecondaryState (cache theta_left on a VALID frame only) and write a SecondaryCmd.
// The Wire controller calls are synchronous -- no byte-pacing, no RX reassembly. The
// setWireTimeout set in setup() bounds a stuck bus: a stuck/timed-out read
// returns short, which leaves the cache stale, and the inter-board watchdog
// (kLinkWatchdogMs) then cuts force. Runs in loop().
// ----------------------------------------------------------------------------

void linkExchange() {
  static uint32_t last_link = 0;
  const uint32_t now = millis();
  if ((uint32_t)(now - last_link) < config::kLinkSendIntervalMs) return;
  last_link = now;

  // --- read SecondaryState (controller clocks 5 bytes out of the target) ---
  uint8_t buf[protocol::kSecondaryStateLen];
  Wire.requestFrom((uint8_t)config::kSecondaryI2cAddr,
                   (uint8_t)protocol::kSecondaryStateLen);
  uint8_t got = 0;
  while (Wire.available() && got < sizeof(buf)) buf[got++] = (uint8_t)Wire.read();
  while (Wire.available()) Wire.read();   // drain any extra bytes

  protocol::SecondaryState s;
  if (protocol::unpackSecondaryState(buf, got, &s)) {
    const float th = protocol::q15ToFloat(s.theta_left_q15, protocol::kAngleMaxRad);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      g_theta_left       = th;
      g_secondary_status  = s.status;
      g_last_secondary_ms = millis();
    }
  }
  // STALE-RETENTION (explicit): a short / timed-out / corrupt read does NOT
  // refresh g_theta_left or g_last_secondary_ms. Stale theta is bounded to
  // kLinkWatchdogMs, then force is cut. The ISR always reads the last VALID theta.

  // --- write SecondaryCmd (controller writes 5 bytes to the target) ---
  int16_t pwm; bool en;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { pwm = g_left_pwm_cmd; en = g_link_enable; }
  protocol::SecondaryCmd cmd;
  cmd.left_pwm = pwm;
  cmd.flags    = en ? protocol::kSecondaryEnable : 0;
  cmd._pad     = 0;
  uint8_t out[protocol::kSecondaryCmdLen];
  protocol::packSecondaryCmd(cmd, out);
  Wire.beginTransmission((uint8_t)config::kSecondaryI2cAddr);
  Wire.write(out, protocol::kSecondaryCmdLen);
  Wire.endTransmission();   // result ignored: a failed write ages out the
                            // secondary's own command watchdog -> its motor off
}

// ----------------------------------------------------------------------------
// STATE_UP TX. Pull a consistent snapshot under ATOMIC_BLOCK because the
// floats are 4 bytes each and the AVR cannot copy them atomically.
// ----------------------------------------------------------------------------

void sendStateUp() {
  protocol::StateUp s;
  uint8_t status_snap;
  uint16_t loop_us_snap;
  float ex, ey, vx, vy;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ex = g_x_mm;  ey = g_y_mm;          // pin joint E (kinematic frame)
    vx = g_vx_mms; vy = g_vy_mms;
    status_snap  = g_status;
    loop_us_snap = g_max_loop_us;
  }
  // Report the end-effector TIP, re-centered on the workspace origin (constant
  // offset from E, so velocity is unchanged). jacobianTranspose still uses E.
  float rx, ry;
  reportPointFromE(ex, ey, rx, ry);
  s.x_q15  = protocol::floatToQ15(rx, protocol::kPosMaxMm);
  s.y_q15  = protocol::floatToQ15(ry, protocol::kPosMaxMm);
  s.vx_q15 = protocol::floatToQ15(vx, protocol::kVelMaxMmS);
  s.vy_q15 = protocol::floatToQ15(vy, protocol::kVelMaxMmS);
  if (inner::loopOverrun(loop_us_snap)) {
    status_snap |= config::kStatusLoopOverrun;
  }
  if (g_polarity_ok) {
    status_snap |= config::kStatusPolarityOk;
  }
  s.max_loop_us = loop_us_snap;
  s.status      = status_snap;
  s._pad        = 0;

  uint8_t frame[32];
  const size_t n = protocol::buildFrame(
      protocol::MSG_STATE_UP, g_tx_seq++,
      reinterpret_cast<const uint8_t*>(&s), sizeof(s), frame);
  Serial.write(frame, n);
}

// ----------------------------------------------------------------------------
// Setup / main loop.
// ----------------------------------------------------------------------------

void setup() {
  pinMode(config::kPinMotPwm, OUTPUT);
  pinMode(config::kPinMotDir, OUTPUT);
  pinMode(config::kPinErmPwm, OUTPUT);
  pinMode(config::kPinErmDir, OUTPUT);
  digitalWrite(config::kPinErmDir, LOW);  // ERM on the Motor-2 H-bridge: DIR held
                                          // LOW, cue modulates PWM.
  pinMode(config::kPinMrSensor, INPUT);
  motorsOff();

  // MR sensor boot tare: prime the unwrapper and resolve this board's
  // zero so the boot pose reads as the reference angle. The operator holds the
  // end-effector at the centerline (theta = pi/2) during power-on (BRINGUP).
  // Done BEFORE the ISR starts so the first inner-loop sees a valid g_theta_right.
  {
    const int raw0 = analogRead(config::kPinMrSensor);
    g_mrR.prime(raw0);
    const long u0 = g_mrR.update(raw0, config::kMrFlipThreshold, config::kMrCountsPerRev);
    g_tare_offset_r = inner::mrTareOffsetRad(u0, config::kMrSlopeDegPerCount,
                                             config::kEncRightSign, config::kThetaTareRefRad);
    g_theta_right = inner::mrCountsToThetaRad(u0, config::kMrSlopeDegPerCount,
                                              config::kEncRightSign, g_tare_offset_r);
  }

  Serial.begin(115200);              // PC link (hardware UART)
  Wire.begin();                      // secondary link: I2C CONTROLLER on A4/A5
  // Bound a stuck/wedged bus so a held SDA cannot hang the 1 kHz-paced loop:
  // on timeout TWI resets and requestFrom returns short -> stale -> watchdog off.
  Wire.setWireTimeout(config::kI2cWireTimeoutUs, /*reset_on_timeout=*/true);

  // Seed the watchdogs so we are not SAFE_LATCHED before the first heartbeat
  // arrives, and the inter-board watchdog does not trip before the secondary's
  // first frame.
  g_last_heartbeat_ms = millis();
  g_last_secondary_ms  = millis();

  timer1InnerLoopBegin();
}

void loop() {
  // Local right joint: read the MR sensor HERE (not the ISR) and cache theta.
  // The ~104 us analogRead would blow the 1 kHz ISR budget; loop() runs well
  // above 1 kHz so the cache stays fresh.
  {
    const int raw = analogRead(config::kPinMrSensor);
    const long u  = g_mrR.update(raw, config::kMrFlipThreshold, config::kMrCountsPerRev);
    const float th = inner::mrCountsToThetaRad(u, config::kMrSlopeDegPerCount,
                                               config::kEncRightSign, g_tare_offset_r);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_theta_right = th; }
  }

  serialPoll();    // PC
  linkExchange();  // secondary <-> main over I2C (read state, write command)

  if (g_tx_due) {
    g_tx_due = false;
    sendStateUp();
  }

  // Roll the loop-time window every kLoopWindowMs.
  static uint32_t last_window_ms = 0;
  const uint32_t now = millis();
  if ((uint32_t)(now - last_window_ms) >= config::kLoopWindowMs) {
    last_window_ms = now;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      g_max_loop_us = g_max_loop_window_us;
      g_max_loop_window_us = 0;
    }
  }
}
