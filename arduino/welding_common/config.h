#ifndef WELDING_CONFIG_H
#define WELDING_CONFIG_H

// All Arduino-side magic numbers in one place.
// Pin assignments are gated by ARDUINO so the host test build can include
// this header for the numeric constants without dragging in <Arduino.h>.
//
// Pin / timer allocation rationale on the atmega328p (Uno):
//
//   Resource    | Used for                  | Why
//   ------------+---------------------------+--------------------------------
//   A2 (ADC2)   | MR analog angle sensor    | Hapkit sensor; read in loop()
//   A4 / A5     | I2C SDA / SCL             | MAIN<->secondary link; HW TWI
//   Timer0      | millis() + D5/D6 PWM      | Motor 1 PWM (D5) + ERM PWM (D6)
//   Timer1      | 1 kHz CTC for inner loop  | CTC mode leaves D9/D10 untouched
//   Timer2      | (free)                    | ERM moved off D11/Timer2
//   D5  (OC0B)  | Motor 1 PWM               | Hapkit Motor 1 channel
//   D8          | Motor 1 DIR               | Hapkit Motor 1 channel
//   D6  (OC0A)  | ERM PWM (MAIN only)       | Hapkit Motor 2 channel
//   D7          | ERM DIR (MAIN only)       | Hapkit Motor 2 channel (held LOW)
//   D2 / D3     | (free)                    | freed by the I2C migration
//
// Pin map verified against the Hapkit shield pinout reference: D5/D8=Motor 1,
// D6/D7=Motor 2, A2=MR sensor. The inter-board link is hardware I2C/TWI on the
// dedicated A4/A5 SDA/SCL pins: the one hardware UART is owned by the PC,
// so TWI is the only free hardware serial peripheral. D2/D3 (the only
// no-special-function digital pins) are now FREE. AVOID the shield-routed pins:
// D4 (SD-card SS), D9/D10/A0/A1 (Grove), D11/D12/D13 (SD-card SPI), A3 (FSR).
// D9/D10 also stay clear so Timer1 CTC remains free of OC1A/OC1B pin coupling.
//
// ⚠ PWM-WHINE / millis() TRAP. Motor PWM (D5) AND ERM PWM (D6)
// are BOTH on Timer0 (~490-980 Hz) and WILL whine audibly. Hapkit rigs commonly
// fix this with setPwmFrequency(D5 or D6, 1) (~31 kHz) — but that reprograms
// TIMER0, which also clocks millis()/delay(), making them run 64x FAST. The
// watchdog (kWatchdogTimeoutMs below) and the 100 ms loop window both use raw
// millis(); if anyone adds setPwmFrequency WITHOUT a compensating millis()/64
// correction, the 250 ms watchdog silently becomes ~4 ms and
// trips every loop. Do BOTH in one commit, or move the watchdog/window timing to
// micros(). Do not "fix the whine" alone.

#include <stdint.h>

namespace welding {
namespace config {

// --- Hapkit native pin map (verified vs the Hapkit shield pinout) -----------
// These are the Hapkit shield's FIXED motor-driver channels, not a free choice:
//   Motor 1 channel: PWM D5 / DIR D8   -> the rotating joint motor (BOTH boards)
//   Motor 2 channel: PWM D6 / DIR D7   -> the ERM (MAIN only; spare H-bridge ch.)
//   A2 = MR sensor output (both boards). A3=FSR, D4=SD-card SS, D9/D10/A0/A1=Grove,
//   D11/D12/D13=SD-card SPI are all routed by the shield — do NOT reuse them.
// The ERM rides the Motor-2 H-bridge, which IS its drive+flyback stage (so no
// separate transistor is needed); DIR is held LOW and the cue modulates PWM.
// (A2 == 16 on the Uno; written numeric so this header stays host-portable.)
constexpr uint8_t kPinMrSensor = 16;  // A2 — MR analog angle sensor (both boards)
constexpr uint8_t kPinMotPwm = 5;   // Motor 1 PWM (D5, Timer0 OC0B)
constexpr uint8_t kPinMotDir = 8;   // Motor 1 DIR (D8)
constexpr uint8_t kPinErmPwm = 6;   // Motor 2 PWM (D6, Timer0 OC0A) -- MAIN ERM
constexpr uint8_t kPinErmDir = 7;   // Motor 2 DIR (D7) -- MAIN ERM, held LOW

// --- Inter-board link (hardware I2C/TWI on A4/A5) ---------------------------
// The MAIN is the I2C controller; the secondary is target kSecondaryI2cAddr. A4=SDA,
// A5=SCL, shared GND, ONE 4.7 kOhm pull-up pair to +5V on the bus (bare Unos
// have no A4/A5 pull-ups). 100 kHz. The wire form is a bare struct + crc8, NO
// COBS -- I2C self-frames each transaction (requestFrom / endTransmission are the
// boundaries). WHY I2C over the old SoftwareSerial link: the MAIN's one hardware
// UART is owned by the PC link, so TWI is the only free hardware serial
// peripheral, and unlike SoftwareSerial it does not hold interrupts off for whole
// bytes -- removing the contention against the 1 kHz Timer1 ISR at its root
// (supersedes the earlier SoftwareSerial transport). Bonus: the secondary's link
// is off D0/D1, so its hardware Serial is free to flash + monitor while live.
// (400 kHz + 2.2 kOhm pull-ups is deferred -- needs a rise-time scope check.)
constexpr uint8_t  kSecondaryI2cAddr  = 0x08;   // secondary's 7-bit I2C target address
constexpr uint32_t kI2cWireTimeoutUs = 3000;   // ~5x a 100 kHz 5-byte txn; << kLinkWatchdogMs.
                                               // Passed to Wire.setWireTimeout(.., reset=true)
                                               // so a wedged bus fails SAFE (timeout -> short
                                               // read -> stale -> watchdog -> motorsOff),
                                               // never hangs the loop. (core >= 1.8.1.)

// --- MR analog angle-sensor calibration (replaces quadrature) ---------------
// Standard Hapkit: analogRead(A2) 0..1023, software flip-unwrap (inner.h
// MrUnwrapper), then a per-kit linear fit counts->degrees. Values are a
// bench-calibrated Hapkit fit; the SLOPE + geometry are shared across kits, but the
// per-board ZERO is resolved by a boot tare (mrTareOffsetRad) at a known
// reference pose, so no per-board intercept is baked. PER-KIT — confirm the
// slope and that the flip threshold catches every wrap on the bench (BRINGUP
// §0.2 / §2.1) before trusting position.
constexpr int      kMrFlipThreshold    = 700;       // raw jump => 180-degree wrap
constexpr int      kMrCountsPerRev      = 920;       // MR counts per revolution
constexpr float    kMrSlopeDegPerCount = 0.01555f;  // bench linear-fit slope
// Boot-tare reference: the operator holds the end-effector at the centerline
// pose (theta1 = theta5 = pi/2) during power-on, so each board tares its joint
// to pi/2. The hand-move check (BRINGUP §2.1) validates the result.
constexpr float    kThetaTareRefRad    = 1.5707963f; // pi/2 (centerline pose)

// Per-board sign conventions (+1 = "positive sensor sweep = positive theta CCW
// from +x"; "+PWM moves the joint positive"). Each board's pair is locked after
// that board's JOG_MODE bringup. The main uses *Right; the secondary uses *Left.
constexpr int8_t kEncRightSign   = +1;
constexpr int8_t kMotorRightSign = +1;
constexpr int8_t kEncLeftSign    = +1;
constexpr int8_t kMotorLeftSign  = +1;

// --- Inner loop timing ------------------------------------------------------
constexpr uint16_t kInnerLoopHz = 1000;
// Timer1 CTC: prescaler 8, OCR1A = (16e6 / 8 / 1000) - 1 = 1999.
constexpr uint16_t kTimer1OcrFor1kHz = 1999;

// --- Velocity estimation window ---------------------------------------------
// Velocity is a finite difference of the MR-derived end-effector position. A
// 1-step diff at 1 kHz turns the sensor's position quantization (~tenths of a
// mm) into tens of mm/s of spike — against a ~20 mm/s signal. Differencing over
// this many 1 kHz ticks instead (inner::VelocityEstimator) reports a ~N-ms-
// window velocity = the mean of N one-step diffs, cutting the zero-mean
// quantization noise by ~sqrt(N). 16 ticks = 16 ms window, ~4x quieter; the
// handle's own motion is a few Hz so a 16 ms window is transparent. Group delay
// ~= N/2 ms (~8 ms here). MUST live here (firmware ISR), NOT on the PC: STATE_UP
// reaches the of-app at only ~60 Hz (loop() is paced by the I2C exchange), so
// the 1 kHz position stream the average needs exists ONLY on the primary — the
// bench measured of-app frames/pump = 1, i.e. a PC-side average is a no-op.
// PER-FEEL: widen for smoother/laggier, narrow for crisper/noisier.
constexpr int kVelWindowTicks = 16;   // 16 ms window at 1 kHz; ~4x noise cut

// Loop-time budget. If max_loop_us exceeds this for any 100 ms window we set
// kStatusLoopOverrun in STATE_UP.
constexpr uint16_t kLoopBudgetUs = 900;
constexpr uint32_t kLoopWindowMs = 100;

// --- Watchdog ---------------------------------------------------------------
// On the AVR side, "fresh heartbeat" means a HEARTBEAT (or CMD_DOWN; see
// implementation) arrived within this window. Stale -> SAFE_LATCHED until
// an ARM message lifts the latch.
//
// 250 ms, widened from 100 ms. The PC heartbeat is pumped
// from ofApp::update() at the ~60 Hz render rate (ControlThreads::start() is
// bypassed), so a normal render hitch (path load, GUI dialog, alt-tab) can
// exceed 100 ms and trip a spurious E-stop. 250 ms clears typical hitches while
// still cutting force within a quarter second of a real CPU death. TRADEOFF:
// this lengthens the worst-case runaway window after a genuine heartbeat loss.
// If the 1 kHz threaded heartbeat path is ever enabled, tighten this
// back toward 100 ms.
constexpr uint32_t kWatchdogTimeoutMs = 250;

// --- Inter-board link timing ------------------------------------------------
// The CONTROLLER runs the I2C exchange (read SecondaryState, write SecondaryCmd) at
// most this often. 10 ms = 100 Hz (was 5 ms/200 Hz). The whole linkage is
// one rigid body (hand-bandwidth, a few Hz), so 100 Hz is ample and still leaves
// 5x margin on the 50 ms link watchdog. With hardware TWI the per-byte
// interrupt-off contention that drove the old SoftwareSerial rate choice is gone,
// so this is now purely the controller's poll cadence.
constexpr uint32_t kLinkSendIntervalMs = 10;
// If a VALID SecondaryState (main) / SecondaryCmd (secondary) has not arrived within
// this window, fail to motors-off. STALE-RETENTION (explicit): an invalid /
// short / timed-out frame NEVER refreshes the freshness timestamp or the cached
// theta_left, so a silently-rotated Jacobian cannot outlive this window. A stale
// theta_left would otherwise mis-rotate the Jacobian, so the main treats it as a
// force-kill. 50 ms = 5 missed 100 Hz frames of slack before force cuts.
constexpr uint32_t kLinkWatchdogMs = 50;

// --- Motor PWM clamp --------------------------------------------------------
// Safety ceiling, in raw PWM units. 200/255 = 78% duty. Tune up only after
// the polarity bringup test passes and torque-to-PWM scaling is calibrated.
constexpr uint8_t kPwmMaxClamp = 200;

// --- Torque -> PWM: capstan reduction + sqrt law (supersedes the linear
// kTorqueToPwmScale). The motor+drive produces torque ~ duty^2, so
// commanding duty ~ sqrt(torque) linearizes it (the standard Hapkit drive
// law). The device is capstan-driven, so joint torque is first reduced to the
// motor-pulley torque by the radius ratio rp/rs:
//     Tp[N*m] = (rp/rs) * tau_joint[N*m]
//     duty    = sqrt(|Tp| / kMotorTorqueConstNm)   clamped to [0,1]
// Values are the bench Hapkit calibration (rp=0.005 m, rs=0.077 m -> ratio
// ~0.0649; k = 0.0183 N*m at duty^2 = 1). PER-KIT — confirm rp/rs and the
// 0.0183-equivalent on the bench (BRINGUP §1.4) before trusting force magnitude.
constexpr float kCapstanRpOverRs   = 0.005f / 0.077f;  // ~0.06494 (motor pulley / sector)
constexpr float kMotorTorqueConstNm = 0.0183f;         // N*m at duty^2 = 1 (bench fit)

// Force scale used on the wire: q15 +/- 10 N (protocol::kForceMaxN).
// Re-stated here as a float so inner_loop.h does not have to pull in protocol.h.
constexpr float kForceMaxN = 10.0f;

// Cartesian force command ceiling, N. The capstan cable slips above a per-kit
// force, so commanding past it just slips instead of delivering force. The
// bench Hapkit slipped at ~4 N; 5 N keeps a little headroom over
// that observation. The commanded |F| is clamped to this before the Jacobian so
// the motors are never asked for a force the cable can't hold. Kept separate
// from the 10 N wire scale (transport range) on purpose: this is a hardware
// actuator limit, not a protocol property. PER-KIT — re-measure the slip point
// on the real rig (BRINGUP) before trusting the magnitude.
constexpr float kForceCmdMaxN = 5.0f;

// --- JOG_MODE ---------------------------------------------------------------
// Compile-time gate for the motor-polarity bringup test. When defined, the
// inner loop ignores CMD_DOWN, cycles each motor at a low PWM in turn, and
// holds the polarity_ok status bit clear so closed-loop force cannot engage.
//
// To enable: uncomment the next line, reflash, run the bringup procedure in
// BRINGUP.md, then comment it back out and reflash for normal operation.
// #define WELDING_JOG_MODE

// JOG_MODE parameters: alternate motor1 +/- and motor2 +/- per phase.
// kJogPwm bumped from 60 after bench testing: 60 (and even 127) was too
// gentle to visibly rotate the capstan-reduced joint, so the polarity read was
// unreadable. An open-loop smoke test (welding_motor_smoketest) showed motion
// becomes clearly visible at ~180. jogModePwm() now returns int16_t, so the
// full 0..255 range is usable (it used to cast to int8_t, capping at 127 and
// wrapping 150 -> -106). Stays under kPwmMaxClamp=200.
constexpr uint8_t  kJogPwm = 180;         // ~71% duty; clearly visible sweep
constexpr uint32_t kJogPhaseMs = 1200;    // 4 phases x 1.2 s: +1.2s, -1.2s, 2.4s pause

// --- STATE_UP status bits ---------------------------------------------------
// Mirror the protocol.h StateUp.status bit layout. Defined here so the
// inner loop can build the status byte without including protocol.h (keeps
// inner_loop.h test-clean).
constexpr uint8_t kStatusSafeLatched  = 0x01;
constexpr uint8_t kStatusPolarityOk   = 0x02;
constexpr uint8_t kStatusLoopOverrun  = 0x04;

}  // namespace config
}  // namespace welding

#endif  // WELDING_CONFIG_H
