// welding_motor_smoketest.ino
// --------------------------------------------------------------------------
// MINIMAL open-loop motor smoke test, modeled on the known-good stock Hapkit
// demo sketch. Purpose: isolate "does the motor drive at
// all" from the full welding firmware (no I2C, no 1 kHz ISR, no safety state
// machine, no sensor). If THIS moves the motor, the fault is in the welding
// firmware's drive path; if this ALSO shows 0 A / no motion, the fault is the
// hardware power path (motor supply rail, H-bridge, or motor leads).
//
// Pins match welding_common/config.h:
//   Motor 1: PWM = D5, DIR = D8   (Hapkit Motor 1 channel)
//
// Behavior: swing one direction ~0.8 s, pause ~0.4 s, swing the other way
// ~0.8 s, pause ~0.4 s, repeat. Same "wiggle" shape as JOG_MODE, open-loop.
// --------------------------------------------------------------------------

const int pwmPin = 5;   // PWM output pin for motor 1  (matches config.h kPinMotPwm)
const int dirPin = 8;   // direction output pin for motor 1 (matches config.h kPinMotDir)

const int kDuty   = 180;   // 0..255 (~71%). Open-loop; hand on the kill switch.
const int kDriveMs = 1200; // drive time each direction (longer -> bigger sweep)
const int kPauseMs = 500;  // pause between direction flips

void setup() {
  Serial.begin(115200);

  // DO NOT call setPwmFrequency(5, ...) here. Pins 5/6 PWM run off Timer0 —
  // the same timer as millis()/delay(). The stock Hapkit sketch sets divisor 1
  // (~31 kHz), which makes Timer0 tick 64x fast and delay() run 64x short,
  // collapsing the 0.8 s swings into a buzz. The stock sketch was immune because
  // its loop never calls delay(). We leave Timer0 at default (~976 Hz PWM, mild
  // whine) so the delay()-based timing below is correct.

  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  analogWrite(pwmPin, 0);     // start stopped
  digitalWrite(dirPin, LOW);

  Serial.println(F("motor smoke test: pins D5(PWM)/D8(DIR), duty 130, +/- 0.8s"));
}

void loop() {
  // --- swing one way ---
  digitalWrite(dirPin, HIGH);
  analogWrite(pwmPin, kDuty);
  Serial.println(F("DIR=HIGH  PWM=130  (swing A)"));
  delay(kDriveMs);

  analogWrite(pwmPin, 0);
  Serial.println(F("stop"));
  delay(kPauseMs);

  // --- swing the other way ---
  digitalWrite(dirPin, LOW);
  analogWrite(pwmPin, kDuty);
  Serial.println(F("DIR=LOW   PWM=130  (swing B)"));
  delay(kDriveMs);

  analogWrite(pwmPin, 0);
  Serial.println(F("stop"));
  delay(kPauseMs);
}
