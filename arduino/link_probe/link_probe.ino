// link_probe.ino -- I2C link DATA probe (Tier 1; NOT part of the product).
//
// Flash to the MAIN. Unlike link_diag (which only address-scans), this performs
// the REAL controller exchange that welding_trainer.linkExchange() does:
//   1. requestFrom(0x08, 5)  -> read a SecondaryState, unpackSecondaryState()
//   2. beginTransmission/write(SecondaryCmd)/endTransmission  -> and report wErr
// then prints the raw bytes, got count, CRC-ok, decoded theta_left, and the
// write error code over USB Serial @115200, ~10 Hz.
//
// CRITICAL: this sketch runs NO Timer1 ISR. welding_trainer runs a 1 kHz Timer1
// ISR doing software-float fwdKin with interrupts disabled, which can starve the
// interrupt-driven TWI mid-transaction. So:
//   * probe reads clean (got=5 ok=1) but welding_trainer's link is dead
//       -> the ISR/TWI interaction is the culprit.
//   * probe ALSO fails (got<5 / ok=0 / wErr!=0)
//       -> a more basic Wire/onRequest/clock issue, independent of the ISR.
// Run it with the real welding_secondary on COM7; move the LEFT joint and watch
// tL_mrad swing if ok=1.

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "protocol.h"

using namespace welding;

void setup() {
  Serial.begin(115200);
  Wire.begin();                                   // I2C controller (A4/A5)
  Wire.setWireTimeout(config::kI2cWireTimeoutUs, /*reset_on_timeout=*/true);
  Serial.println(F("I2C LINK PROBE (controller, NO Timer1 ISR): read SecondaryState + write SecondaryCmd"));
}

void loop() {
  // --- READ SecondaryState (exactly as linkExchange does) ---
  uint8_t buf[protocol::kSecondaryStateLen];
  const uint8_t reqRet = Wire.requestFrom((uint8_t)config::kSecondaryI2cAddr,
                                          (uint8_t)protocol::kSecondaryStateLen);
  uint8_t got = 0;
  while (Wire.available() && got < sizeof(buf)) buf[got++] = (uint8_t)Wire.read();
  while (Wire.available()) Wire.read();
  protocol::SecondaryState s;
  const bool ok = protocol::unpackSecondaryState(buf, got, &s);

  // --- WRITE a zero SecondaryCmd and capture the error code ---
  protocol::SecondaryCmd c;
  c.left_pwm = 0;
  c.flags    = 0;
  c._pad     = 0;
  uint8_t out[protocol::kSecondaryCmdLen];
  protocol::packSecondaryCmd(c, out);
  Wire.beginTransmission((uint8_t)config::kSecondaryI2cAddr);
  Wire.write(out, protocol::kSecondaryCmdLen);
  const uint8_t wErr = Wire.endTransmission();   // 0=ok 1=long 2=addrNACK 3=dataNACK 4=other 5=timeout

  // --- report ---
  Serial.print(F("M reqRet=")); Serial.print(reqRet);
  Serial.print(F(" got="));     Serial.print(got);
  Serial.print(F(" ok="));      Serial.print(ok ? 1 : 0);
  Serial.print(F(" raw="));
  for (uint8_t i = 0; i < got; ++i) {
    if (buf[i] < 16) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.print(F("st="));       Serial.print(ok ? s.status : 0);
  Serial.print(F(" tL_mrad=")); Serial.print(ok ? (int)(protocol::q15ToFloat(s.theta_left_q15, protocol::kAngleMaxRad) * 1000.0f) : 0);
  Serial.print(F(" wErr="));    Serial.println(wErr);
  delay(100);
}
