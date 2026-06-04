// link_diag.ino  --  I2C bus-scan smoke test (Tier 1; NOT part of the product).
//
// Flash to the MAIN board. Scans I2C addresses 1..126 and prints which ones ACK,
// over USB Serial at 115200. With the secondary powered and wired (A4<->A4 SDA,
// A5<->A5 SCL, shared GND, ONE 4.7 kOhm pull-up pair to +5V), this should report
// `found 0x08  <- secondary` -- the bench gate before the full firmware swap. No
// Timer1 ISR runs here, so it isolates the bus/wiring from the control loop.
//
// Replaces the retired SoftwareSerial RX byte/frame-count diagnostic: the
// inter-board link moved to hardware I2C, so that probe no longer applies.

#include <Arduino.h>
#include <Wire.h>

#include "config.h"   // kSecondaryI2cAddr / kI2cWireTimeoutUs

using namespace welding;

void setup() {
  Serial.begin(115200);
  Wire.begin();                                   // I2C controller
  Wire.setWireTimeout(config::kI2cWireTimeoutUs, /*reset_on_timeout=*/true);
  Serial.println(F("I2C BUS SCAN  (controller; A4=SDA A5=SCL; no Timer1 ISR)"));
}

void loop() {
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    const uint8_t err = Wire.endTransmission();   // 0 == device ACKed
    if (err == 0) {
      ++found;
      Serial.print(F("  found 0x"));
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      if (addr == config::kSecondaryI2cAddr) Serial.print(F("  <- secondary"));
      Serial.println();
    }
  }
  if (found == 0) {
    Serial.println(F("  (no devices ACKed -- check wiring / pull-ups / shared GND)"));
  }
  Serial.print(F("scan done: "));
  Serial.print(found);
  Serial.println(F(" device(s)"));
  delay(2000);
}
