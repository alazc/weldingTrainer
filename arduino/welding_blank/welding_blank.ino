// welding_blank — inert "do nothing" firmware.
// Reset leaves all GPIO as high-impedance inputs, so nothing drives the
// motors, ERM, or I2C bus. Flash this to safely park a board.
void setup() {}
void loop() {}
