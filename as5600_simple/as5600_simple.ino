// ===========================================================================
//  as5600_simple  --  does the encoder answer, and is the magnet right?
// ===========================================================================
//
//  Run this one FIRST. It takes no commands, needs no settings, and starts
//  printing on its own the moment it boots. Once it prints, move to
//  as5600_test/ for the measurements that actually go into Config.h.
//
//  Three deliberate choices, each removing a way to see nothing at all:
//
//    9600 baud     -- the Serial Monitor's default, so a wrong baud setting
//                     cannot make this look dead.
//    No commands   -- nothing to type, so the monitor's line-ending dropdown
//                     cannot matter. ("No line ending" is the default, and it
//                     makes a command-driven sketch completely silent.)
//    Prints always -- every 500 ms, forever. Miss the boot banner and the
//                     next line is half a second away.
//
//  Wiring: VCC, GND, SDA->A4, SCL->A5. If your breakout brings DIR out and
//  does not tie it, jumper DIR to GND -- a floating DIR leaves the counting
//  direction undefined.

#include <Wire.h>

const uint8_t AS5600 = 0x36;      // fixed in silicon, not selectable

uint8_t reg8(uint8_t r) {
  Wire.beginTransmission(AS5600);
  Wire.write(r);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom(AS5600, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

uint16_t reg12(uint8_t r) {
  Wire.beginTransmission(AS5600);
  Wire.write(r);
  if (Wire.endTransmission(false) != 0) return 0xFFFF;
  Wire.requestFrom(AS5600, (uint8_t)2);
  if (Wire.available() < 2) return 0xFFFF;
  uint8_t hi = Wire.read(), lo = Wire.read();
  return (((uint16_t)hi << 8) | lo) & 0x0FFF;
}

// If 0x36 is silent, the useful question is not "is it broken" but "what IS
// on this bus" -- one device at an unexpected address and no devices at all
// are completely different problems with completely different fixes.
void scanBus() {
  Serial.println(F("Scanning I2C bus..."));
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  device at 0x"));
      if (a < 16) Serial.print('0');
      Serial.print(a, HEX);
      if (a == 0x36) Serial.print(F("  <- AS5600"));
      Serial.println();
      found++;
    }
  }
  if (found == 0) {
    Serial.println(F("  nothing at all."));
    Serial.println(F("  -> SDA and SCL swapped, or not on A4/A5"));
    Serial.println(F("  -> no power to the board"));
    Serial.println(F("  -> missing pull-ups"));
  }
}

void setup() {
  Serial.begin(9600);
  delay(300);                     // let the port settle before the banner
  Wire.begin();

  Serial.println();
  Serial.println(F("=== AS5600 simple test ==="));
  Serial.println(F("Set the Serial Monitor to 9600 baud."));
  Serial.println();
  scanBus();
  Serial.println();
  Serial.println(F("angle    degrees   AGC   magnet"));
}

void loop() {
  const uint16_t raw = reg12(0x0C);         // RAW_ANGLE
  const uint8_t  st  = reg8(0x0B);          // STATUS
  const uint8_t  agc = reg8(0x1A);          // AGC

  if (raw == 0xFFFF) {
    Serial.println(F("no answer from 0x36"));
    delay(1000);
    return;
  }

  // 4096 counts per revolution: 0.0879 degrees each.
  const float deg = raw * (360.0f / 4096.0f);

  Serial.print(raw);
  Serial.print(F("\t"));
  Serial.print(deg, 2);
  Serial.print(F("\t  "));
  Serial.print(agc);
  Serial.print(F("\t"));

  // MD bit 5 = detected, ML bit 4 = too weak, MH bit 3 = too strong.
  if      (!(st & 0x20)) Serial.println(F("NONE - wrong magnet or too far"));
  else if (st & 0x10)    Serial.println(F("weak - move closer"));
  else if (st & 0x08)    Serial.println(F("strong - move further"));
  else                   Serial.println(F("ok"));

  delay(500);
}
