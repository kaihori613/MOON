// ===========================================================================
//  as5600_simple  --  does the encoder answer, and if not, why not?
// ===========================================================================
//
//  Run this one FIRST. No commands, 9600 baud, prints on its own forever.
//  Once it reads angles, move to as5600_characterise/ for the measurements that go
//  into Config.h.
//
//  When the part does NOT answer, "no" is not a useful answer, so this
//  reports three things instead:
//
//    1. The idle state of SDA and SCL, checked BEFORE the Wire library
//       claims the pins. Both should sit HIGH, held there by pull-ups. A
//       line stuck LOW is a short or a device jamming the bus, and no amount
//       of software will talk over it.
//    2. A full bus scan, repeated every two seconds. Nothing at 0x36 and
//       nothing anywhere are different faults with different fixes, and
//       re-scanning means you can wiggle a wire and watch it appear.
//    3. The I2C error code, which distinguishes "nobody acknowledged that
//       address" from "the bus itself is broken".
//
//  Wiring: VCC, GND, SDA->A4, SCL->A5. If your board brings DIR out and does
//  not tie it, jumper DIR to GND.

#include <Wire.h>

const uint8_t AS5600 = 0x36;      // fixed in silicon, not selectable

bool     g_found = false;
uint32_t g_nextScan = 0;

// Before Wire.begin() takes the pins, look at them as plain inputs. With
// pull-ups present and the bus idle, both float HIGH. This is the one check
// that sees a hardware fault the I2C library can only report as silence.
void checkIdleLevels() {
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  delay(5);
  const bool sda = digitalRead(A4);
  const bool scl = digitalRead(A5);

  Serial.print(F("SDA (A4) idle: ")); Serial.println(sda ? F("HIGH") : F("LOW"));
  Serial.print(F("SCL (A5) idle: ")); Serial.println(scl ? F("HIGH") : F("LOW"));

  if (!sda || !scl) {
    Serial.println(F("  A line is LOW at rest. Expect HIGH on both."));
    Serial.println(F("  -> SDA or SCL shorted to ground"));
    Serial.println(F("  -> a device holding the bus down"));
    Serial.println(F("  -> no pull-ups AND nothing driving the line"));
  } else {
    Serial.println(F("  Both high: pull-ups present, bus idle. Good."));
  }
}

const __FlashStringHelper* i2cError(uint8_t code) {
  switch (code) {
    case 0: return F("ok");
    case 1: return F("data too long");
    case 2: return F("NACK on address - nobody home at that address");
    case 3: return F("NACK on data");
    case 4: return F("bus error");
    case 5: return F("timeout");
    default: return F("unknown");
  }
}

bool scanBus() {
  Serial.println(F("Scanning 0x01-0x7E..."));
  uint8_t found = 0;
  bool sawTarget = false;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  0x"));
      if (a < 16) Serial.print('0');
      Serial.print(a, HEX);
      if      (a == 0x36) { Serial.print(F("  <- AS5600")); sawTarget = true; }
      else if (a == 0x27 || a == 0x3F) Serial.print(F("  <- LCD backpack"));
      Serial.println();
      found++;
    }
  }
  if (found == 0) {
    Serial.println(F("  NOTHING on the bus."));
    Serial.println(F("  -> SDA and SCL swapped (SDA=A4, SCL=A5)"));
    Serial.println(F("  -> VCC or GND not actually connected"));
    Serial.println(F("  -> a header pin soldered but not wetted to the pad"));
    Serial.println(F("     (check continuity pin-to-pad with a meter)"));
  } else if (!sawTarget) {
    Serial.println(F("  Bus works, but nothing at 0x36."));
    Serial.println(F("  The AS5600 address is fixed, so a device at some"));
    Serial.println(F("  other address is a different part than you think."));
  }
  return sawTarget;
}

uint8_t reg8(uint8_t r, bool* ok) {
  Wire.beginTransmission(AS5600);
  Wire.write(r);
  const uint8_t e = Wire.endTransmission(false);
  if (e != 0) { *ok = false; return 0; }
  if (Wire.requestFrom(AS5600, (uint8_t)1) != 1) { *ok = false; return 0; }
  *ok = true;
  return Wire.read();
}

uint16_t reg12(uint8_t r, bool* ok) {
  Wire.beginTransmission(AS5600);
  Wire.write(r);
  const uint8_t e = Wire.endTransmission(false);
  if (e != 0) { *ok = false; return 0; }
  if (Wire.requestFrom(AS5600, (uint8_t)2) != 2) { *ok = false; return 0; }
  const uint8_t hi = Wire.read(), lo = Wire.read();
  *ok = true;
  return (((uint16_t)hi << 8) | lo) & 0x0FFF;
}

void setup() {
  Serial.begin(9600);
  delay(300);
  Serial.println();
  Serial.println(F("=== AS5600 simple test ==="));
  Serial.println(F("Serial Monitor at 9600 baud."));
  Serial.println();

  checkIdleLevels();
  Serial.println();

  Wire.begin();
  g_found = scanBus();
  Serial.println();
  if (g_found) Serial.println(F("angle    degrees   AGC   magnet"));
}

void loop() {
  if (!g_found) {
    // Keep looking. Wiggle a wire and watch it turn up.
    if (millis() >= g_nextScan) {
      g_nextScan = millis() + 2000;
      Wire.beginTransmission(AS5600);
      const uint8_t e = Wire.endTransmission();
      Serial.print(F("0x36: "));
      Serial.println(i2cError(e));
      if (e == 0) {
        Serial.println(F("Found it. Reading angles:"));
        Serial.println(F("angle    degrees   AGC   magnet"));
        g_found = true;
      }
    }
    return;
  }

  bool ok1, ok2, ok3;
  const uint16_t raw = reg12(0x0C, &ok1);   // RAW_ANGLE
  const uint8_t  st  = reg8(0x0B, &ok2);    // STATUS
  const uint8_t  agc = reg8(0x1A, &ok3);    // AGC

  if (!ok1 || !ok2 || !ok3) {
    Serial.println(F("lost it - connection dropped mid-read"));
    g_found = false;
    g_nextScan = millis() + 2000;
    return;
  }

  Serial.print(raw);
  Serial.print(F("\t"));
  Serial.print(raw * (360.0f / 4096.0f), 2);   // 0.0879 deg per count
  Serial.print(F("\t  "));
  Serial.print(agc);
  Serial.print(F("\t"));

  // MD bit5 = detected, ML bit4 = too weak, MH bit3 = too strong.
  if      (!(st & 0x20)) Serial.println(F("NONE - wrong magnet type, or too far"));
  else if (st & 0x10)    Serial.println(F("weak - move closer"));
  else if (st & 0x08)    Serial.println(F("strong - move further"));
  else                   Serial.println(F("ok"));

  delay(500);
}
