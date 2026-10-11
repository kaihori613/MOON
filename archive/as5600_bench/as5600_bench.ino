// ===========================================================================
//  as5600_bench  --  is it the code or the magnet?
// ===========================================================================
//
//  For "the angle barely changes when I turn the magnet". 115200 baud, no
//  commands, prints four times a second. Each line reads the angle TWO ways:
//
//    burst  one 2-byte read from 0x0C with a repeated start -- how
//           as5600_simple, as5600_test and actuator_v2 read it
//    split  0x0D then 0x0C as two separate 1-byte reads, each ending in a
//           STOP -- how the OLED reference sketch reads it
//
//  plus what the chip says about the magnet. Turn the magnet one full turn,
//  slowly, and read the result:
//
//    burst and split agree, span reaches ~360      everything works
//    burst and split DISAGREE                      the read method matters --
//                                                  report it, it is a code bug
//    both agree, both barely move, and
//      AGC is pinned near its maximum, or
//      magnitude is small                          the field is too weak in the
//                                                  chip's plane: wrong magnet
//                                                  (axial, not diametric), too
//                                                  far away, or off-centre
//    both agree, both barely move, AGC and
//      magnitude look healthy                      the magnet is not turning
//                                                  relative to the chip
//
//  'span' is the widest arc seen since reset; send any character to reset it.
//
//  Wiring: VCC, GND, SDA->A4, SCL->A5, DIR->GND.

#include <Wire.h>

const uint8_t AS5600 = 0x36;      // an AS5600L answers at 0x40 instead

static uint16_t g_first = 0;      // reference for the span
static int16_t  g_lo = 0, g_hi = 0;
static bool     g_haveFirst = false;

// Signed shortest way round, so turning through 0/4095 does not look like a
// jump of a whole turn.
int16_t angleDiff(uint16_t a, uint16_t b) {
  int16_t d = (int16_t)a - (int16_t)b;
  if (d >  2048) d -= 4096;
  if (d < -2048) d += 4096;
  return d;
}

// The sketches' way: one transaction, repeated start, two bytes.
bool readBurst(uint8_t reg, uint16_t* out) {
  Wire.beginTransmission(AS5600);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600, (uint8_t)2) != 2) return false;
  const uint8_t hi = Wire.read(), lo = Wire.read();
  *out = (((uint16_t)hi << 8) | lo) & 0x0FFF;
  return true;
}

// The reference's way: one register per transaction, each ended with a STOP.
bool readOne(uint8_t reg, uint8_t* out) {
  Wire.beginTransmission(AS5600);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(AS5600, (uint8_t)1) != 1) return false;
  *out = Wire.read();
  return true;
}

bool readSplit(uint16_t* out) {
  uint8_t lo, hi;
  if (!readOne(0x0D, &lo)) return false;   // RAW ANGLE 7:0
  if (!readOne(0x0C, &hi)) return false;   // RAW ANGLE 11:8
  *out = (((uint16_t)hi << 8) | lo) & 0x0FFF;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("=== AS5600 bench: burst vs split read ==="));
  Wire.begin();
  Wire.setClock(100000);   // slow enough for a BSS138 shifter

  Wire.beginTransmission(AS5600);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("Nothing at 0x36. Run as5600_simple for the bus scan."));
  }
  Serial.println(F("Turn the magnet one full turn, slowly. Any key resets span."));
  Serial.println(F("burst  split  deg      span    magnet  agc  magnitude"));
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    g_haveFirst = false;
    Serial.println(F("-- span reset --"));
  }

  uint16_t burst = 0, split = 0, mag = 0;
  uint8_t st = 0, agc = 0;
  const bool okB = readBurst(0x0C, &burst);
  const bool okS = readSplit(&split);
  const bool okM = readBurst(0x1B, &mag);    // MAGNITUDE, 12 bits
  const bool okT = readOne(0x0B, &st);       // STATUS
  const bool okA = readOne(0x1A, &agc);      // AGC

  if (!okB || !okS || !okM || !okT || !okA) {
    Serial.println(F("read failed -- check wiring, then run as5600_simple"));
    delay(500);
    return;
  }

  if (!g_haveFirst) { g_first = burst; g_lo = 0; g_hi = 0; g_haveFirst = true; }
  const int16_t off = angleDiff(burst, g_first);
  if (off < g_lo) g_lo = off;
  if (off > g_hi) g_hi = off;
  const float spanDeg = (float)(g_hi - g_lo) * (360.0f / 4096.0f);

  Serial.print(burst);                          Serial.print(F("\t"));
  Serial.print(split);
  // A difference of a count or two is the angle moving between the two reads.
  if (abs(angleDiff(burst, split)) > 4) Serial.print('!');
  Serial.print(F("\t"));
  Serial.print(burst * (360.0f / 4096.0f), 2);  Serial.print(F("\t"));
  Serial.print(spanDeg, 1);                     Serial.print(F("\t"));
  if      (!(st & 0x20)) Serial.print(F("NONE"));
  else if (st & 0x10)    Serial.print(F("weak"));
  else if (st & 0x08)    Serial.print(F("strong"));
  else                   Serial.print(F("ok"));
  Serial.print(F("\t"));
  Serial.print(agc);                            Serial.print(F("\t"));
  Serial.println(mag);

  delay(250);
}
