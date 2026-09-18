// ===========================================================================
//  as5600_test -- encoder bring-up, before any of it goes near the motor
// ===========================================================================
//  Three questions, in order, and the loop cannot be trusted until all three
//  have answers:
//
//    1. IS THE MAGNET MOUNTED RIGHT?  The AS5600 will happily report a
//       plausible-looking angle from a magnet that is too far away, off
//       centre, or axially magnetised instead of diametrically. It reports
//       all of that in STATUS and AGC, and this sketch shows both live. A
//       reading that looks fine on the bench and drifts on the mount is
//       almost always a magnet that was never in spec.
//
//    2. WHICH WAY DOES IT COUNT?  Push the boom the way the driver calls
//       positive and watch whether raw goes up or down. That answers
//       ENCODER_INVERT in actuator_v2/Config.h. Getting it wrong turns the
//       PID into positive feedback, and the first move runs to a cam at
//       whatever duty the loop asked for.
//
//    3. DOES THE TRAVEL STRADDLE THE WRAP?  Sweep the full mechanical range
//       by hand and this records the extremes. If the span it reports is
//       nearly the whole circle when the boom only moved a few degrees, the
//       range crosses the 0/4095 rollover and the magnet needs rotating on
//       its hub.
//
//  There is no motor code here on purpose. Nothing in this sketch can move
//  anything, so it is safe to run with the driver unpowered.

#include <Wire.h>

const uint8_t AS5600_ADDR = 0x36;
const uint16_t COUNTS = 4096;
const float DEG_PER_COUNT = 360.0f / (float)COUNTS;

const uint16_t PRINT_MS = 200;

static uint16_t g_min = 0xFFFF;
static uint16_t g_max = 0;
static bool     g_sweeping = false;
static uint16_t g_last = 0;
static bool     g_havePrev = false;
static unsigned long g_lastPrint = 0;

bool readReg(uint8_t reg, uint8_t n, uint8_t *buf) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool readAngle(uint16_t *out) {
  uint8_t b[2];
  if (!readReg(0x0C, 2, b)) return false;
  *out = (((uint16_t)b[0] << 8) | b[1]) & 0x0FFF;
  return true;
}

void printDeg(float v) {
  long milli = (long)(v * 1000.0f + (v >= 0 ? 0.5f : -0.5f));
  if (milli < 0) { Serial.print('-'); milli = -milli; }
  Serial.print(milli / 1000);
  Serial.print('.');
  long frac = milli % 1000;
  if (frac < 100) Serial.print('0');
  if (frac < 10)  Serial.print('0');
  Serial.print(frac);
}

// Signed shortest way round -- the same helper actuator_v2 uses, so a range
// that crosses the wrap still reports a sane span.
int16_t angleDiff(uint16_t a, uint16_t b) {
  int16_t d = (int16_t)a - (int16_t)b;
  const int16_t half = (int16_t)(COUNTS / 2);
  if (d >  half) d -= (int16_t)COUNTS;
  if (d < -half) d += (int16_t)COUNTS;
  return d;
}

void reportMagnet() {
  uint8_t st, agc;
  if (!readReg(0x0B, 1, &st))  { Serial.println(F("STATUS read failed")); return; }
  if (!readReg(0x1A, 1, &agc)) { Serial.println(F("AGC read failed"));    return; }

  Serial.print(F("magnet: "));
  if      (!(st & 0x20)) Serial.print(F("NOT DETECTED"));
  else if (st & 0x10)    Serial.print(F("TOO WEAK -- move closer"));
  else if (st & 0x08)    Serial.print(F("TOO STRONG -- move away"));
  else                   Serial.print(F("ok"));

  // AGC runs 0-255 on a 5 V part. Mid-scale is the target: pinned at either
  // end means the gap is at the edge of what the part can compensate for,
  // and it will fall out of range with temperature.
  Serial.print(F("   agc="));
  Serial.print(agc);
  if (agc < 32 || agc > 224) Serial.print(F("  <-- near the end of its range"));
  Serial.println();
}

void printHelp() {
  Serial.println(F("  m   magnet status + AGC"));
  Serial.println(F("  b   begin sweep -- then move the boom by hand"));
  Serial.println(F("  x   end sweep and report the span"));
  Serial.println(F("  z   reset the sweep extremes"));
  Serial.println(F("  s   one sample"));
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("as5600_test -- encoder bring-up, no motor code here"));
  Wire.begin();
  Wire.setClock(400000);

  uint16_t raw;
  if (!readAngle(&raw)) {
    Serial.println(F("nothing answered at 0x36."));
    Serial.println(F("SDA/SCL are A4/A5 and cannot be moved. Check the level shifter."));
  } else {
    Serial.print(F("raw=")); Serial.println(raw);
    reportMagnet();
  }
  printHelp();
}

void loop() {
  if (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == 'm') reportMagnet();
    else if (c == 'b') {
      g_min = 0xFFFF; g_max = 0; g_sweeping = true; g_havePrev = false;
      Serial.println(F("sweeping -- move the boom slowly to both stops"));
    }
    else if (c == 'x') {
      g_sweeping = false;
      if (g_min == 0xFFFF) { Serial.println(F("no samples")); }
      else {
        const int16_t span = angleDiff(g_max, g_min);
        Serial.print(F("min=")); Serial.print(g_min);
        Serial.print(F(" max=")); Serial.print(g_max);
        Serial.print(F(" span=")); Serial.print(abs(span));
        Serial.print(F(" counts = ")); printDeg(fabs(span * DEG_PER_COUNT));
        Serial.println(F(" deg"));
        Serial.print(F("suggested ENCODER_ZERO_DEFAULT = "));
        Serial.println((uint16_t)(((long)g_min + (long)g_min + span) / 2) & 0x0FFF);
        if (g_min < 200 || g_max > (COUNTS - 200)) {
          Serial.println(F("WARNING: range sits near the 0/4095 wrap."));
          Serial.println(F("Rotate the magnet on its hub so travel centres near 2048."));
        }
      }
    }
    else if (c == 'z') { g_min = 0xFFFF; g_max = 0; g_havePrev = false; }
    else if (c == 's' || c == '?') {
      uint16_t raw;
      if (readAngle(&raw)) { Serial.print(F("raw=")); Serial.println(raw); }
      if (c == '?') printHelp();
    }
  }

  const unsigned long now = millis();
  if (now - g_lastPrint < PRINT_MS) return;
  g_lastPrint = now;

  uint16_t raw;
  if (!readAngle(&raw)) { Serial.println(F("read failed")); return; }

  if (g_sweeping) {
    if (g_min == 0xFFFF) { g_min = raw; g_max = raw; }
    else {
      if (angleDiff(raw, g_min) < 0) g_min = raw;
      if (angleDiff(raw, g_max) > 0) g_max = raw;
    }
    Serial.print(F("raw=")); Serial.print(raw);
    if (g_havePrev) {
      const int16_t d = angleDiff(raw, g_last);
      Serial.print(F("  step="));
      Serial.print(d);
      if (d > 0) Serial.print(F(" (counting UP)"));
      if (d < 0) Serial.print(F(" (counting DOWN)"));
    }
    Serial.println();
    g_last = raw;
    g_havePrev = true;
  }
}
