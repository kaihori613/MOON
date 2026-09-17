// ===========================================================================
//  as5600_test  --  bring-up and characterisation for the AS5600 encoder
// ===========================================================================
//
//  Same job as reed_switch_test/: not "does the part answer" but "what are
//  its numbers". A green light tells you nothing you can put in Config.h.
//  What this produces is an air gap you have set deliberately, a measured
//  noise floor in degrees, and the sample count that is actually worth
//  averaging -- all of which are guesses until somebody measures them.
//
//  WHAT THE AS5600 IS
//
//  A 12-bit contactless magnetic rotary encoder. It watches a magnet spinning
//  above the package and reports absolute angle over I2C: 4096 counts per
//  revolution, 0.0879 deg per count. Absolute, so no homing. Readable while
//  MOVING, which is the one thing the accelerometer structurally cannot do.
//
//  THE MAGNET IS THE WHOLE GAME
//
//  It must be DIAMETRICALLY magnetised -- poles across the diameter, not
//  through the thickness. A through-thickness magnet does not fail loudly:
//  you get a number that moves, just not linearly with angle, and nothing in
//  a naive sketch would tell you. Typical part is 6 mm x 2.5 mm, centred over
//  the package within ~0.25 mm, at 0.5-3 mm of air gap.
//
//  The part tells you about its own magnet, and that is what 'm' is for:
//    AGC        automatic gain. High = weak field (too far), low = strong
//               field (too close). Range is 0-255 at 5 V, 0-128 at 3.3 V,
//               and the target is the MIDDLE, which leaves headroom both
//               ways for temperature and mechanical slop.
//    MAGNITUDE  the CORDIC magnitude of the field it resolved.
//    MD/ML/MH   magnet detected / too weak / too strong.
//
//  Set the gap with 'm' running and watch AGC, rather than measuring with a
//  ruler and hoping.
//
//  NOT RUN ON HARDWARE. The register map is from the datasheet and has not
//  been checked against silicon. Nothing here drives a motor, so the worst
//  case is a sketch that prints nothing.

#include <Arduino.h>
#include <Wire.h>

// ===========================================================================
//  CONFIG
// ===========================================================================

// Fixed in silicon. Not selectable, so only one AS5600 per bus without a mux.
const uint8_t AS5600_ADDR = 0x36;

// Sets what a healthy AGC looks like: the part's gain range is 0-255 at 5 V
// and only 0-128 at 3.3 V, so the target midpoint halves with the supply.
#define AS5600_SUPPLY_5V 1

#if AS5600_SUPPLY_5V
  const uint8_t AGC_MAX = 255;
#else
  const uint8_t AGC_MAX = 128;
#endif
const uint8_t AGC_TARGET = AGC_MAX / 2;
const uint8_t AGC_TOLERANCE = AGC_MAX / 4;   // "close enough to the middle"

const uint16_t MONITOR_PERIOD_MS = 100;      // 10 Hz is plenty for gap tuning
const unsigned long NOISE_DEFAULT_MS = 10000;
const unsigned long DRIFT_DEFAULT_MS = 300000;   // 5 min
const uint16_t DRIFT_PERIOD_MS = 2000;

const float DEG_PER_LSB = 360.0f / 4096.0f;  // 0.087890625

// ===========================================================================
//  REGISTERS
// ===========================================================================

const uint8_t REG_ZMCO       = 0x00;   // how many times ZPOS/MPOS were burned
const uint8_t REG_ZPOS_H     = 0x01;
const uint8_t REG_MPOS_H     = 0x03;
const uint8_t REG_MANG_H     = 0x05;
const uint8_t REG_CONF_H     = 0x07;
const uint8_t REG_STATUS     = 0x0B;
const uint8_t REG_RAW_ANGLE  = 0x0C;   // unscaled 12-bit
const uint8_t REG_ANGLE      = 0x0E;   // after ZPOS/MPOS scaling + hysteresis
const uint8_t REG_AGC        = 0x1A;
const uint8_t REG_MAGNITUDE  = 0x1B;

// STATUS bits.
const uint8_t ST_MH = 0x08;   // gain minimum overflow -- magnet too STRONG
const uint8_t ST_ML = 0x10;   // gain maximum overflow -- magnet too WEAK
const uint8_t ST_MD = 0x20;   // magnet detected

// ===========================================================================
//  I2C
// ===========================================================================

bool g_present = false;

bool readBytes(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// Returns 0xFFFF on a bus failure, which is outside the 12-bit range and so
// cannot be mistaken for a reading.
uint16_t read12(uint8_t reg) {
  uint8_t b[2];
  if (!readBytes(reg, b, 2)) return 0xFFFF;
  return (((uint16_t)b[0] << 8) | b[1]) & 0x0FFF;
}

uint8_t read8(uint8_t reg) {
  uint8_t b;
  if (!readBytes(reg, &b, 1)) return 0xFF;
  return b;
}

// ===========================================================================
//  ANGLE HANDLING
// ===========================================================================
//
//  The encoder wraps 4095 -> 0. Over a +/-15 deg arc you will probably never
//  cross it, but "probably never" is exactly how a coordinate system ends up
//  jumping 360 deg in the middle of a measurement. Everything below works in
//  UNWRAPPED counts: the wrap is removed as samples arrive, so a run that
//  straddles zero reads as continuous.

long  g_unwrapped = 0;      // counts, continuous across the wrap
long  g_zero      = 0;      // subtracted from the readout
bool  g_haveLast  = false;
uint16_t g_last   = 0;

void resetUnwrap(uint16_t raw) {
  g_last = raw;
  g_unwrapped = raw;
  g_haveLast = true;
}

// Feed every raw sample through this, in order. Half a revolution of apparent
// jump between consecutive samples is read as a wrap rather than as motion --
// which is correct as long as you are not sampling slower than the dish can
// turn half a turn, and this dish moves +/-15 deg total.
long feedUnwrap(uint16_t raw) {
  if (!g_haveLast) { resetUnwrap(raw); return g_unwrapped; }
  int32_t d = (int32_t)raw - (int32_t)g_last;
  if (d >  2048) d -= 4096;
  if (d < -2048) d += 4096;
  g_unwrapped += d;
  g_last = raw;
  return g_unwrapped;
}

float countsToDeg(long counts) { return (float)counts * DEG_PER_LSB; }

// ===========================================================================
//  REPORTING
// ===========================================================================

void printAgcBar(uint8_t agc) {
  // 20 cells across the part's full gain range, with the target marked. The
  // point is to make "move it closer / further" obvious at a glance while
  // your hands are on the bracket.
  const uint8_t cells = 20;
  uint8_t fill = (uint16_t)agc * cells / (AGC_MAX + 1);
  uint8_t mark = cells / 2;
  Serial.print('[');
  for (uint8_t i = 0; i < cells; i++) {
    if (i == mark)      Serial.print(i < fill ? '#' : '|');
    else                Serial.print(i < fill ? '#' : '.');
  }
  Serial.print(']');
}

void printMagnetVerdict(uint8_t status, uint8_t agc) {
  if (!(status & ST_MD)) {
    Serial.println(F("  NO MAGNET DETECTED."));
    Serial.println(F("  Either nothing is over the package, the gap is far too"));
    Serial.println(F("  large, or the magnet is magnetised through its THICKNESS"));
    Serial.println(F("  instead of across its diameter. Check that first -- it is"));
    Serial.println(F("  the most common wrong part and it fails silently."));
    return;
  }
  if (status & ST_ML) {
    Serial.println(F("  MAGNET TOO WEAK (gain maxed). Closer, or a stronger magnet."));
    return;
  }
  if (status & ST_MH) {
    Serial.println(F("  MAGNET TOO STRONG (gain floored). Further away."));
    return;
  }
  const int16_t off = (int16_t)agc - (int16_t)AGC_TARGET;
  if (abs(off) <= (int16_t)AGC_TOLERANCE) {
    Serial.println(F("  Magnet OK, gain near mid-range. Good gap."));
  } else if (off > 0) {
    Serial.println(F("  Magnet OK but gain is high -- field is weak for the range."));
    Serial.println(F("  Works, but little headroom. Move it CLOSER."));
  } else {
    Serial.println(F("  Magnet OK but gain is low -- field is strong for the range."));
    Serial.println(F("  Works, but little headroom. Move it FURTHER."));
  }
}

void cmdStatus() {
  const uint8_t status = read8(REG_STATUS);
  const uint8_t agc    = read8(REG_AGC);
  const uint16_t mag   = read12(REG_MAGNITUDE);
  const uint16_t raw   = read12(REG_RAW_ANGLE);
  const uint16_t ang   = read12(REG_ANGLE);
  const uint8_t zmco   = read8(REG_ZMCO);
  const uint16_t conf  = read12(REG_CONF_H);
  const uint16_t zpos  = read12(REG_ZPOS_H);
  const uint16_t mpos  = read12(REG_MPOS_H);
  const uint16_t mang  = read12(REG_MANG_H);

  if (raw == 0xFFFF) { Serial.println(F("  bus read failed")); return; }

  Serial.print(F("  RAW_ANGLE  ")); Serial.print(raw);
  Serial.print(F("  =  "));         Serial.print(countsToDeg(raw), 3);
  Serial.println(F(" deg"));

  Serial.print(F("  ANGLE      ")); Serial.print(ang);
  Serial.print(F("  =  "));         Serial.print(countsToDeg(ang), 3);
  Serial.println(F(" deg"));

  // A difference between the two is not a fault -- it is ZPOS/MPOS scaling
  // and the output hysteresis doing their job. It matters because a
  // second-hand or pre-programmed board can arrive with those burned, and
  // then ANGLE is a scaled view of a range somebody else chose.
  if (raw != ang) {
    Serial.println(F("  (RAW and ANGLE differ: scaling or hysteresis is active)"));
  }

  Serial.print(F("  STATUS     0x")); Serial.print(status, HEX);
  Serial.print(F("   MD="));  Serial.print((status & ST_MD) ? 1 : 0);
  Serial.print(F(" ML="));    Serial.print((status & ST_ML) ? 1 : 0);
  Serial.print(F(" MH="));    Serial.println((status & ST_MH) ? 1 : 0);

  Serial.print(F("  AGC        ")); Serial.print(agc);
  Serial.print(F(" / "));           Serial.print(AGC_MAX);
  Serial.print(F("   target ~"));   Serial.print(AGC_TARGET);
  Serial.print(F("   "));           printAgcBar(agc); Serial.println();

  Serial.print(F("  MAGNITUDE  ")); Serial.println(mag);

  Serial.print(F("  CONF 0x"));   Serial.print(conf, HEX);
  Serial.print(F("  ZPOS "));     Serial.print(zpos);
  Serial.print(F("  MPOS "));     Serial.print(mpos);
  Serial.print(F("  MANG "));     Serial.print(mang);
  Serial.print(F("  ZMCO "));     Serial.println(zmco);
  if (zmco > 0) {
    Serial.println(F("  NOTE: ZMCO is non-zero -- someone has burned settings"));
    Serial.println(F("  into this part's OTP. That is permanent. Use RAW_ANGLE."));
  }

  printMagnetVerdict(status, agc);
}

// ===========================================================================
//  MODES
// ===========================================================================

// Any inbound byte aborts a running mode. Same fingering as the rest of the
// project: whatever is running, hitting a key stops it.
bool aborted() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  return true;
}

// Live view, for setting the air gap with your hands on the bracket.
void cmdMonitor() {
  Serial.println(F("  Live. Adjust the gap and watch AGC settle mid-range."));
  Serial.println(F("  Any key stops."));
  Serial.println();
  g_haveLast = false;

  unsigned long next = 0;
  while (!aborted()) {
    const unsigned long now = millis();
    if ((long)(now - next) < 0) continue;
    next = now + MONITOR_PERIOD_MS;

    const uint16_t raw = read12(REG_RAW_ANGLE);
    if (raw == 0xFFFF) { Serial.println(F("  bus read failed")); return; }
    const uint8_t status = read8(REG_STATUS);
    const uint8_t agc    = read8(REG_AGC);
    const long pos = feedUnwrap(raw) - g_zero;

    Serial.print(F("  "));   Serial.print(raw);
    Serial.print(F("  "));
    // Signed, relative to the zero, which is what you care about on a +/-15
    // deg arc where the absolute number is meaningless.
    const float d = countsToDeg(pos);
    if (d >= 0) Serial.print(' ');
    Serial.print(d, 3);
    Serial.print(F(" deg   AGC "));
    if (agc < 100) Serial.print(' ');
    if (agc < 10)  Serial.print(' ');
    Serial.print(agc); Serial.print(' ');
    printAgcBar(agc);
    if (!(status & ST_MD)) Serial.print(F("  NO MAGNET"));
    else if (status & ST_ML) Serial.print(F("  TOO WEAK"));
    else if (status & ST_MH) Serial.print(F("  TOO STRONG"));
    Serial.println();
  }
  Serial.println(F("  stopped"));
}

// The measurement Config.h is actually waiting on.
void cmdNoise(unsigned long ms) {
  if (ms == 0) ms = NOISE_DEFAULT_MS;
  Serial.print(F("  Noise floor for ")); Serial.print(ms / 1000.0, 1);
  Serial.println(F(" s. DO NOT TOUCH the mount."));
  Serial.println(F("  Any key aborts."));

  g_haveLast = false;
  const uint16_t first = read12(REG_RAW_ANGLE);
  if (first == 0xFFFF) { Serial.println(F("  bus read failed")); return; }
  const long ref = feedUnwrap(first);

  // Accumulate deviations from the first sample, not the samples themselves:
  // 4095^2 times a few thousand overflows an int32, deviations squared do not.
  int32_t sum = 0, sumsq = 0;
  int32_t lo = 0, hi = 0;
  uint32_t n = 0;

  const unsigned long endAt = millis() + ms;
  while ((long)(millis() - endAt) < 0) {
    if (aborted()) { Serial.println(F("  aborted")); return; }
    const uint16_t raw = read12(REG_RAW_ANGLE);
    if (raw == 0xFFFF) continue;
    const int32_t d = feedUnwrap(raw) - ref;
    sum += d; sumsq += d * d; n++;
    if (d < lo) lo = d;
    if (d > hi) hi = d;
  }

  if (n < 2) { Serial.println(F("  no samples")); return; }

  const float mean = (float)sum / (float)n;
  float var = (float)sumsq / (float)n - mean * mean;
  if (var < 0) var = 0;
  const float sd = sqrt(var);

  Serial.println();
  Serial.print(F("  samples      ")); Serial.println(n);
  Serial.print(F("  sample rate  ")); Serial.print((float)n / (ms / 1000.0f), 0);
  Serial.println(F(" Hz"));
  Serial.print(F("  sigma        ")); Serial.print(sd, 3);
  Serial.print(F(" LSB  =  "));       Serial.print(sd * DEG_PER_LSB, 4);
  Serial.println(F(" deg"));
  Serial.print(F("  peak-to-peak ")); Serial.print(hi - lo);
  Serial.print(F(" LSB  =  "));       Serial.print((hi - lo) * DEG_PER_LSB, 4);
  Serial.println(F(" deg"));
  Serial.print(F("  drift        ")); Serial.print(mean, 2);
  Serial.println(F(" LSB over the run"));
  Serial.println();
  Serial.println(F("  One LSB is 0.0879 deg. The 8 ft dish has a 5.08 deg beam"));
  Serial.println(F("  at 1694 MHz, where 1 deg off boresight costs 0.46 dB, so"));
  Serial.println(F("  anything under ~0.1 deg here is far inside the budget."));
  Serial.println(F("  If sigma is large, suspect the gap or a nearby motor"));
  Serial.println(F("  before suspecting the part."));
}

// How much does averaging actually buy? Past some N the noise stops being
// independent and averaging stops helping -- and that N is the only honest
// way to pick a sample count.
void cmdAveraging() {
  Serial.println(F("  Averaging curve. DO NOT TOUCH the mount."));
  Serial.println();
  Serial.println(F("     N    sigma(LSB)   sigma(deg)"));

  const uint8_t trials = 32;
  for (uint8_t nAvg = 1; nAvg <= 128; nAvg *= 2) {
    if (aborted()) { Serial.println(F("  aborted")); return; }

    g_haveLast = false;
    const uint16_t f = read12(REG_RAW_ANGLE);
    if (f == 0xFFFF) { Serial.println(F("  bus read failed")); return; }
    const long ref = feedUnwrap(f);

    float sum = 0, sumsq = 0;
    for (uint8_t t = 0; t < trials; t++) {
      int32_t acc = 0;
      for (uint8_t i = 0; i < nAvg; i++) {
        const uint16_t raw = read12(REG_RAW_ANGLE);
        acc += feedUnwrap(raw) - ref;
      }
      const float m = (float)acc / (float)nAvg;
      sum += m; sumsq += m * m;
    }
    const float mean = sum / trials;
    float var = sumsq / trials - mean * mean;
    if (var < 0) var = 0;
    const float sd = sqrt(var);

    Serial.print(F("   "));
    if (nAvg < 100) Serial.print(' ');
    if (nAvg < 10)  Serial.print(' ');
    Serial.print(nAvg);
    Serial.print(F("      "));  Serial.print(sd, 3);
    Serial.print(F("        ")); Serial.println(sd * DEG_PER_LSB, 4);
  }
  Serial.println();
  Serial.println(F("  Ideally sigma falls as 1/sqrt(N). Where the column stops"));
  Serial.println(F("  falling is where averaging stops paying, and that is the"));
  Serial.println(F("  sample count worth putting in Config.h -- not the next"));
  Serial.println(F("  power of two after it."));
}

// Long, slow log. Thermal drift and a bracket relaxing overnight both look
// like a slow ramp, and neither shows up in a ten-second noise run.
void cmdDrift(unsigned long ms) {
  if (ms == 0) ms = DRIFT_DEFAULT_MS;
  Serial.print(F("  Drift log for ")); Serial.print(ms / 60000.0, 1);
  Serial.println(F(" min. Any key stops."));
  Serial.println(F("DRIFT,seconds,raw,deg_from_zero,agc"));

  g_haveLast = false;
  const unsigned long startAt = millis();
  const unsigned long endAt = startAt + ms;
  unsigned long next = startAt;

  while ((long)(millis() - endAt) < 0) {
    if (aborted()) { Serial.println(F("  stopped")); return; }
    const unsigned long now = millis();
    if ((long)(now - next) < 0) continue;
    next = now + DRIFT_PERIOD_MS;

    const uint16_t raw = read12(REG_RAW_ANGLE);
    if (raw == 0xFFFF) continue;
    const long pos = feedUnwrap(raw) - g_zero;

    Serial.print(F("DRIFT,"));
    Serial.print((now - startAt) / 1000.0, 1);       Serial.print(',');
    Serial.print(raw);                               Serial.print(',');
    Serial.print(countsToDeg(pos), 4);               Serial.print(',');
    Serial.println(read8(REG_AGC));
  }
  Serial.println(F("  drift log done"));
}

void cmdZero() {
  const uint16_t raw = read12(REG_RAW_ANGLE);
  if (raw == 0xFFFF) { Serial.println(F("  bus read failed")); return; }
  g_haveLast = false;
  g_zero = feedUnwrap(raw);
  Serial.print(F("  zero = here  (raw "));
  Serial.print(raw);
  Serial.println(F(")"));
  Serial.println(F("  This lives in RAM only. It is a convenience for reading"));
  Serial.println(F("  a small arc, not a calibration -- nothing is burned."));
}

void printHelp() {
  Serial.println(F("--------------------------------------------------------"));
  Serial.println(F("  s          status: angle, magnet, AGC, burned settings"));
  Serial.println(F("  m          live monitor -- use this to set the air gap"));
  Serial.println(F("  z          zero here (RAM only)"));
  Serial.println(F("  n [ms]     noise floor, default 10 s, hands off"));
  Serial.println(F("  a          averaging curve: sigma vs sample count"));
  Serial.println(F("  w [ms]     drift log as CSV, default 5 min"));
  Serial.println(F("  ?          this help"));
  Serial.println(F("  <any key>  stops a running mode"));
  Serial.println(F("--------------------------------------------------------"));
}

// ===========================================================================
//  CONSOLE
// ===========================================================================

char    g_line[24];
uint8_t g_len = 0;

bool parseULong(const char* s, unsigned long* out) {
  while (*s == ' ') s++;
  if (*s < '0' || *s > '9') return false;
  unsigned long v = 0;
  while (*s >= '0' && *s <= '9') { v = v * 10 + (unsigned long)(*s - '0'); s++; }
  *out = v;
  return true;
}

void handleCommand(char* line) {
  if (line[0] == '\0') { printHelp(); return; }

  const char cmd = line[0];
  const char* arg = line + 1;
  unsigned long v = 0;

  if (!g_present && cmd != '?') {
    Serial.println(F("  no AS5600 answering at 0x36 -- check wiring first"));
    return;
  }

  switch (cmd) {
    case 's': cmdStatus(); break;
    case 'm': cmdMonitor(); break;
    case 'z': cmdZero(); break;
    case 'n': cmdNoise(parseULong(arg, &v) ? v : 0); break;
    case 'a': cmdAveraging(); break;
    case 'w': cmdDrift(parseULong(arg, &v) ? v : 0); break;
    case '?': printHelp(); break;
    default:
      Serial.println(F("  unknown -- ? for help"));
      break;
  }
}

void pollSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      g_line[g_len] = '\0';
      handleCommand(g_line);
      g_len = 0;
    } else if (g_len < sizeof(g_line) - 1) {
      g_line[g_len++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Wire.begin();

  Serial.println();
  Serial.println(F("=== AS5600 bench test ==="));
  Serial.println(F("SDA A4, SCL A5, address 0x36 (fixed)."));
  Serial.println();

  // Probe before anything else. Every other command is meaningless if the
  // part is not on the bus, and a silent failure here would otherwise show up
  // as a plausible-looking angle of zero.
  Wire.beginTransmission(AS5600_ADDR);
  g_present = (Wire.endTransmission() == 0);

  if (!g_present) {
    Serial.println(F("NOT FOUND at 0x36."));
    Serial.println(F("  - SDA/SCL swapped, or not on A4/A5"));
    Serial.println(F("  - no power, or a 3.3 V board on 5 V logic without a"));
    Serial.println(F("    level shifter (an Uno needs ~3.0 V to read a HIGH,"));
    Serial.println(F("    so a 3.3 V bus has almost no margin)"));
    Serial.println(F("  - missing pull-ups on a long run"));
  } else {
    Serial.println(F("Found at 0x36."));
    Serial.println();
    cmdStatus();
  }

  Serial.println();
  Serial.println(F("Set the gap with 'm' before trusting any other number."));
  printHelp();
}

void loop() {
  pollSerial();
}
