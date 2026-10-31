// ===========================================================================
//  MOON controller_v1
// ===========================================================================
//  Yaw positioning closed around an absolute encoder on the pivot.
//
//  The one thing to understand before reading further: THIS SKETCH DOES NOT
//  CHASE SNR. Signal strength against pointing angle is a peak, not a ramp --
//  the same error reading occurs on both sides of it, so a PID fed SNR has no
//  sign to act on and cannot know which way to move. Peaking is a SEARCH, it
//  lives on the host, and it talks to this sketch by handing it target
//  angles. What runs here is the inner loop: a servo on angle, where the
//  error does have a sign.
//
//  Layout of this file, top to bottom:
//    hardware    encoder, kill switch, buzzer, motor driver
//    control     PID, state, faults, EEPROM
//    motion      health checks, moves, panel buttons, auto-calibration
//    console     serial commands
//    setup/loop
//
//  Everything that is not fitted on this build (reed, limit switches, ref
//  switch, step test) is compiled out behind a USE_* switch in Config.h and
//  may be re-enabled in the future.
//
//  Compile-checked and simulated against a model of the dish; not yet run on
//  hardware. First power-up with the actuator unloaded.

#include "Config.h"
#include "Types.h"
#include <Wire.h>
#if USE_EEPROM
  #include <EEPROM.h>
#endif
#if USE_WATCHDOG
  #include <avr/wdt.h>
#endif

// ---------------------------------------------------------------------------
//  Reed -- witness only (USE_REED, disabled)
// ---------------------------------------------------------------------------
//  Deliberately not integrated into anything. It counts, and the count is
//  only ever compared against itself over a window to answer "is the motor
//  turning". Position comes from the encoder.

#if USE_REED
volatile unsigned long g_reedPulses  = 0;
volatile unsigned long g_reedLastUs  = 0;
volatile unsigned long g_reedLastMs  = 0;

void onReedEdge() {
  const unsigned long now = micros();
  if (now - g_reedLastUs < REED_DEBOUNCE_US) return;   // bounce, or pickup
  g_reedLastUs = now;
  g_reedLastMs = millis();
  g_reedPulses++;
}

unsigned long reedPulses() {
  noInterrupts();
  const unsigned long n = g_reedPulses;
  interrupts();
  return n;
}

unsigned long reedLastMs() {
  noInterrupts();
  const unsigned long n = g_reedLastMs;
  interrupts();
  return n;
}

static unsigned long g_winPulses   = 0;   // linkage window
static float         g_winStartDeg = 0.0f;
#endif

// ---------------------------------------------------------------------------
//  AS5600
// ---------------------------------------------------------------------------

static uint8_t  g_encFails   = 0;
static uint16_t g_encRaw     = 0;
static bool     g_encOk      = false;
static int16_t  g_zeroCount  = ENCODER_ZERO_DEFAULT;
static bool     g_liveAngleStream = false;   // 'L': stream the angle to the host

// Returns false and leaves *buf untouched on any bus error. Callers must
// treat that as fatal mid-move: a stale angle in the loop is worse than a
// stopped motor.
bool as5600Read(uint8_t reg, uint8_t n, uint8_t *buf) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  bool ok = (Wire.endTransmission(false) == 0) &&
            (Wire.requestFrom((uint8_t)AS5600_ADDR, n) == n);
  if (ok) {
    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  }
#ifdef WIRE_HAS_TIMEOUT
  // A read that timed out is a failed read, whatever the bytes look like.
  if (Wire.getWireTimeoutFlag()) { Wire.clearWireTimeoutFlag(); ok = false; }
#endif
  return ok;
}

// Register 0x0C is ANGLE, the filtered output. (0x0E is the unfiltered RAW
// ANGLE; the loop was tuned against the filtered one, so it stays.)
bool as5600Angle(uint16_t *out) {
  uint8_t b[2];
  if (!as5600Read(0x0C, 2, b)) return false;
  *out = (((uint16_t)b[0] << 8) | b[1]) & 0x0FFF;
  return true;
}

// STATUS bit 5 = MD (magnet detected), 4 = ML (too weak), 3 = MH (too strong)
bool as5600Status(uint8_t *out) { return as5600Read(0x0B, 1, out); }
bool as5600Agc(uint8_t *out)    { return as5600Read(0x1A, 1, out); }

// Signed shortest way round, so a working range that happens to straddle the
// 0/4095 wrap still produces a sane difference.
int16_t angleDiff(uint16_t a, uint16_t b) {
  int16_t d = (int16_t)a - (int16_t)b;
  const int16_t half = (int16_t)(AS5600_COUNTS / 2);
  if (d >  half) d -= (int16_t)AS5600_COUNTS;
  if (d < -half) d += (int16_t)AS5600_COUNTS;
  return d;
}

uint16_t wrapCount(long v) {
  v %= (long)AS5600_COUNTS;
  if (v < 0) v += (long)AS5600_COUNTS;
  return (uint16_t)v;
}

float countsToDeg(int16_t counts) {
#if ENCODER_INVERT
  return -(float)counts * DEG_PER_COUNT;
#else
  return  (float)counts * DEG_PER_COUNT;
#endif
}

int16_t degToCounts(float deg) {
#if ENCODER_INVERT
  const float c = -deg / DEG_PER_COUNT;
#else
  const float c =  deg / DEG_PER_COUNT;
#endif
  return (int16_t)(c + (c >= 0.0f ? 0.5f : -0.5f));
}

// Reads the encoder into the cached state. This is the ONLY place the angle
// is read: every consumer -- PID, buttons, calibration -- uses g_encRaw and
// positionDeg(), refreshed once per control tick. Returns false once the
// failure run reaches ENCODER_MAX_FAILS.
bool encoderPoll() {
  uint16_t raw;
  if (as5600Angle(&raw)) {
    g_encRaw = raw;
    g_encFails = 0;
    g_encOk = true;
    return true;
  }
  if (g_encFails < 255) g_encFails++;
  if (g_encFails >= ENCODER_MAX_FAILS) g_encOk = false;
  return g_encOk;
}

// Boot only: one failed read before the first good one must not fault.
bool encoderBoot() {
  for (uint8_t i = 0; i < 5; i++) {
    uint16_t raw;
    if (as5600Angle(&raw)) {
      g_encRaw = raw;
      g_encFails = 0;
      g_encOk = true;
      return true;
    }
    delay(5);
  }
  g_encOk = false;
  return false;
}

float positionDeg() {
  return countsToDeg(angleDiff(g_encRaw, (uint16_t)g_zeroCount));
}

// ---------------------------------------------------------------------------
//  Limit and reference switches (USE_LIMIT_SWITCHES / USE_REF_SWITCH)
// ---------------------------------------------------------------------------
//  Both are disabled on this build. With them off, every function here reads
//  false and driveSigned()'s limit interlock does nothing.

#define USE_INPUT_FILTER (USE_LIMIT_SWITCHES || USE_REF_SWITCH)

#if USE_INPUT_FILTER
const uint8_t IN_LIM_POS = 0, IN_LIM_NEG = 1, IN_REF = 2, IN_COUNT = 3;
static bool          g_inState[IN_COUNT];   // filtered: true = tripped / active
static bool          g_inRaw[IN_COUNT];     // last raw reading
static unsigned long g_inSince[IN_COUNT];   // when the raw reading last changed

#if USE_LIMIT_SWITCHES
static bool limitPinTripped(uint8_t pin) {
#if LIMITS_NORMALLY_CLOSED
  return digitalRead(pin) == HIGH;          // open, or the wire is broken
#else
  return digitalRead(pin) == LOW;
#endif
}
#endif

bool readInputRaw(uint8_t i) {
  switch (i) {
#if USE_LIMIT_SWITCHES
    case IN_LIM_POS: return limitPinTripped(PIN_LIM_POS);
    case IN_LIM_NEG: return limitPinTripped(PIN_LIM_NEG);
#endif
#if USE_REF_SWITCH
    case IN_REF:     return digitalRead(PIN_REF) == LOW;
#endif
    default:         return false;
  }
}

void filterInput(uint8_t i, unsigned long now, uint8_t onMs, uint8_t offMs) {
  const bool raw = readInputRaw(i);
  if (raw != g_inRaw[i]) { g_inRaw[i] = raw; g_inSince[i] = now; }
  if (raw != g_inState[i] && now - g_inSince[i] >= (raw ? onMs : offMs)) {
    g_inState[i] = raw;
  }
}

// Sampled every pass of loop(), so a real change is seen within a couple of
// milliseconds. A reading only becomes the filtered state once it has held for
// its time in Config.h; a shorter spike never gets through.
void pollInputs() {
  const unsigned long now = millis();
  filterInput(IN_LIM_POS, now, LIMIT_TRIP_MS, LIMIT_CLEAR_MS);
  filterInput(IN_LIM_NEG, now, LIMIT_TRIP_MS, LIMIT_CLEAR_MS);
  filterInput(IN_REF,     now, REF_STABLE_MS,  REF_STABLE_MS);
}

// At boot, trust what the pins say: a limit that is already tripped must
// count as tripped from the very first tick.
void initInputs() {
  const unsigned long now = millis();
  for (uint8_t i = 0; i < IN_COUNT; i++) {
    g_inRaw[i] = g_inState[i] = readInputRaw(i);
    g_inSince[i] = now;
  }
}
#else
void pollInputs() {}
void initInputs() {}
#endif

#if USE_LIMIT_SWITCHES
bool limitPos() { return g_inState[IN_LIM_POS]; }
bool limitNeg() { return g_inState[IN_LIM_NEG]; }
#else
bool limitPos() { return false; }
bool limitNeg() { return false; }
#endif

#if USE_REF_SWITCH
bool refActive() { return g_inState[IN_REF]; }
#else
bool refActive() { return false; }
#endif

// ---------------------------------------------------------------------------
//  Kill switch
// ---------------------------------------------------------------------------
//  See Config.h: it latches a fault, and it is a request to the firmware, not
//  a power cut. Deliberately NOT filtered: it acts on the first sample. A
//  spike can only stop the motor, which is the safe direction to be wrong in.

#if USE_KILL
bool killAsserted() {
#if KILL_IS_NORMALLY_CLOSED
  return digitalRead(PIN_KILL) == HIGH;   // thrown, or the wire is broken
#else
  return digitalRead(PIN_KILL) == LOW;
#endif
}
#else
bool killAsserted() { return false; }     // not fitted
#endif

// ---------------------------------------------------------------------------
//  Buzzer
// ---------------------------------------------------------------------------
//  tone() with a duration returns at once and Timer2 ends the note, so the
//  only state kept here is how many more beeps are owed and when the next one
//  starts. A new pattern replaces whatever was still playing.

#if USE_BUZZER
static uint16_t      g_beepMs   = 0;
static uint8_t       g_beepLeft = 0;
static unsigned long g_beepNext = 0;

void beep(uint8_t count, uint16_t ms) {
  g_beepMs = ms;
  g_beepLeft = count;
  g_beepNext = millis();
}

void buzzerTick() {
  if (g_beepLeft == 0) return;
  const unsigned long now = millis();
  if ((long)(now - g_beepNext) < 0) return;
  tone(PIN_BUZZER, BEEP_HZ, g_beepMs);
  g_beepLeft--;
  g_beepNext = now + 2UL * g_beepMs;      // the note, then an equal gap
}
#else
void beep(uint8_t count, uint16_t ms) { (void)count; (void)ms; }
void buzzerTick() {}
#endif

// ---------------------------------------------------------------------------
//  Motor
// ---------------------------------------------------------------------------

static int      g_duty       = 0;    // signed, what we last commanded
static int8_t   g_lastSign   = 0;
static bool     g_limitBlocked = false;  // last call was clipped by a limit
static unsigned long g_coastUntil = 0;

void motorOff() {
#if MOTOR_DRIVER == DRV_L298N
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_ENA, LOW);        // detach the PWM, whichever timer drove it
  digitalWrite(PIN_IN1, LOW);        // and coast rather than brake
  digitalWrite(PIN_IN2, LOW);
#elif MOTOR_DRIVER == DRV_HW039
  digitalWrite(PIN_RPWM, LOW);       // detach both PWMs before dropping EN,
  digitalWrite(PIN_LPWM, LOW);       // same order as the L298N branch
  digitalWrite(PIN_EN,   LOW);
#else
  analogWrite(PIN_PWM, 0);
  digitalWrite(PIN_PWM, LOW);
  digitalWrite(PIN_SLP, LOW);        // put the bridge to sleep
#endif
  g_duty = 0;
  g_lastSign = 0;
}

// Signed duty in, with every interlock applied on the way through. This is
// the ONLY place the motor is energised.
void driveSigned(int duty) {
  duty = constrain(duty, -(int)SPEED_MAX, (int)SPEED_MAX);

  // Limit switches are signal only, so THIS is what stops travel into a
  // tripped one. The flag lets the caller tell "we are escaping a limit"
  // (fine) from "we drove into one" (a fault). Always false while
  // USE_LIMIT_SWITCHES is 0.
  g_limitBlocked = false;
  if (limitPos() && duty > 0) { duty = 0; g_limitBlocked = true; }
  if (limitNeg() && duty < 0) { duty = 0; g_limitBlocked = true; }

  const int8_t sign = (duty > 0) ? 1 : (duty < 0 ? -1 : 0);

  // Never cross zero under power.
  if (sign != 0 && g_lastSign != 0 && sign != g_lastSign) {
    motorOff();
    g_coastUntil = millis() + REVERSE_DEAD_MS;
    return;
  }
  if (millis() < g_coastUntil) { motorOff(); return; }

  if (sign == 0) { motorOff(); return; }

#if MOTOR_DRIVER == DRV_L298N
  // Direction first, then enable: setting IN1/IN2 while ENA is already high
  // walks the bridge through a state it does not need to see.
  digitalWrite(PIN_IN1, sign > 0 ? HIGH : LOW);
  digitalWrite(PIN_IN2, sign > 0 ? LOW  : HIGH);
  analogWrite(PIN_ENA, (uint8_t)abs(duty));
#elif MOTOR_DRIVER == DRV_HW039
  // Idle side hard off before the driven side goes anywhere near on -- RPWM
  // and LPWM both live is the state that fights the motor against itself.
  if (sign > 0) {
    digitalWrite(PIN_LPWM, LOW);
    digitalWrite(PIN_EN,   HIGH);
    analogWrite(PIN_RPWM, (uint8_t)abs(duty));
  } else {
    digitalWrite(PIN_RPWM, LOW);
    digitalWrite(PIN_EN,   HIGH);
    analogWrite(PIN_LPWM, (uint8_t)abs(duty));
  }
#else
  digitalWrite(PIN_SLP, HIGH);
  digitalWrite(PIN_DIR, sign > 0 ? HIGH : LOW);
  analogWrite(PIN_PWM, (uint8_t)abs(duty));
#endif
  g_duty = duty;
  g_lastSign = sign;
}

// ---------------------------------------------------------------------------
//  PID
// ---------------------------------------------------------------------------

static float g_kp = KP_DEFAULT;
static float g_ki = KI_DEFAULT;
static float g_kd = KD_DEFAULT;
static float g_kff = KFF_DEFAULT;

// PidState is defined in Types.h.
static PidState g_pid = {0.0f, 0.0f, false, false};

void pidReset() {
  g_pid.integral = 0.0f;
  g_pid.primed = false;
  g_pid.holdIntegral = false;
}

// Returns unclamped signed duty. Clamping and the interlocks belong to
// driveSigned(); keeping them out of here means the saturation test below
// sees the real, unclamped demand.
float pidStep(float target, float meas, float dt) {
  const float e = target - meas;

  if (fabs(e) <= DEADBAND_DEG) {
    // Inside the deadband there is nothing worth doing. Dumping the
    // integrator here is what stops a backlash-y plant hunting: integral
    // action across a mechanical dead zone is the textbook limit cycle, and
    // the deadband is deliberately set wider than the backlash so that this
    // branch, not the integrator, is what ends every move.
    g_pid.integral = 0.0f;
    g_pid.prevMeas = meas;
    g_pid.primed = true;
    return 0.0f;
  }

  float u = g_kp * e;

  // Friction feedforward. Below breakaway the motor draws current, heats up
  // and does not turn, so every correction starts at the duty that actually
  // moves the thing and the gains only trim from there. On a plant this
  // stiction-dominated it does more work than the integrator does.
  u += (e > 0.0f) ? g_kff : -g_kff;

  // Conditional integration: stop winding whenever the last output was
  // already saturated, or the integrator fills up during every long move and
  // then drives a huge overshoot at the end of it.
  if (!g_pid.holdIntegral) {
    g_pid.integral += e * dt;
    if (g_pid.integral >  I_MAX) g_pid.integral =  I_MAX;
    if (g_pid.integral < -I_MAX) g_pid.integral = -I_MAX;
  }
  u += g_ki * g_pid.integral;

  // Derivative on the MEASUREMENT, not the error: differentiating the error
  // puts a spike through the output every time the host hands us a new
  // target, which has nothing to do with what the plant is doing.
  if (g_kd != 0.0f && g_pid.primed && dt > 0.0f) {
    u -= g_kd * (meas - g_pid.prevMeas) / dt;
  }
  g_pid.prevMeas = meas;
  g_pid.primed = true;

  return u;
}

// ---------------------------------------------------------------------------
//  State and faults
// ---------------------------------------------------------------------------

// State, Fault, Owner, RefPhase and CalState are defined in Types.h.

static State g_state = ST_IDLE;
static Fault g_fault = F_NONE;

static float         g_target      = 0.0f;
static unsigned long g_moveStarted = 0;
static unsigned long g_jogUntil    = 0;
static int           g_jogDuty     = 0;
static unsigned long g_lastLoopMs  = 0;
static unsigned long g_lastTeleMs  = 0;
static CalState      g_cal;

// Progress window for the stall check: where the dish was, and since when.
static uint16_t      g_progRaw = 0;
static unsigned long g_progMs  = 0;

// Who last commanded the axis. Not a permission system -- every command is
// still accepted -- but the host needs to know it was pre-empted by someone
// at the panel, and "why did my move stop" should have an answer.
static Owner g_owner = OWN_LOCAL;

#if USE_REF_SWITCH
static RefPhase g_refPhase = RP_GOTO_START;
static bool     g_refAdopt = false;
static bool     g_refPrev  = false;          // last polled switch state
static int16_t  g_refSeenPos = -1;           // captured during this pass
static int16_t  g_refSeenNeg = -1;
#endif
static int16_t  g_refRawPos = REF_RAW_POS_DEFAULT;   // also saved in EEPROM
static int16_t  g_refRawNeg = REF_RAW_NEG_DEFAULT;

#if USE_STEP_TEST
static unsigned long g_stepUntil = 0;
static unsigned long g_stepNext  = 0;
static unsigned long g_stepT0    = 0;
#endif

const __FlashStringHelper *faultName(Fault f) {
  switch (f) {
    case F_NONE:    return F("none");
    case F_STALL:   return F("stall");
    case F_LINKAGE: return F("linkage");
    case F_ENCODER: return F("encoder");
    case F_LIMIT:   return F("limit");
    case F_TIMEOUT: return F("timeout");
    case F_CURRENT: return F("current");
    case F_RANGE:   return F("range");
    case F_REF:     return F("ref");
    case F_KILL:    return F("kill");
    case F_CAL:     return F("calibration");
  }
  return F("?");
}

const __FlashStringHelper *stateName(State s) {
  switch (s) {
    case ST_IDLE:     return F("IDLE");
    case ST_MOVING:   return F("MOVING");
    case ST_JOG:      return F("JOG");
    case ST_MANUAL:   return F("MANUAL");
    case ST_STEPTEST: return F("STEP");
    case ST_REFPASS:  return F("REFPASS");
    case ST_CALIB:    return F("CALIB");
    case ST_FAULT:    return F("FAULT");
  }
  return F("?");
}

// Anything that ends ST_CALIB from outside (stop, panel button, kill, fault)
// leaves the zero exactly as it was. Say so, so nobody wonders.
static void noteCalAbort() {
  if (g_state == ST_CALIB) Serial.println(F("calibration aborted -- zero unchanged"));
}

void raiseFault(Fault f) {
  noteCalAbort();
  motorOff();
  pidReset();
  g_fault = f;
  g_state = ST_FAULT;
  beep(3, BEEP_LONG_MS);
  Serial.print(F("FAULT "));
  Serial.println(faultName(f));
}

void stopMotion(const __FlashStringHelper *why) {
  noteCalAbort();
  motorOff();
  pidReset();
  if (g_state != ST_FAULT) g_state = ST_IDLE;
  if (why) { Serial.print(F("stop: ")); Serial.println(why); }
}

// ---------------------------------------------------------------------------
//  Printing -- integer only
// ---------------------------------------------------------------------------
//  No %f anywhere. AVR's default printf has no float support and pulling in
//  the version that does costs about 1.5 kB of flash for the privilege of
//  printing three decimal places.

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

// 's': the full block, for a human.
void printStatus() {
  Serial.println(F("\n----------------- SYSTEM STATUS -----------------"));
  Serial.print(F(" Mode/State : ")); Serial.print(stateName(g_state));
  if (g_state == ST_FAULT) {
    Serial.print(F(" [FAULT: ")); Serial.print(faultName(g_fault)); Serial.print(F("]"));
  }
  Serial.println();

  Serial.print(F(" Position   : ")); printDeg(positionDeg()); Serial.println(F(" deg"));
  Serial.print(F(" Target     : ")); printDeg(g_target);      Serial.println(F(" deg"));
  Serial.print(F(" Motor Duty : ")); Serial.print(g_duty);
  Serial.print(F(" / "));           Serial.print(SPEED_MAX);   Serial.println(F(" PWM"));

  Serial.print(F(" Encoder    : Raw ")); Serial.print(g_encRaw);
  Serial.print(F(" (Zero: ")); Serial.print(g_zeroCount);
  Serial.println(g_encOk ? F(") [OK]") : F(") [ERROR]"));

#if USE_REED
  Serial.print(F(" Reed Pulses: ")); Serial.println(reedPulses());
#endif
  Serial.print(F(" Kill Switch: ")); Serial.println(killAsserted() ? F("THROWN (ACTIVE)") : F("Clear"));
  Serial.print(F(" Controller : ")); Serial.println(g_owner == OWN_LOCAL ? F("Panel Buttons") : F("Serial Command"));
  Serial.println(F("-------------------------------------------------\n"));
}

// One line, printed while moving. See TELEMETRY_VERBOSE in Config.h.
void printTelemetry() {
  Serial.print(stateName(g_state));
  if (g_state == ST_CALIB) Serial.print(g_cal.leg == 0 ? F(" extend") : F(" retract"));
  Serial.print(F("  pos "));  printDeg(positionDeg());
  if (g_state == ST_MOVING) { Serial.print(F("  tgt ")); printDeg(g_target); }
  Serial.print(F("  duty ")); Serial.println(g_duty);
}

// ---------------------------------------------------------------------------
//  EEPROM
// ---------------------------------------------------------------------------

#if USE_EEPROM
// Saved is defined in Types.h.

void saveSettings() {
  Saved s;
  s.magic = EEPROM_MAGIC;
  s.zero = g_zeroCount;
  s.refPos = g_refRawPos;
  s.refNeg = g_refRawNeg;
  s.kp = g_kp; s.ki = g_ki; s.kd = g_kd; s.kff = g_kff;
  EEPROM.put(EEPROM_BASE_ADDR, s);
  Serial.println(F("saved"));
}

// A block with the right magic can still be damaged. A NaN gain or a zero
// outside one turn would drive the loop on garbage, so check before trusting.
static bool settingsSane(const Saved &s) {
  return s.zero >= 0 && s.zero < (int16_t)AS5600_COUNTS &&
         isfinite(s.kp)  && s.kp  >= 0.0f && s.kp  <= 1000.0f &&
         isfinite(s.ki)  && s.ki  >= 0.0f && s.ki  <= 1000.0f &&
         isfinite(s.kd)  && s.kd  >= 0.0f && s.kd  <= 1000.0f &&
         isfinite(s.kff) && s.kff >= 0.0f && s.kff <= 255.0f;
}

void loadSettings() {
  Saved s;
  EEPROM.get(EEPROM_BASE_ADDR, s);
  if (s.magic != EEPROM_MAGIC) { Serial.println(F("eeprom: blank, using defaults")); return; }
  if (!settingsSane(s))        { Serial.println(F("eeprom: damaged, using defaults")); return; }
  g_zeroCount = s.zero;
  g_refRawPos = s.refPos;
  g_refRawNeg = s.refNeg;
  g_kp = s.kp; g_ki = s.ki; g_kd = s.kd; g_kff = s.kff;
  Serial.println(F("eeprom: loaded"));
}
#else
void saveSettings() {}
void loadSettings() {}
#endif

// ---------------------------------------------------------------------------
//  Progress window and health checks
// ---------------------------------------------------------------------------

// Start of any powered motion: restart the clocks the health checks run on.
static void beginWindow() {
  g_moveStarted = millis();
  g_progRaw = g_encRaw;
  g_progMs  = g_moveStarted;
#if USE_REED
  g_winPulses = reedPulses();
  g_winStartDeg = positionDeg();
#endif
}

// Run on every tick while the motor is commanded on. Returns false if it
// ended the motion.
//
// stallFaults: a stall during a requested move ('g', '+', '-') is a fault. A
// stall under a panel button or a jog is just the dish at the end of travel
// with the button still held, which is normal, so it stops the motor quietly.
bool healthOk(bool stallFaults) {
  const unsigned long now = millis();

  if (!g_encOk) { raiseFault(F_ENCODER); return false; }

  if (now - g_moveStarted > MAX_RUN_MS) { raiseFault(F_TIMEOUT); return false; }

#if USE_CURRENT_LIMIT
  if (analogRead(PIN_CS) > CURRENT_TRIP_ADC) { raiseFault(F_CURRENT); return false; }
#endif

  if (g_duty == 0) {                    // coasting, or inside the deadband:
    g_progRaw = g_encRaw;               //   nothing is being asked of the dish,
    g_progMs  = now;                    //   so nothing to judge
    return true;
  }

#if USE_REED
  // Reed silent while the motor is on. Past the breakaway grace this is a
  // stall, a cam that has cut its own current, or a dead fuse.
  const unsigned long sinceStart = now - g_moveStarted;
  const unsigned long sincePulse = now - reedLastMs();
  if (sinceStart > START_GRACE_MS && sincePulse > STALL_TIMEOUT_MS) {
    raiseFault(F_STALL);
    return false;
  }

  // Reed pulsing but the boom not moving: the motor is turning something
  // that is no longer attached to the dish.
  const unsigned long pulses = reedPulses() - g_winPulses;
  if (pulses >= LINKAGE_PULSES) {
    if (fabs(positionDeg() - g_winStartDeg) < LINKAGE_MIN_DEG) {
      raiseFault(F_LINKAGE);
      return false;
    }
    g_winPulses = reedPulses();
    g_winStartDeg = positionDeg();
  }
#endif

  // Encoder stall: driven, and the dish has not moved STALL_MIN_COUNTS in
  // STALL_WINDOW_MS.
  if (abs(angleDiff(g_encRaw, g_progRaw)) >= (int16_t)STALL_MIN_COUNTS) {
    g_progRaw = g_encRaw;
    g_progMs  = now;
  } else if (now - g_progMs > STALL_WINDOW_MS) {
    if (stallFaults) {
      Serial.println(F("stall: driven, but the dish is not moving"));
      raiseFault(F_STALL);
    } else {
      stopMotion(F("not moving -- end of travel?"));
    }
    return false;
  }

  return true;
}

// ---------------------------------------------------------------------------
//  Motion entry points
// ---------------------------------------------------------------------------

void beginMove(float targetDeg) {
  g_owner = OWN_REMOTE;
  if (g_state == ST_FAULT) { Serial.println(F("faulted -- 'k' first")); return; }
  if (!g_encOk)            { Serial.println(F("no encoder")); return; }

  // Not a travel limit -- see TARGET_LIMIT_DEG in Config.h.
  if (fabs(targetDeg) > TARGET_LIMIT_DEG) {
    Serial.print(F("refused: target beyond +/-"));
    printDeg(TARGET_LIMIT_DEG);
    Serial.println(F(" deg"));
    return;
  }

  g_target = targetDeg;
  beginWindow();
  pidReset();
  g_state = ST_MOVING;

  Serial.print(F("move to ")); printDeg(g_target);
  Serial.print(F(" from "));   printDeg(positionDeg());
  Serial.println();
}

void beginJog(int8_t dir, uint16_t ms) {
  g_owner = OWN_REMOTE;
  if (g_state == ST_FAULT) { Serial.println(F("faulted -- 'k' first")); return; }
  ms = constrain(ms, 1, JOG_MS_MAX);
  g_jogDuty = dir > 0 ? (int)SPEED_SLOW : -(int)SPEED_SLOW;
  g_jogUntil = millis() + ms;
  beginWindow();
  g_state = ST_JOG;
}

// ---------------------------------------------------------------------------
//  Panel buttons -- manual mode
// ---------------------------------------------------------------------------
//  Human-timescale debounce, which is a completely different problem from the
//  reed's: no interrupt, no series resistor, no capacitor. Just refuse to
//  believe a change until it has held for BTN_DEBOUNCE_MS.

#if USE_BUTTONS
// Button is defined in Types.h.

static Button g_btnExtend  = {PIN_BTN_EXTEND,  false, false, 0};
static Button g_btnRetract = {PIN_BTN_RETRACT, false, false, 0};
#if USE_STOP_BUTTON
static Button g_btnStop    = {PIN_BTN_STOP,    false, false, 0};
#endif

// Returns true on the rising edge of a debounced press.
bool buttonPoll(Button &b) {
  const bool raw = (digitalRead(b.pin) == LOW);   // to GND, INPUT_PULLUP
  const unsigned long now = millis();
  if (raw != b.last) { b.last = raw; b.changed = now; return false; }
  if (now - b.changed < BTN_DEBOUNCE_MS) return false;
  if (raw == b.stable) return false;
  b.stable = raw;
  return raw;                                     // edge, and it is a press
}

static int8_t        g_manualDir   = 0;
static unsigned long g_manualStart = 0;

// Called every tick, ahead of the state machine, so a press pre-empts
// whatever else was running. Returns true if the buttons now own the axis.
bool handleButtons() {
#if USE_STOP_BUTTON
  const bool stopEdge    = buttonPoll(g_btnStop);
#else
  const bool stopEdge    = false;     // no stop button; the kill switch stops
#endif
  const bool extendEdge  = buttonPoll(g_btnExtend);
  const bool retractEdge = buttonPoll(g_btnRetract);

  if (stopEdge) {
    g_owner = OWN_LOCAL;
    g_manualDir = 0;
    stopMotion(F("panel STOP"));
    return true;
  }

  // Both at once is ambiguous, so it means stop rather than guessing.
  if (g_btnExtend.stable && g_btnRetract.stable) {
    if (g_state == ST_MANUAL) { g_manualDir = 0; stopMotion(F("both buttons")); }
    return g_state == ST_MANUAL;
  }

  if (extendEdge || retractEdge) {
    if (g_state == ST_FAULT) {
      Serial.println(F("faulted -- 'k' first"));
      return false;
    }
    // Whoever is at the panel is standing next to the dish; the host is not.
    if (g_state != ST_IDLE && g_state != ST_MANUAL) {
      noteCalAbort();
      Serial.println(F("panel took over"));
    }
    g_owner = OWN_LOCAL;
    g_manualDir = extendEdge ? +1 : -1;
    g_manualStart = millis();
    beginWindow();
    pidReset();
    g_state = ST_MANUAL;
    return true;
  }

  return g_state == ST_MANUAL;
}

// Hold-to-run. Released is stopped, that instant -- which is the entire
// reason these are buttons and not a serial command. Slow for the first
// BTN_SLOW_MS of a press, then full duty (USE_BUTTON_RAMP).
void manualTick() {
  const bool held = (g_manualDir > 0) ? g_btnExtend.stable : g_btnRetract.stable;
  if (!held || g_manualDir == 0) {
    g_manualDir = 0;
    motorOff();
    g_state = ST_IDLE;
    Serial.print(F("manual stop at "));
    printDeg(positionDeg());
    Serial.println(F(" deg"));
    return;
  }

#if USE_BUTTON_RAMP
  const bool slow = (millis() - g_manualStart) < BTN_SLOW_MS;
  const int duty = slow ? (int)SPEED_BUTTON_SLOW : (int)SPEED_BUTTON_FAST;
#else
  const int duty = (int)SPEED_BUTTON_FAST;
#endif
  driveSigned(g_manualDir > 0 ? duty : -duty);
  if (g_limitBlocked) raiseFault(F_LIMIT);
}
#endif  // USE_BUTTONS

// ---------------------------------------------------------------------------
//  Reference switch (USE_REF_SWITCH, disabled)
// ---------------------------------------------------------------------------
//  Not a hard stop, and never confused with one. This switch sits inside the
//  travel, carries no safety duty, and exists only so the stored ZERO can be
//  checked against a physical fact. Because it is mid-travel the boom crosses
//  it on ordinary moves, so most checking is passive. 'h' forces a deliberate
//  slow pass when the authoritative number is wanted.
//
//  Retained for possible future use. Untested since the encoder stall check
//  was added: REF_CROSS_SPEED is slow, so check it clears STALL_WINDOW_MS.

#if USE_REF_SWITCH

// Called exactly once per tick. Returns true on an inactive->active edge.
bool refEdge() {
  const bool now = refActive();
  const bool entering = (now && !g_refPrev);
  g_refPrev = now;
  return entering;
}

// A crossing during ordinary motion. Free integrity check: if this drifts,
// the magnet has moved on its hub or the switch has.
void refPassive(bool entering) {
  if (!entering || g_duty == 0) return;
  const bool positive = (g_duty > 0);
  const int16_t stored = positive ? g_refRawPos : g_refRawNeg;
  if (stored < 0) return;                       // nothing adopted yet

  const float drift = countsToDeg(angleDiff(g_encRaw, (uint16_t)stored));
  if (fabs(drift) > REF_DRIFT_WARN_DEG) {
    Serial.print(F("REF drift "));
    printDeg(drift);
    Serial.print(F(" deg on a "));
    Serial.print(positive ? '+' : '-');
    Serial.println(F(" crossing -- check the magnet and the switch"));
  }
}

void beginRefPass(bool adopt) {
  g_owner = OWN_REMOTE;
  if (g_state == ST_FAULT) { Serial.println(F("faulted -- 'k' first")); return; }
  if (!g_encOk)            { Serial.println(F("no encoder")); return; }

  g_refAdopt = adopt;
  g_refPhase = RP_GOTO_START;
  g_refSeenPos = -1;
  g_refSeenNeg = -1;
  beginWindow();
  pidReset();
  g_state = ST_REFPASS;

  Serial.println(adopt ? F("ref pass -- will ADOPT both crossings")
                       : F("ref pass -- check only, nothing will change"));
}

void finishRefPass() {
  motorOff();
  g_state = ST_IDLE;
  beep(2, BEEP_SHORT_MS);

  Serial.print(F("crossings: +dir raw ")); Serial.print(g_refSeenPos);
  Serial.print(F("  -dir raw "));          Serial.println(g_refSeenNeg);

  if (g_refSeenPos >= 0 && g_refSeenNeg >= 0) {
    // Lobe width plus switch hysteresis. A constant of the mechanism, so a
    // change in it means the lever is bending or the cam has worked loose.
    const float spread = countsToDeg(
        angleDiff((uint16_t)g_refSeenNeg, (uint16_t)g_refSeenPos));
    Serial.print(F("lobe+hysteresis: ")); printDeg(spread);
    Serial.println(F(" deg"));
  }

  bool reported = false;
  if (g_refRawPos >= 0 && g_refSeenPos >= 0) {
    Serial.print(F("drift +dir: "));
    printDeg(countsToDeg(angleDiff((uint16_t)g_refSeenPos, (uint16_t)g_refRawPos)));
    Serial.println(F(" deg"));
    reported = true;
  }
  if (g_refRawNeg >= 0 && g_refSeenNeg >= 0) {
    Serial.print(F("drift -dir: "));
    printDeg(countsToDeg(angleDiff((uint16_t)g_refSeenNeg, (uint16_t)g_refRawNeg)));
    Serial.println(F(" deg"));
    reported = true;
  }
  if (!reported) Serial.println(F("no stored reference yet -- 'H' to adopt"));

  if (g_refAdopt) {
    if (g_refSeenPos < 0 || g_refSeenNeg < 0) {
      Serial.println(F("did not see both crossings -- nothing adopted"));
      return;
    }
    g_refRawPos = g_refSeenPos;
    g_refRawNeg = g_refSeenNeg;
    // Zero is derived from the midpoint of the two crossings, which cancels
    // the switch's hysteresis instead of inheriting whichever direction
    // happened to be captured.
    const int16_t half = angleDiff((uint16_t)g_refSeenNeg, (uint16_t)g_refSeenPos) / 2;
    const uint16_t mid = wrapCount((long)g_refSeenPos + half);
    g_zeroCount = (int16_t)wrapCount((long)mid - (long)degToCounts(REF_ANGLE_DEG));
    Serial.print(F("adopted. zero=")); Serial.print(g_zeroCount);
    Serial.print(F("  position now ")); printDeg(positionDeg());
    Serial.println(F(" deg -- 'w' to save"));
  }
}

void refPassTick(bool entering) {
  if (millis() - g_moveStarted > REF_TIMEOUT_MS) {
    Serial.println(F("ref pass never found the switch"));
    raiseFault(F_REF);
    return;
  }

  const float pos = positionDeg();
  const float startAt = REF_ANGLE_DEG - REF_MARGIN_DEG;
  const float endAt   = REF_ANGLE_DEG + REF_MARGIN_DEG;

  switch (g_refPhase) {

    case RP_GOTO_START:
      // Get clear of the switch on the negative side, so the first crossing
      // is always entered from the same direction.
      if (pos <= startAt && !refActive()) {
        motorOff();
        g_refPhase = RP_CROSS_POS;
        g_moveStarted = millis();
      } else {
        driveSigned(pos > startAt ? -(int)SPEED_SLOW : (int)REF_CROSS_SPEED);
      }
      break;

    case RP_CROSS_POS:
      if (entering) {
        g_refSeenPos = (int16_t)g_encRaw;
        Serial.print(F("  +dir crossing at raw ")); Serial.println(g_refSeenPos);
        g_refPhase = RP_GOTO_END;
        g_moveStarted = millis();
      } else {
        driveSigned((int)REF_CROSS_SPEED);
      }
      break;

    case RP_GOTO_END:
      if (pos >= endAt && !refActive()) {
        motorOff();
        g_refPhase = RP_CROSS_NEG;
        g_moveStarted = millis();
      } else {
        driveSigned((int)REF_CROSS_SPEED);
      }
      break;

    case RP_CROSS_NEG:
      if (entering) {
        g_refSeenNeg = (int16_t)g_encRaw;
        Serial.print(F("  -dir crossing at raw ")); Serial.println(g_refSeenNeg);
        finishRefPass();
      } else {
        driveSigned(-(int)REF_CROSS_SPEED);
      }
      break;
  }
}

#else   // ------------------------------------------------------- not fitted

bool refEdge() { return false; }
void refPassive(bool entering) { (void)entering; }

#endif  // USE_REF_SWITCH

// ---------------------------------------------------------------------------
//  Step test (USE_STEP_TEST, disabled)
// ---------------------------------------------------------------------------
//  Streams ms,target,position,duty as CSV while the PID chases a step, for
//  tuning offline. Retained for possible future use.

#if USE_STEP_TEST
void beginStepTest(float stepDeg) {
  g_owner = OWN_REMOTE;
  if (g_state == ST_FAULT) { Serial.println(F("faulted -- 'k' first")); return; }
  if (!g_encOk)            { Serial.println(F("no encoder")); return; }

  g_target = positionDeg() + stepDeg;
  g_stepT0 = millis();
  g_stepNext = g_stepT0;
  g_stepUntil = g_stepT0 + STEP_TEST_MS;
  beginWindow();
  pidReset();
  g_state = ST_STEPTEST;
  Serial.println(F("ms,target,position,duty"));
}
#endif

// ---------------------------------------------------------------------------
//  Auto-calibration  ('c y')
// ---------------------------------------------------------------------------
//  Finds the zero from the mechanism, for when the stored one has been lost.
//  It runs inside loop() like every other state, one step per tick, on the
//  same encoder reading as everything else -- so a stop command, the panel
//  buttons and the kill switch all work throughout, which a blocking sweep
//  cannot offer.
//
//    leg 0  extend until the encoder stops moving -> that is the extend stop
//    leg 1  retract the same way; the encoder travel accumulated on this leg
//           is the full span between the stops
//    zero = the extend stop, minus half the span
//
//  The travel is accumulated in raw counts, one tick at a time, so it needs no
//  idea of where zero used to be and cannot wrap wrongly, which a midpoint
//  taken in degrees from a lost zero can. Nothing is adopted unless the span
//  and its sign look right. Afterwards the dish centres itself at the new 0.

static void calStartPhase(CalPhase p, unsigned long now) {
  g_cal.phase = p;
  g_cal.phaseStart = now;
  g_cal.markRaw = g_encRaw;
  g_cal.markMs = now;
}

static void calStartLeg(uint8_t leg, unsigned long now) {
  g_cal.leg = leg;
  g_cal.legStart = now;
  g_cal.travel = 0;
  g_cal.lastRaw = g_encRaw;
  calStartPhase(CAL_SWEEP, now);
}

void printCalibrateHelp() {
  Serial.println(F("AUTO-CALIBRATION finds zero from the end stops:"));
  Serial.println(F("  - drives the dish to BOTH ends of its travel at full duty"));
  Serial.println(F("  - sets 0 deg at the midpoint and saves it"));
  Serial.println(F("  - then centres the dish; this can take 10+ minutes"));
  Serial.println(F("Keep clear. <enter>, a panel button, or kill aborts it and"));
  Serial.println(F("leaves the old zero as it was.  Send 'c y' to start."));
}

void beginCalibrate() {
  g_owner = OWN_REMOTE;
  if (g_state == ST_FAULT) { Serial.println(F("faulted -- 'k' first")); return; }
  if (g_state != ST_IDLE)  { Serial.println(F("busy -- <enter> to stop first")); return; }
  if (!g_encOk)            { Serial.println(F("no encoder")); return; }

  pidReset();
  g_state = ST_CALIB;
  calStartLeg(0, millis());
  Serial.println(F("calibrating: extending to the end stop..."));
}

static void calFail(const __FlashStringHelper *why) {
  Serial.print(F("calibration FAILED: ")); Serial.println(why);
  Serial.println(F("zero unchanged"));
  raiseFault(F_CAL);
}

static void calFinish() {
  const int32_t travel = g_cal.travel;     // signed raw counts, extend stop -> retract stop
  const float span = (float)labs(travel) * DEG_PER_COUNT;

  Serial.print(F("cal: measured travel ")); printDeg(span);
  Serial.print(F(" deg (expected ")); printDeg(2.0f * TRAVEL_HALF_DEG);
  Serial.println(F(")"));

  // Retracting moves the dish toward -deg, so the raw count must have moved up
  // if the encoder is inverted and down if it is not. The wrong sign means
  // ENCODER_INVERT is backwards, and the PID would run away on it.
  const bool rawShouldRise = (ENCODER_INVERT != 0);
  if ((travel > 0) != rawShouldRise) {
    calFail(F("travel has the wrong sign -- is ENCODER_INVERT backwards?"));
    return;
  }
  if (fabs(span - 2.0f * TRAVEL_HALF_DEG) > CAL_SPAN_TOL_DEG) {
    calFail(F("span is not the expected travel -- a stall mid-travel?"));
    return;
  }

  const uint16_t mid = wrapCount((long)g_cal.rawA + travel / 2);
  const float shift = countsToDeg(angleDiff(mid, (uint16_t)g_zeroCount));

  Serial.print(F("cal: zero was raw ")); Serial.print(g_zeroCount);
  Serial.print(F(", now raw "));        Serial.print(mid);
  Serial.print(F(" (moved "));          printDeg(shift);
  Serial.println(F(" deg)"));

  g_zeroCount = (int16_t)mid;
#if CAL_AUTOSAVE
  saveSettings();
#else
  Serial.println(F("cal: not saved -- 'w' to save"));
#endif
  beep(2, BEEP_SHORT_MS);

  Serial.println(F("cal: centring at 0.000 deg"));
  beginMove(0.0f);                          // leaves ST_CALIB for ST_MOVING
}

// A leg whose end stop has been proven.
static void calLegDone(unsigned long now) {
  if (g_cal.leg == 0) {
    g_cal.rawA = g_encRaw;
    Serial.print(F("cal: extend stop at raw ")); Serial.println(g_encRaw);
    Serial.println(F("cal: retracting to the other end stop..."));
    calStartLeg(1, now);
  } else {
    Serial.print(F("cal: retract stop at raw ")); Serial.println(g_encRaw);
    calFinish();
  }
}

void calibTick(unsigned long now) {
  if (!g_encOk) { raiseFault(F_ENCODER); return; }

  // Accumulate unwrapped travel. The sum of per-tick differences telescopes,
  // so encoder noise cancels and only the real displacement is left.
  g_cal.travel += angleDiff(g_encRaw, g_cal.lastRaw);
  g_cal.lastRaw = g_encRaw;

  if (now - g_cal.legStart > CAL_SWEEP_TIMEOUT_MS) {
    Serial.println(F("cal: this leg timed out"));
    raiseFault(F_TIMEOUT);
    return;
  }

  const int dir = (g_cal.leg == 0) ? +1 : -1;
  const bool moved = abs(angleDiff(g_encRaw, g_cal.markRaw)) >= (int16_t)CAL_MOVE_COUNTS;

  switch (g_cal.phase) {

    case CAL_SWEEP:                  // drive until the encoder stops moving
      driveSigned(dir * (int)SPEED_CALIB);
      if (moved) {
        g_cal.markRaw = g_encRaw;
        g_cal.markMs = now;
      } else if (now - g_cal.markMs >= CAL_STALL_MS) {
        motorOff();
        calStartPhase(CAL_SETTLE, now);
      }
      break;

    case CAL_SETTLE:                 // motor off, let it come to rest
      motorOff();
      if (now - g_cal.phaseStart >= CAL_SETTLE_MS) calStartPhase(CAL_PUSH, now);
      break;

    case CAL_PUSH:                   // one more try: is it really the stop?
      driveSigned(dir * (int)SPEED_CALIB);
      if (moved) {
        // It moved. That was a pause, not the end of travel: carry on.
        Serial.println(F("cal: it moved again -- that was not the stop"));
        calStartPhase(CAL_SWEEP, now);
      } else if (now - g_cal.phaseStart >= CAL_PUSH_MS) {
        motorOff();
        calStartPhase(CAL_REST, now);
      }
      break;

    case CAL_REST:                   // coast before reversing or finishing
      motorOff();
      if (now - g_cal.phaseStart >= CAL_SETTLE_MS) calLegDone(now);
      break;
  }
}

// ---------------------------------------------------------------------------
//  Console
// ---------------------------------------------------------------------------

static char g_line[40];
static uint8_t g_len = 0;

void printHelp() {
  Serial.println(F("\n=================== COMMAND MENU ==================="));
  Serial.println(F(" [ Movement Control ]"));
  Serial.println(F("   g <deg>   : Move to absolute angle (e.g. 'g 5.5')"));
  Serial.println(F("   +<deg>    : Extend relative angle  (e.g. '+2.0')"));
  Serial.println(F("   -<deg>    : Retract relative angle (e.g. '-2.0')"));
  Serial.println(F("   e <ms>    : Open-loop extend jog   (e.g. 'e 500')"));
  Serial.println(F("   r <ms>    : Open-loop retract jog  (e.g. 'r 500')"));
  Serial.println(F("   <enter>/x : Instant STOP / Coast motor"));
  Serial.println();
  Serial.println(F(" [ Status & Diagnostics ]"));
  Serial.println(F("   s         : Print system status"));
  Serial.println(F("   m         : Monitor AS5600 encoder & magnet health"));
  Serial.println(F("   L         : Toggle live angle stream"));
#if USE_REED
  Serial.println(F("   n         : Run 5-second Reed sensor noise test"));
#endif
  Serial.println();
  Serial.println(F(" [ Setup & Tuning ]"));
  Serial.println(F("   z         : Set current angle as ZERO reference"));
  Serial.println(F("   c         : Explain auto-calibration; 'c y' runs it"));
  Serial.println(F("   w         : Save zero angle & PID settings to EEPROM"));
  Serial.println(F("   k         : Clear active fault state"));
  Serial.println(F("   p/i/d/f   : Set gains live (e.g. 'p 12.0', 'f 60.0')"));
#if USE_REF_SWITCH
  Serial.println(F("   h / H     : Reference pass: check / adopt"));
#endif
#if USE_STEP_TEST
  Serial.println(F("   t <deg>   : Step test, streamed as CSV"));
#endif
  Serial.println(F("====================================================\n"));
}

void monitorEncoder() {
  uint8_t st = 0, agc = 0;
  bool okS = as5600Status(&st);
  bool okA = as5600Agc(&agc);
  Serial.print(F("raw="));  Serial.print(g_encRaw);
  Serial.print(F(" pos=")); printDeg(positionDeg());
  if (okS) {
    Serial.print(F(" magnet="));
    if      (st & 0x10) Serial.print(F("WEAK"));
    else if (st & 0x08) Serial.print(F("STRONG"));
    else if (st & 0x20) Serial.print(F("ok"));
    else                Serial.print(F("NONE"));
  }
  if (okA) { Serial.print(F(" agc=")); Serial.print(agc); }
  Serial.println();
}

#if USE_REED
void reedNoiseFloor() {
  Serial.println(F("reed noise floor, 5 s, bridge powered, motor OFF"));
  motorOff();
  const unsigned long start = reedPulses();
  const unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
#if USE_WATCHDOG
    wdt_reset();
#endif
  }
  Serial.print(F("edges="));
  Serial.println(reedPulses() - start);
  Serial.println(F("anything but 0 is pickup, not travel"));
}
#endif

// A number and nothing else. atof() reads "abc" as 0, which for 'g' would
// send the dish to zero on a typo.
static bool parseNumber(const char *s, float *out) {
  char *end;
  const double v = strtod(s, &end);
  if (end == s || !isfinite(v)) return false;
  while (*end == ' ') end++;
  if (*end != '\0') return false;
  *out = (float)v;
  return true;
}

// While calibrating, only commands that cannot move anything are accepted.
// Anything else would fight the sweep for the motor.
static bool allowedDuringCal(char c) {
  return c == 's' || c == '?' || c == 'L' || c == 'l' || c == 'm' ||
         c == 'x' || c == 'X';
}

void handleLine(char *s) {
  while (*s == ' ') s++;

  if (*s == '\0') { stopMotion(F("Emergency Stop Requested")); return; }

  const char c = *s;
  char *arg = s + 1;
  while (*arg == ' ') arg++;
  const bool hasArg = (*arg != '\0');
  float fv = 0.0f;
  const bool numOk = hasArg && parseNumber(arg, &fv);

  if (g_state == ST_CALIB && !allowedDuringCal(c)) {
    Serial.println(F("calibrating -- <enter> or a panel button aborts"));
    return;
  }

  switch (c) {
    case 'x':
    case 'X':
      stopMotion(F("Stop Requested"));
      break;

    case 'C':
    case 'c':
      if (hasArg && (*arg == 'y' || *arg == 'Y')) beginCalibrate();
      else printCalibrateHelp();
      break;

    case 'L':
    case 'l':
      g_liveAngleStream = !g_liveAngleStream;
      Serial.print(F("Live Angle Stream: "));
      Serial.println(g_liveAngleStream ? F("ON (send 'L' to stop)") : F("OFF"));
      break;

    case 's': printStatus(); break;
    case '?': printHelp();   break;

    case 'g':
      if (numOk) beginMove(fv);
      else Serial.println(F("Usage: g <degrees>"));
      break;
    case '+':
    case '-': {
      float d = 0.5f;                       // a bare '+' or '-' nudges half a degree
      if (hasArg && !numOk) { Serial.println(F("Usage: +<degrees> or -<degrees>")); break; }
      if (numOk) d = fv;
      beginMove(positionDeg() + (c == '+' ? d : -d));
      break;
    }

    case 'e':
    case 'r': {
      if (hasArg && !numOk) { Serial.println(F("Usage: e <ms>  or  r <ms>")); break; }
      const float want = numOk ? fv : (float)JOG_MS_DEFAULT;
      const uint16_t ms = (uint16_t)constrain(want, 1.0f, (float)JOG_MS_MAX);
      Serial.print(c == 'e' ? F("Jog Extending for ") : F("Jog Retracting for "));
      Serial.print(ms); Serial.println(F(" ms"));
      beginJog(c == 'e' ? +1 : -1, ms);
      break;
    }

    case 'm': monitorEncoder(); break;
#if USE_REED
    case 'n': reedNoiseFloor(); break;
#endif
#if USE_REF_SWITCH
    case 'h': beginRefPass(false); break;
    case 'H': beginRefPass(true);  break;
#endif
#if USE_STEP_TEST
    case 't':
      if (numOk) beginStepTest(fv);
      else Serial.println(F("Usage: t <degrees>"));
      break;
#endif

    case 'z':
      g_zeroCount = (int16_t)g_encRaw;
      Serial.print(F("--> Zero angle calibrated at raw count: "));
      Serial.println(g_zeroCount);
      break;

    case 'p': if (numOk) { g_kp  = fv; Serial.print(F("Kp set to: ")); Serial.println(g_kp); } break;
    case 'i': if (numOk) { g_ki  = fv; pidReset(); Serial.print(F("Ki set to: ")); Serial.println(g_ki); } break;
    case 'd': if (numOk) { g_kd  = fv; Serial.print(F("Kd set to: ")); Serial.println(g_kd); } break;
    case 'f': if (numOk) { g_kff = fv; Serial.print(F("Kff set to: ")); Serial.println(g_kff); } break;

    case 'w': saveSettings(); break;

    case 'k':
      if (killAsserted()) {
        Serial.println(F("Cannot clear fault: Physical KILL switch on Pin 7 is thrown."));
        break;
      }
      motorOff();
      g_fault = F_NONE;
      g_state = ST_IDLE;
      g_encFails = 0;
      pidReset();
      Serial.println(F("--> Fault state cleared. System ready."));
      break;

    default:
      Serial.println(F("Unknown command. Type '?' for available options."));
  }
}

void pollSerial() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      g_line[g_len] = '\0';
      handleLine(g_line);
      g_len = 0;
      return;
    }
    if (g_len < sizeof(g_line) - 1) g_line[g_len++] = ch;
  }
}

// ---------------------------------------------------------------------------
//  setup / loop
// ---------------------------------------------------------------------------

void setup() {
  // What reset us, read before anything can clear it. A watchdog reset has to
  // be reported: it means the sketch hung and recovered, which nobody would
  // otherwise know.
  const uint8_t resetCause = MCUSR;
  MCUSR = 0;
#if USE_WATCHDOG
  wdt_disable();      // a watchdog left running would reset us again mid-setup
#endif

  // Claim the bridge low BEFORE anything else. An unconfigured output is a
  // floating input, and a driver gets to decide for itself what that means.
#if MOTOR_DRIVER == DRV_L298N
  #if L298N_CHANNEL == 2
  // The dead channel first: see PIN_DEAD_* in Config.h.
  pinMode(PIN_DEAD_ENA, OUTPUT); digitalWrite(PIN_DEAD_ENA, LOW);
  pinMode(PIN_DEAD_IN1, OUTPUT); digitalWrite(PIN_DEAD_IN1, LOW);
  pinMode(PIN_DEAD_IN2, OUTPUT); digitalWrite(PIN_DEAD_IN2, LOW);
  #endif
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
#elif MOTOR_DRIVER == DRV_HW039
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
  digitalWrite(PIN_RPWM, LOW);
  digitalWrite(PIN_LPWM, LOW);
  digitalWrite(PIN_EN,   LOW);
#else
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_SLP, OUTPUT);
  digitalWrite(PIN_PWM, LOW);
  digitalWrite(PIN_DIR, LOW);
  digitalWrite(PIN_SLP, LOW);
#endif

#if USE_REED
  pinMode(PIN_REED,    INPUT_PULLUP);
#endif
#if USE_LIMIT_SWITCHES
  pinMode(PIN_LIM_POS, INPUT_PULLUP);
  pinMode(PIN_LIM_NEG, INPUT_PULLUP);
#endif
#if USE_REF_SWITCH
  pinMode(PIN_REF,     INPUT_PULLUP);
#endif
#if USE_BUTTONS
  pinMode(PIN_BTN_EXTEND,  INPUT_PULLUP);
  pinMode(PIN_BTN_RETRACT, INPUT_PULLUP);
#if USE_STOP_BUTTON
  pinMode(PIN_BTN_STOP,    INPUT_PULLUP);
#endif
#endif
#if USE_KILL
  pinMode(PIN_KILL,        INPUT_PULLUP);
#endif
#if USE_BUZZER
  pinMode(PIN_BUZZER, OUTPUT);
#endif

  // Let the pull-ups charge the 100 nF filter caps before the first reading:
  // an uncharged cap reads LOW, and an NC limit that reads LOW looks clear.
  delay(10);
  initInputs();

#if USE_REED
  attachInterrupt(digitalPinToInterrupt(PIN_REED), onReedEdge, FALLING);
#endif

  Serial.begin(115200);
  Serial.println(F("MOON controller_v1 -- Rev B"));
  if (resetCause & _BV(WDRF))      Serial.println(F("reset: WATCHDOG -- the sketch hung and recovered"));
  else if (resetCause & _BV(BORF)) Serial.println(F("reset: brown-out -- check the 5 V supply"));

  Wire.begin();
  // 100 kHz: the BSS138 shifter's 10k pull-ups make edges too slow for 400.
  // A 2-byte angle read still takes well under 1 ms of a 20 ms tick.
  Wire.setClock(100000);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
#endif

  loadSettings();

  uint8_t st;
  if (!as5600Status(&st)) {
    Serial.println(F("AS5600 did not answer at 0x36"));
  } else if (!(st & 0x20)) {
    Serial.println(F("AS5600 sees no magnet"));
  } else if (st & 0x10) {
    Serial.println(F("AS5600 magnet WEAK -- too far"));
  } else if (st & 0x08) {
    Serial.println(F("AS5600 magnet STRONG -- too close"));
  }

  encoderBoot();
#if USE_REF_SWITCH
  g_refPrev = refActive();   // so boot never looks like a crossing
#endif
  if (!g_encOk) {
    raiseFault(F_ENCODER);
  } else {
    Serial.print(F("boot position "));
    printDeg(positionDeg());
    Serial.println(F(" deg -- no homing needed"));
    beep(1, BEEP_SHORT_MS);
  }
  if (resetCause & _BV(WDRF)) beep(3, BEEP_LONG_MS);   // replaces the boot chirp

  // Booting into a thrown kill switch must not look like a healthy start.
  // Checked last so it is the fault that stands.
  if (killAsserted()) raiseFault(F_KILL);

  Serial.println(F("? for help"));
  g_lastLoopMs = millis();

#if USE_WATCHDOG
  wdt_enable(WDT_TIMEOUT_CODE);   // last, so a slow boot cannot trip it
#endif
}

void loop() {
#if USE_WATCHDOG
  wdt_reset();
#endif
  pollInputs();   // every pass, not every tick -- see filterInput()
  pollSerial();
  buzzerTick();

  // Every pass, not every LOOP_MS tick: the kill switch should not wait for
  // the control loop. Latched once, so a held switch does not refire.
  if (killAsserted() && !(g_state == ST_FAULT && g_fault == F_KILL)) {
    raiseFault(F_KILL);
  }

  const unsigned long now = millis();
  if (now - g_lastLoopMs < LOOP_MS) return;
  const float dt = (float)(now - g_lastLoopMs) / 1000.0f;
  g_lastLoopMs = now;

  encoderPoll();

  // Exactly one edge poll per tick -- calling refEdge() twice would eat the
  // transition and the crossing would silently never be seen.
  const bool refEntering = refEdge();

#if USE_BUTTONS
  // Ahead of the state machine on purpose: a press pre-empts a host move.
  handleButtons();
#endif

  switch (g_state) {

    case ST_MOVING:
    case ST_STEPTEST: {
      if (!healthOk(true)) break;

      const float pos = positionDeg();
      const float u = pidStep(g_target, pos, dt);

      // Ceiling drops on the approach so the last of the move is slow enough
      // that coast does not throw the landing past the deadband.
      const int ceiling = (fabs(g_target - pos) < SLOW_ZONE_DEG)
                            ? (int)SPEED_SLOW : (int)SPEED_MAX;

      int out = (int)u;
      if (out >  ceiling) out =  ceiling;
      if (out < -ceiling) out = -ceiling;
      g_pid.holdIntegral = ((int)u != out);      // saturated -> stop winding

      // Anything under breakaway heats the motor and moves nothing.
      if (out != 0 && abs(out) < (int)SPEED_FLOOR) {
        out = (out > 0) ? (int)SPEED_FLOOR : -(int)SPEED_FLOOR;
      }

      driveSigned(out);
      refPassive(refEntering);

      // Driving INTO a limit switch under a normal move is a fault. Never
      // true while USE_LIMIT_SWITCHES is 0.
      if (g_limitBlocked) { raiseFault(F_LIMIT); break; }

#if USE_STEP_TEST
      if (g_state == ST_STEPTEST) {
        if (now >= g_stepNext) {
          g_stepNext = now + STEP_SAMPLE_MS;
          Serial.print(now - g_stepT0); Serial.print(',');
          printDeg(g_target);           Serial.print(',');
          printDeg(pos);                Serial.print(',');
          Serial.println(g_duty);
        }
        if (now >= g_stepUntil) { stopMotion(F("step test done")); }
        break;
      }
#endif

      if (out == 0 && fabs(g_target - pos) <= DEADBAND_DEG) {
        motorOff();
        g_state = ST_IDLE;
        Serial.print(F("landed at ")); printDeg(pos);
        Serial.print(F(" err "));      printDeg(g_target - pos);
        Serial.println();
      }
      break;
    }

#if USE_REF_SWITCH
    case ST_REFPASS:
      if (!healthOk(false)) break;
      refPassTick(refEntering);
      break;
#endif

#if USE_BUTTONS
    case ST_MANUAL:
      if (!healthOk(false)) break;
      manualTick();
      refPassive(refEntering);
      break;
#endif

    case ST_JOG:
      if (!healthOk(false)) break;
      if (now >= g_jogUntil) { stopMotion(F("jog done")); break; }
      driveSigned(g_jogDuty);
      refPassive(refEntering);
      if (g_limitBlocked) { raiseFault(F_LIMIT); }
      break;

    case ST_CALIB:
      calibTick(now);
      break;

    case ST_IDLE:
    case ST_FAULT:
    default:
      motorOff();
      break;
  }

  // Telemetry.
  const uint16_t period = (g_state == ST_CALIB) ? CAL_TELEMETRY_MS : TELEMETRY_MS;
  if (g_state != ST_STEPTEST && now - g_lastTeleMs >= period) {
    g_lastTeleMs = now;
    if (g_liveAngleStream) {
      Serial.print(F("Angle: "));
      printDeg(positionDeg());
      Serial.print(F(" deg | Raw: "));
      Serial.println(g_encRaw);
    } else if (g_state == ST_MOVING || g_state == ST_JOG ||
               g_state == ST_REFPASS || g_state == ST_MANUAL || g_state == ST_CALIB) {
#if TELEMETRY_VERBOSE
      printStatus();
#else
      printTelemetry();
#endif
    }
  }
}
