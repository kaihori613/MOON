// ===========================================================================
//  MOON actuator_v2 -- Rev B
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
//  Nothing here has been on hardware.

#include "Config.h"
#include <Wire.h>
#if USE_EEPROM
  #include <EEPROM.h>
#endif

// ---------------------------------------------------------------------------
//  Reed -- witness only
// ---------------------------------------------------------------------------
//  Deliberately not integrated into anything. It counts, and the count is
//  only ever compared against itself over a window to answer "is the motor
//  turning". Position comes from the encoder.

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

// ---------------------------------------------------------------------------
//  AS5600
// ---------------------------------------------------------------------------

static uint8_t  g_encFails   = 0;
static uint16_t g_encRaw     = 0;
static bool     g_encOk      = false;
static int16_t  g_zeroCount  = ENCODER_ZERO_DEFAULT;

// Returns false and leaves *out untouched on any bus error. Callers must
// treat that as fatal mid-move: a stale angle in the loop is worse than a
// stopped motor.
bool as5600Read(uint8_t reg, uint8_t n, uint8_t *buf) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)AS5600_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool as5600Angle(uint16_t *out) {
  uint8_t b[2];
  if (!as5600Read(0x0C, 2, b)) return false;        // RAW ANGLE hi/lo
  *out = (((uint16_t)b[0] << 8) | b[1]) & 0x0FFF;
  return true;
}

// STATUS bit 5 = MD (magnet detected), 4 = ML (too weak), 3 = MH (too strong)
bool as5600Status(uint8_t *out) {
  return as5600Read(0x0B, 1, out);
}

bool as5600Agc(uint8_t *out) {
  return as5600Read(0x1A, 1, out);
}

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

// Reads the encoder into the cached state. Returns false once the failure
// run reaches ENCODER_MAX_FAILS.
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

float positionDeg() {
  return countsToDeg(angleDiff(g_encRaw, (uint16_t)g_zeroCount));
}

// ---------------------------------------------------------------------------
//  Limits
// ---------------------------------------------------------------------------
//  NC contacts to GND: LOW is the healthy, closed, not-at-a-limit state.
//  A pulled-up HIGH means either the cam is tripped or the wire has broken,
//  and both should stop travel in that direction.

bool limitPos() { return digitalRead(PIN_LIM_POS) == HIGH; }
bool limitNeg() { return digitalRead(PIN_LIM_NEG) == HIGH; }

// The reference switch carries no safety duty, so unlike the cams it is wired
// only to this pin -- nothing it does interrupts motor current. Active LOW to
// GND with INPUT_PULLUP, same as everything else here.
bool refActive() { return digitalRead(PIN_REF) == LOW; }

// ---------------------------------------------------------------------------
//  Motor
// ---------------------------------------------------------------------------

static int      g_duty       = 0;    // signed, what we last commanded
static int8_t   g_lastSign   = 0;
static bool     g_limitBlocked = false;  // last call was clipped by a cam
static unsigned long g_coastUntil = 0;

void motorOff() {
  analogWrite(PIN_PWM, 0);
  digitalWrite(PIN_PWM, LOW);        // detach the PWM, whichever timer drove it
  digitalWrite(PIN_SLP, LOW);        // and put the bridge to sleep
  g_duty = 0;
  g_lastSign = 0;
}

// Signed duty in, with every interlock applied on the way through. This is
// the ONLY place the motor is energised.
void driveSigned(int duty) {
  duty = constrain(duty, -(int)SPEED_MAX, (int)SPEED_MAX);

  // Hardware already blocks travel into a tripped cam; refusing here too
  // means the firmware and the wiring agree rather than fighting. The flag
  // lets the caller tell "we are escaping a cam" (fine) from "we drove into
  // one" (a fault), which the duty alone cannot say once it has been zeroed.
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

  digitalWrite(PIN_SLP, HIGH);
  digitalWrite(PIN_DIR, sign > 0 ? HIGH : LOW);
  analogWrite(PIN_PWM, (uint8_t)abs(duty));
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

struct PidState {
  float integral;
  float prevMeas;
  bool  primed;
  bool  holdIntegral;   // set when the output saturated last tick
};
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
//  State
// ---------------------------------------------------------------------------

enum State : uint8_t { ST_IDLE, ST_MOVING, ST_JOG, ST_MANUAL, ST_STEPTEST, ST_REFPASS, ST_FAULT };
static State g_state = ST_IDLE;

enum Fault : uint8_t {
  F_NONE = 0, F_STALL, F_LINKAGE, F_ENCODER, F_LIMIT, F_TIMEOUT, F_CURRENT, F_RANGE, F_REF
};
static Fault g_fault = F_NONE;

static float         g_target      = 0.0f;
static unsigned long g_moveStarted = 0;
static unsigned long g_jogUntil    = 0;
static int           g_jogDuty     = 0;

// health window
static unsigned long g_winPulses   = 0;
static float         g_winStartDeg = 0.0f;
static unsigned long g_lastLoopMs  = 0;
static unsigned long g_lastTeleMs  = 0;

// Who last commanded the axis. Not a permission system -- every command is
// still accepted -- but the host needs to know it was pre-empted by someone
// at the panel, and "why did my move stop" should have an answer.
enum Owner : uint8_t { OWN_LOCAL, OWN_REMOTE };
static Owner g_owner = OWN_LOCAL;

// reference switch
enum RefPhase : uint8_t { RP_GOTO_START, RP_CROSS_POS, RP_GOTO_END, RP_CROSS_NEG };
static RefPhase g_refPhase = RP_GOTO_START;
static bool     g_refAdopt = false;
static bool     g_refPrev  = false;          // last polled switch state
static int16_t  g_refRawPos = REF_RAW_POS_DEFAULT;
static int16_t  g_refRawNeg = REF_RAW_NEG_DEFAULT;
static int16_t  g_refSeenPos = -1;           // captured during this pass
static int16_t  g_refSeenNeg = -1;

// step test
static unsigned long g_stepUntil = 0;
static unsigned long g_stepNext  = 0;
static unsigned long g_stepT0    = 0;

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
    case ST_FAULT:    return F("FAULT");
  }
  return F("?");
}

void raiseFault(Fault f) {
  motorOff();
  pidReset();
  g_fault = f;
  g_state = ST_FAULT;
  Serial.print(F("FAULT "));
  Serial.println(faultName(f));
}

void stopMotion(const __FlashStringHelper *why) {
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

void printStatus() {
  Serial.print(F("state="));   Serial.print(stateName(g_state));
  Serial.print(F(" pos="));    printDeg(positionDeg());
  Serial.print(F(" target=")); printDeg(g_target);
  Serial.print(F(" duty="));   Serial.print(g_duty);
  Serial.print(F(" unit=deg"));
  Serial.print(F(" enc="));    Serial.print(g_encRaw);
  Serial.print(F(" zero="));   Serial.print(g_zeroCount);
  Serial.print(F(" encok="));  Serial.print(g_encOk ? 1 : 0);
  Serial.print(F(" lim="));
  Serial.print(limitNeg() ? '-' : '.');
  Serial.print(limitPos() ? '+' : '.');
  Serial.print(F(" reed="));   Serial.print(reedPulses());
  Serial.print(F(" refpos=")); Serial.print(g_refRawPos);
  Serial.print(F(" refneg=")); Serial.print(g_refRawNeg);
  Serial.print(F(" ref="));    Serial.print(refActive() ? 1 : 0);
  Serial.print(F(" owner="));  Serial.print(g_owner == OWN_LOCAL ? F("local") : F("remote"));
  Serial.print(F(" kp="));     printDeg(g_kp);
  Serial.print(F(" ki="));     printDeg(g_ki);
  Serial.print(F(" kd="));     printDeg(g_kd);
  Serial.print(F(" kff="));    printDeg(g_kff);
  Serial.print(F(" fault="));  Serial.print(faultName(g_fault));
  Serial.println();
}

// ---------------------------------------------------------------------------
//  EEPROM
// ---------------------------------------------------------------------------

#if USE_EEPROM
struct Saved {
  uint32_t magic;
  int16_t  zero;
  int16_t  refPos;
  int16_t  refNeg;
  float    kp, ki, kd, kff;
};

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

void loadSettings() {
  Saved s;
  EEPROM.get(EEPROM_BASE_ADDR, s);
  if (s.magic != EEPROM_MAGIC) { Serial.println(F("eeprom: blank, using defaults")); return; }
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
//  Motion entry points
// ---------------------------------------------------------------------------

void beginMove(float targetDeg) {
  g_owner = OWN_REMOTE;
  if (g_state == ST_FAULT) { Serial.println(F("faulted -- 'k' first")); return; }
  if (!g_encOk)            { Serial.println(F("no encoder")); return; }

  if (fabs(targetDeg) > SOFT_LIMIT_DEG) {
    Serial.print(F("refused: outside +/-"));
    printDeg(SOFT_LIMIT_DEG);
    Serial.println(F(" deg soft limit"));
    g_fault = F_RANGE;
    return;
  }

  g_target = targetDeg;
  g_moveStarted = millis();
  g_winPulses = reedPulses();
  g_winStartDeg = positionDeg();
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
  g_moveStarted = millis();
  g_winPulses = reedPulses();
  g_winStartDeg = positionDeg();
  g_state = ST_JOG;
}

// ---------------------------------------------------------------------------
//  Panel buttons -- manual mode
// ---------------------------------------------------------------------------
//  Human-timescale debounce, which is a completely different problem from the
//  reed's: no interrupt, no series resistor, no capacitor. Just refuse to
//  believe a change until it has held for BTN_DEBOUNCE_MS.

#if USE_BUTTONS
struct Button {
  uint8_t  pin;
  bool     stable;        // debounced state, true = pressed
  bool     last;          // last raw read
  unsigned long changed;  // when the raw read last differed
};

static Button g_btnExtend  = {PIN_BTN_EXTEND,  false, false, 0};
static Button g_btnRetract = {PIN_BTN_RETRACT, false, false, 0};
static Button g_btnStop    = {PIN_BTN_STOP,    false, false, 0};

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
#endif

// ---------------------------------------------------------------------------
//  Reference switch
// ---------------------------------------------------------------------------
//  Not a hard stop, and never confused with one. The +/-10 deg cams protect
//  the antenna and are a fault when reached; this switch sits inside the
//  travel, carries no safety duty, and exists only so the stored ZERO can be
//  checked against a physical fact.
//
//  Because it is mid-travel the boom crosses it on ordinary moves, so most
//  checking is passive -- no procedure to remember. 'h' forces a deliberate
//  slow pass when the authoritative number is wanted.

// Called exactly once per tick. Returns true on an inactive->active edge.
bool refEdge() {
  const bool now = refActive();
  const bool entering = (now && !g_refPrev);
  g_refPrev = now;
  return entering;
}

// A crossing during ordinary motion. Free integrity check: if this drifts,
// the magnet has moved on its hub or the switch has, and nothing else in the
// system can see either.
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
  g_moveStarted = millis();
  g_winPulses = reedPulses();
  g_winStartDeg = positionDeg();
  pidReset();
  g_state = ST_REFPASS;

  Serial.println(adopt ? F("ref pass -- will ADOPT both crossings")
                       : F("ref pass -- check only, nothing will change"));
}

void finishRefPass() {
  motorOff();
  g_state = ST_IDLE;

  Serial.print(F("crossings: +dir raw ")); Serial.print(g_refSeenPos);
  Serial.print(F("  -dir raw "));          Serial.println(g_refSeenNeg);

  if (g_refSeenPos >= 0 && g_refSeenNeg >= 0) {
    // Lobe width plus switch hysteresis. A constant of the mechanism, so a
    // change in it means the lever is bending or the cam has worked loose --
    // which neither crossing on its own would reveal.
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

// ---------------------------------------------------------------------------
//  Manual mode
// ---------------------------------------------------------------------------

#if USE_BUTTONS
static int8_t g_manualDir = 0;

// Called every tick, ahead of the state machine, so a press pre-empts
// whatever else was running. Returns true if the buttons now own the axis.
bool handleButtons() {
  const bool stopEdge    = buttonPoll(g_btnStop);
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
      Serial.println(F("panel took over"));
    }
    g_owner = OWN_LOCAL;
    g_manualDir = extendEdge ? +1 : -1;
    g_moveStarted = millis();
    g_winPulses = reedPulses();
    g_winStartDeg = positionDeg();
    pidReset();
    g_state = ST_MANUAL;
    return true;
  }

  return g_state == ST_MANUAL;
}

// Hold-to-run. Released is stopped, that instant -- which is the entire
// reason these are buttons and not a serial command.
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

  // Soft limits apply to manual driving too. The cams are a backstop, not a
  // thing to steer by, and holding a button is not a reason to reach one.
  const float pos = positionDeg();
  if ((g_manualDir > 0 && pos >= SOFT_LIMIT_DEG) ||
      (g_manualDir < 0 && pos <= -SOFT_LIMIT_DEG)) {
    g_manualDir = 0;
    motorOff();
    g_state = ST_IDLE;
    Serial.print(F("soft limit at "));
    printDeg(pos);
    Serial.println(F(" deg"));
    return;
  }

  driveSigned(g_manualDir > 0 ? (int)SPEED_BUTTON : -(int)SPEED_BUTTON);
  if (g_limitBlocked) raiseFault(F_LIMIT);
}
#endif

// ---------------------------------------------------------------------------
//  Health checks, run on every tick while the motor is commanded on
// ---------------------------------------------------------------------------

bool healthOk() {
  const unsigned long now = millis();

  if (!g_encOk) { raiseFault(F_ENCODER); return false; }

  if (now - g_moveStarted > MAX_RUN_MS) { raiseFault(F_TIMEOUT); return false; }

#if USE_CURRENT_LIMIT
  if (analogRead(PIN_CS) > CURRENT_TRIP_ADC) { raiseFault(F_CURRENT); return false; }
#endif

  if (g_duty == 0) return true;         // coasting between directions

  // Reed silent while the motor is on. Past the breakaway grace this is a
  // stall, a cam that has cut its own current, or a dead fuse -- and from out
  // here those look identical, which is why all three stop the motor.
  const unsigned long sinceStart = now - g_moveStarted;
  const unsigned long sincePulse = now - reedLastMs();
  if (sinceStart > START_GRACE_MS && sincePulse > STALL_TIMEOUT_MS) {
    raiseFault(F_STALL);
    return false;
  }

  // Reed pulsing but the boom not moving: the motor is turning something
  // that is no longer attached to the dish. Neither sensor can see this
  // alone, and it is the whole reason the reed stayed wired.
  const unsigned long pulses = reedPulses() - g_winPulses;
  if (pulses >= LINKAGE_PULSES) {
    if (fabs(positionDeg() - g_winStartDeg) < LINKAGE_MIN_DEG) {
      raiseFault(F_LINKAGE);
      return false;
    }
    g_winPulses = reedPulses();
    g_winStartDeg = positionDeg();
  }

  return true;
}

// ---------------------------------------------------------------------------
//  Console
// ---------------------------------------------------------------------------

static char g_line[40];
static uint8_t g_len = 0;

void printHelp() {
  Serial.println(F("  <enter>    STOP"));
  Serial.println(F("  s          status"));
  Serial.println(F("  g <deg>    go to absolute angle"));
  Serial.println(F("  +<deg>     relative move   -<deg>"));
  Serial.println(F("  e <ms>     open-loop jog   r <ms>"));
  Serial.println(F("  m          monitor raw encoder (check mounting/invert)"));
  Serial.println(F("  z          set zero here"));
  Serial.println(F("  (panel)    hold extend/retract to jog; STOP aborts anything"));
  Serial.println(F("  h          cross the reference switch, report drift"));
  Serial.println(F("  H          cross it and ADOPT the crossings as zero"));
  Serial.println(F("  p/i/d/f <v> set kp/ki/kd/kff live"));
  Serial.println(F("  t <deg>    step test, CSV out"));
  Serial.println(F("  n          reed noise floor"));
  Serial.println(F("  w          save zero + gains"));
  Serial.println(F("  k          clear fault"));
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

void reedNoiseFloor() {
  Serial.println(F("reed noise floor, 5 s, bridge powered, motor OFF"));
  motorOff();
  const unsigned long start = reedPulses();
  const unsigned long t0 = millis();
  while (millis() - t0 < 5000) { /* the ISR is the measurement */ }
  Serial.print(F("edges="));
  Serial.println(reedPulses() - start);
  Serial.println(F("anything but 0 is pickup, not travel"));
}

void handleLine(char *s) {
  while (*s == ' ') s++;

  if (*s == '\0') { stopMotion(F("commanded")); return; }

  const char c = *s;
  char *arg = s + 1;
  while (*arg == ' ') arg++;
  const bool hasArg = (*arg != '\0');
  const float fv = hasArg ? atof(arg) : 0.0f;

  switch (c) {
    case 's': printStatus(); break;
    case '?': printHelp();   break;

    case 'g': if (hasArg) beginMove(fv); else Serial.println(F("g <deg>")); break;
    case '+': beginMove(positionDeg() + (hasArg ? atof(s + 1) : 0.1f)); break;
    case '-': beginMove(positionDeg() - (hasArg ? atof(s + 1) : 0.1f)); break;

    case 'e': beginJog(+1, hasArg ? (uint16_t)fv : JOG_MS_DEFAULT); break;
    case 'r': beginJog(-1, hasArg ? (uint16_t)fv : JOG_MS_DEFAULT); break;

    case 'm': monitorEncoder(); break;
    case 'n': reedNoiseFloor(); break;

    case 'z':
    case 'c':                                  // 'c' kept so an old host does
      g_zeroCount = (int16_t)g_encRaw;         // not fall over on it
      Serial.print(F("zero set at raw "));
      Serial.println(g_zeroCount);
      break;

    case 'h': beginRefPass(false); break;  // check the zero against the switch
    case 'H': beginRefPass(true);  break;  // adopt this pass as the reference

    case 'p': if (hasArg) { g_kp  = fv; Serial.println(F("kp set"));  } break;
    case 'i': if (hasArg) { g_ki  = fv; pidReset(); Serial.println(F("ki set")); } break;
    case 'd': if (hasArg) { g_kd  = fv; Serial.println(F("kd set")); } break;
    case 'f': if (hasArg) { g_kff = fv; Serial.println(F("kff set")); } break;

    case 'w': saveSettings(); break;

    case 'k':
      g_fault = F_NONE;
      g_state = ST_IDLE;
      g_encFails = 0;
      pidReset();
      Serial.println(F("fault cleared"));
      break;

    case 't':
      if (!hasArg) { Serial.println(F("t <deg>")); break; }
      beginMove(fv);
      if (g_state == ST_MOVING) {
        g_state = ST_STEPTEST;
        g_stepT0 = millis();
        g_stepUntil = g_stepT0 + STEP_TEST_MS;
        g_stepNext = g_stepT0;
        Serial.println(F("ms,target,pos,duty"));
      }
      break;

    default:
      Serial.println(F("? for help"));
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
  // Claim the bridge low BEFORE anything else. An unconfigured output is a
  // floating input, and a driver gets to decide for itself what that means.
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_SLP, OUTPUT);
  digitalWrite(PIN_PWM, LOW);
  digitalWrite(PIN_DIR, LOW);
  digitalWrite(PIN_SLP, LOW);

  pinMode(PIN_REED,    INPUT_PULLUP);
  pinMode(PIN_LIM_POS, INPUT_PULLUP);
  pinMode(PIN_LIM_NEG, INPUT_PULLUP);
  pinMode(PIN_REF,     INPUT_PULLUP);
#if USE_BUTTONS
  pinMode(PIN_BTN_EXTEND,  INPUT_PULLUP);
  pinMode(PIN_BTN_RETRACT, INPUT_PULLUP);
  pinMode(PIN_BTN_STOP,    INPUT_PULLUP);
#endif
#if USE_BUZZER
  pinMode(PIN_BUZZER, OUTPUT);
#endif

  attachInterrupt(digitalPinToInterrupt(PIN_REED), onReedEdge, FALLING);

  Serial.begin(115200);
  Serial.println(F("MOON actuator_v2 -- Rev B"));

  Wire.begin();
  Wire.setClock(400000);

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

  encoderPoll();
  g_refPrev = refActive();   // so boot never looks like a crossing
  if (!g_encOk) {
    raiseFault(F_ENCODER);
  } else {
    Serial.print(F("boot position "));
    printDeg(positionDeg());
    Serial.println(F(" deg -- no homing needed"));
  }

  Serial.println(F("? for help"));
  g_lastLoopMs = millis();
}

void loop() {
  pollSerial();

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
      if (!healthOk()) break;

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

      // Driving INTO a cam is a fault. The cams are hard stops protecting the
      // antenna, not waypoints, so reaching one under a normal move means
      // something is wrong -- the soft limit should have stopped it first.
      if (g_limitBlocked) { raiseFault(F_LIMIT); break; }

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

      if (out == 0 && fabs(g_target - pos) <= DEADBAND_DEG) {
        motorOff();
        g_state = ST_IDLE;
        Serial.print(F("landed at ")); printDeg(pos);
        Serial.print(F(" err "));      printDeg(g_target - pos);
        Serial.println();
      }
      break;
    }

    case ST_REFPASS:
      if (!healthOk()) break;
      refPassTick(refEntering);
      break;

#if USE_BUTTONS
    case ST_MANUAL:
      if (!healthOk()) break;
      manualTick();
      refPassive(refEntering);
      break;
#endif

    case ST_JOG:
      if (!healthOk()) break;
      if (now >= g_jogUntil) { stopMotion(F("jog done")); break; }
      driveSigned(g_jogDuty);
      refPassive(refEntering);
      // A jog into a cam faults too; a jog AWAY from one is how you escape,
      // and driveSigned does not flag that.
      if (g_limitBlocked) { raiseFault(F_LIMIT); }
      break;

    case ST_IDLE:
    case ST_FAULT:
    default:
      motorOff();
      break;
  }

  if (g_state != ST_STEPTEST && now - g_lastTeleMs >= TELEMETRY_MS) {
    g_lastTeleMs = now;
    if (g_state == ST_MOVING || g_state == ST_JOG ||
        g_state == ST_REFPASS || g_state == ST_MANUAL) printStatus();
  }
}
