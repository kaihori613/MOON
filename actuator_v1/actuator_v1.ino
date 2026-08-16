#include "Config.h"

volatile long          g_counts      = 0;   // signed position, in reed counts
volatile unsigned long g_pulses      = 0;   // lifetime total, unsigned
volatile unsigned long g_lastPulseUs = 0;
volatile unsigned long g_lastGapUs   = 0;
volatile unsigned long g_lastPulseMs = 0;

// Latched before the motor is energised and deliberately never cleared on
// stop, so pulses that arrive while the carriage coasts are counted in the
// direction it is actually still moving.
volatile int8_t g_countSign = +1;

// Live-adjustable with 'd', so the debounce can be retuned against a running
// bridge without a reflash.
unsigned long g_debounceUs = REED_DEBOUNCE_US;

void onReedEdge() {
  const unsigned long now = micros();

  if (now - g_lastPulseUs < g_debounceUs) return;   // bounce, or pickup

  g_lastGapUs   = now - g_lastPulseUs;
  g_lastPulseUs = now;
  g_lastPulseMs = millis();
  g_pulses++;
  g_counts += g_countSign;
}

// A long is four bytes on AVR, so reading one while the interrupt may fire
// mid-read is a genuine tear, not a theoretical one.
long position() {
  noInterrupts();
  const long n = g_counts;
  interrupts();
  return n;
}

unsigned long pulseTotal() {
  noInterrupts();
  const unsigned long n = g_pulses;
  interrupts();
  return n;
}

unsigned long lastPulseMs() {
  noInterrupts();
  const unsigned long n = g_lastPulseMs;
  interrupts();
  return n;
}

float pulseHz() {
  noInterrupts();
  const unsigned long gap = g_lastGapUs;
  interrupts();
  return (gap == 0) ? 0.0 : (1000000.0 / gap);
}

void setPosition(long n) {
  noInterrupts();
  g_counts = n;
  interrupts();
}

// ---------------------------------------------------------------------------
//  Motor
// ---------------------------------------------------------------------------
//  Lifted from the bring-up sketches unchanged. The only addition is latching
//  the count sign before any current flows.

char    g_dir     = 0;    // 'e', 'r', or 0 when the motor is off
uint8_t g_applied = 0;    // what is actually on the pin

void motorOff() {
#if MOTOR_DRIVER == DRV_L298N
  // ENA first: cut the output stage before touching direction, so the INs
  // never change state with the bridge live.
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
#else
  // With the enables possibly tied high, these two pins are the whole of the
  // off switch. digitalWrite rather than analogWrite(0) detaches the PWM
  // whichever timer drove it.
  digitalWrite(PIN_RPWM, LOW);
  digitalWrite(PIN_LPWM, LOW);
  digitalWrite(PIN_EN,   LOW);
#endif
  g_applied = 0;
}

void motorApply(char dir, uint8_t speed) {
  // Before any current flows, so the first pulse of the run is already being
  // counted the right way.
  noInterrupts();
  g_countSign = (dir == 'e') ? +1 : -1;
  interrupts();

#if MOTOR_DRIVER == DRV_L298N
  // Direction set while the output stage is off, then speed. IN1 and IN2 must
  // always be opposites: equal levels are brake (both high) or coast (both
  // low), and neither of those turns the motor.
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_IN1, dir == 'e' ? HIGH : LOW);
  digitalWrite(PIN_IN2, dir == 'e' ? LOW  : HIGH);
  analogWrite(PIN_ENA, speed);
#else
  // Idle side hard off before the driven side goes anywhere near on.
  if (dir == 'e') {
    digitalWrite(PIN_LPWM, LOW);
    digitalWrite(PIN_EN,   HIGH);
    analogWrite(PIN_RPWM, speed);
  } else {
    digitalWrite(PIN_RPWM, LOW);
    digitalWrite(PIN_EN,   HIGH);
    analogWrite(PIN_LPWM, speed);
  }
#endif
  g_applied = speed;
}

// ---------------------------------------------------------------------------
//  Machine state
// ---------------------------------------------------------------------------

enum State : uint8_t {
  ST_IDLE,        // motor off, nothing pending
  ST_JOG,         // open loop, until time runs out or something stops it
  ST_SEEK,        // closed loop, toward g_target
  ST_HOME,        // retracting into the near stop to establish zero
  ST_MEASURE,     // extending into the far stop to learn the travel
  ST_FAULT        // motor off, needs 'k'
};

enum Fault : uint8_t {
  FAULT_NONE,
  FAULT_STALL,      // pulses stopped mid-move: a jam, or an unexpected end
  FAULT_TIMEOUT,    // ran past MAX_RUN_MS with nothing else stopping it
  FAULT_NOT_HOMED   // absolute move asked for before there was an origin
};

State g_state = ST_IDLE;
Fault g_fault = FAULT_NONE;

long g_target       = 0;      // absolute, in counts
long g_travel       = 0;      // learned by 'c'; 0 means not calibrated
bool g_homed        = false;
bool g_hitHardLimit = false;  // the last stop was something refusing to move
bool g_calibrating  = false;  // the home leg is part of a calibration

unsigned long g_motionStartMs = 0;
unsigned long g_pulsesAtStart = 0;
unsigned long g_nextTelemMs   = 0;
unsigned long g_jogEndMs      = 0;   // 0 means run until stopped

// Seek bookkeeping.
char    g_seekDir       = 'e';
uint8_t g_seekSpeed     = SPEED_RUN;
bool    g_seekSettling  = false;
unsigned long g_settleUntil = 0;
uint8_t g_corrections   = 0;   // used so far this move
long    g_seekStartPos  = 0;

// Console settings.
uint8_t  g_speed   = SPEED_RUN;
long     g_step    = STEP_COUNTS_DEFAULT;
uint16_t g_jogMs   = JOG_MS_DEFAULT;
bool     g_telem   = true;

const __FlashStringHelper* stateName() {
  switch (g_state) {
    case ST_IDLE:    return F("IDLE");
    case ST_JOG:     return F("JOG");
    case ST_SEEK:    return F("SEEK");
    case ST_HOME:    return F("HOME");
    case ST_MEASURE: return F("MEASURE");
    default:         return F("FAULT");
  }
}

const __FlashStringHelper* faultName() {
  switch (g_fault) {
    case FAULT_STALL:     return F("STALL");
    case FAULT_TIMEOUT:   return F("TIMEOUT");
    case FAULT_NOT_HOMED: return F("NOT_HOMED");
    default:              return F("none");
  }
}

// ---------------------------------------------------------------------------
//  Motion primitives
// ---------------------------------------------------------------------------

void startMotion(char dir, uint8_t speed) {
  // Changing direction under load without the coast is the transition
  // REVERSE_DEAD_MS exists to prevent. Blocking for 200 ms is acceptable here
  // precisely because the motor is off for all of it.
  if (g_dir != 0 && g_dir != dir) {
    motorOff();
    delay(REVERSE_DEAD_MS);
  }

  g_dir           = dir;
  g_motionStartMs = millis();
  g_pulsesAtStart = pulseTotal();
  g_nextTelemMs   = g_motionStartMs + TELEMETRY_MS;
  g_hitHardLimit  = false;

  motorApply(dir, speed);
}

void haltMotor() {
  motorOff();
  g_dir = 0;
}

void enterIdle() {
  haltMotor();
  g_state       = ST_IDLE;
  g_jogEndMs    = 0;
  g_calibrating = false;   // a calibration stopped part-way is not still owed
}

void enterFault(Fault f) {
  haltMotor();
  g_state = ST_FAULT;
  g_fault = f;

  Serial.print(F("  FAULT: "));
  Serial.print(faultName());
  Serial.print(F("  at pos="));
  Serial.println(position());
  Serial.println(F("  Motor is off. 'k' clears it once you know why."));
}

// Motor commanded on, and nothing arriving from the sensor. Either the cam cut
// current at an end of travel or the carriage is stuck; from out here those
// are the same event.
bool motionStalled(unsigned long now) {
  if (g_dir == 0) return false;

  if (pulseTotal() == g_pulsesAtStart) {
    // Nothing at all yet this run, so the only question is whether it has had
    // long enough to break away.
    return (now - g_motionStartMs) >= START_GRACE_MS;
  }
  return (now - lastPulseMs()) >= STALL_TIMEOUT_MS;
}

// ---------------------------------------------------------------------------
//  Soft limits
// ---------------------------------------------------------------------------
//  Only meaningful once both the origin and the span are known. Uncalibrated,
//  the mechanism's own cam switches are the only limits there are -- which is
//  survivable, since that is what they are for.

bool haveSoftLimits() { return g_homed && g_travel > 0; }
long softMin() { return SOFT_LIMIT_MARGIN; }
long softMax() { return g_travel - SOFT_LIMIT_MARGIN; }

long clampToLimits(long counts, bool* clamped) {
  *clamped = false;
  if (!haveSoftLimits()) return counts;

  if (counts < softMin()) { *clamped = true; return softMin(); }
  if (counts > softMax()) { *clamped = true; return softMax(); }
  return counts;
}

// ---------------------------------------------------------------------------
//  Commands
// ---------------------------------------------------------------------------

void stopNow(const __FlashStringHelper* why) {
  if (g_state == ST_IDLE || g_state == ST_FAULT) {
    haltMotor();                       // belt and braces: pins low regardless
    Serial.println(F("  already stopped"));
    return;
  }

  const unsigned long elapsed = millis() - g_motionStartMs;
  const unsigned long moved   = pulseTotal() - g_pulsesAtStart;

  enterIdle();

  Serial.print(F("  STOP ("));
  Serial.print(why);
  Serial.print(F(")  ran "));
  Serial.print(elapsed / 1000.0, 2);
  Serial.print(F(" s, "));
  Serial.print(moved);
  Serial.print(F(" pulses, pos="));
  Serial.println(position());
}

bool startSeek(long target, uint8_t speed) {
  if (g_state == ST_FAULT) {
    Serial.println(F("  faulted -- 'k' first"));
    return false;
  }

  bool clamped = false;
  target = clampToLimits(target, &clamped);
  if (clamped) {
    Serial.print(F("  clamped to the soft limit: "));
    Serial.println(target);
  }

  const long here = position();
  const long err  = target - here;

  if (labs(err) <= DEADBAND_COUNTS) {
    Serial.print(F("  already there (pos="));
    Serial.print(here);
    Serial.println(F(")"));
    return true;
  }

  g_target       = target;
  g_seekDir      = (err > 0) ? 'e' : 'r';
  g_seekSpeed    = speed;
  g_seekSettling = false;
  g_corrections  = 0;
  g_seekStartPos = here;
  g_state        = ST_SEEK;

  Serial.print(F("  SEEK "));
  Serial.print(here);
  Serial.print(F(" -> "));
  Serial.print(target);
  Serial.print(F("  ("));
  Serial.print(err > 0 ? F("extend ") : F("retract "));
  Serial.print(labs(err));
  Serial.println(F(" counts)"));

  startMotion(g_seekDir, (labs(err) <= SLOW_ZONE_COUNTS) ? SPEED_TRIM : speed);
  return true;
}

void startJog(char dir, unsigned long ms, uint8_t speed) {
  if (g_state == ST_FAULT) {
    Serial.println(F("  faulted -- 'k' first"));
    return;
  }
  if (speed == 0) {
    Serial.println(F("  speed is zero -- 'v <n>' first"));
    return;
  }

  g_state    = ST_JOG;
  g_jogEndMs = (ms == 0) ? 0 : millis() + ms;

  startMotion(dir, speed);

  Serial.print(F("  JOG "));
  Serial.print(dir == 'e' ? F("EXTEND") : F("RETRACT"));
  Serial.print(F("  speed "));
  Serial.print(speed);
  if (ms == 0) {
    Serial.println(F("  (continuous -- Enter stops)"));
  } else {
    Serial.print(F("  ")); Serial.print(ms); Serial.println(F(" ms"));
  }
}

void startHome(bool thenMeasure) {
  if (g_state == ST_FAULT) {
    Serial.println(F("  faulted -- 'k' first"));
    return;
  }

  g_calibrating = thenMeasure;
  g_state       = ST_HOME;
  g_jogEndMs    = 0;

  Serial.println(F("  HOMING -- retracting into the near stop."));
  Serial.println(F("  This drives deliberately into a hard stop and waits for"));
  Serial.println(F("  the cam to cut current. Watch the ammeter."));

  startMotion('r', SPEED_HOMING);
}

void zeroHere() {
  setPosition(0);
  g_target = 0;
  g_homed  = true;

  // The travel figure was measured from the old origin. Keeping it here would
  // leave the soft limits sitting somewhere that no longer corresponds to the
  // mechanical stops, which is worse than having no soft limits at all.
  if (g_travel > 0) {
    g_travel = 0;
    Serial.println(F("  travel forgotten -- it was measured from the old"));
    Serial.println(F("  origin, so the soft limits no longer line up. Run 'c'."));
  }
  Serial.println(F("  zero = here"));
}

// ---------------------------------------------------------------------------
//  The loop, per state
// ---------------------------------------------------------------------------

void reportLanding() {
  const long here = position();
  const long err  = here - g_target;

  Serial.print(F("  landed pos="));
  Serial.print(here);
  Serial.print(F("  target="));
  Serial.print(g_target);
  Serial.print(F("  error="));
  Serial.print(err);
  Serial.print(F("  moved="));
  Serial.print(here - g_seekStartPos);
  if (g_corrections > 0) {
    Serial.print(F("  ("));
    Serial.print(g_corrections);
    Serial.print(F(" correction"));
    Serial.print(g_corrections == 1 ? F(")") : F("s)"));
  }
  Serial.println();

  if (err != 0) {
    const bool overshot = (g_seekDir == 'e') ? (err > 0) : (err < 0);
    if (overshot) {
      // Left alone on purpose. See the backlash note in the file header.
      Serial.println(F("  Overshot. Not corrected -- reversing to recover this"));
      Serial.println(F("  would take up the linkage backlash again and land"));
      Serial.println(F("  less predictably than the error it is fixing."));
    } else {
      Serial.println(F("  Short, and out of corrections. Raise MAX_CORRECTIONS,"));
      Serial.println(F("  or SPEED_TRIM is too low to break away for one count."));
    }
  }
}

void updateSeek(unsigned long now) {
  if (g_seekSettling) {
    // Power is off and the carriage is still moving. Those pulses are real
    // position, so nothing is decided until they stop arriving.
    if (now < g_settleUntil) return;

    const long here = position();
    const long err  = g_target - here;

    if (labs(err) <= DEADBAND_COUNTS) {
      enterIdle();
      reportLanding();
      return;
    }

    const bool sameWay = (g_seekDir == 'e') ? (err > 0) : (err < 0);

    if (sameWay && g_corrections < MAX_CORRECTIONS) {
      g_corrections++;
      g_seekSettling = false;
      Serial.print(F("  short by "));
      Serial.print(labs(err));
      Serial.println(F(" -- creeping on at trim speed"));
      startMotion(g_seekDir, SPEED_TRIM);
      return;
    }

    enterIdle();
    reportLanding();
    return;
  }

  // --- driving --------------------------------------------------------------
  const long here = position();
  const bool reached = (g_seekDir == 'e') ? (here >= g_target) : (here <= g_target);

  if (reached) {
    haltMotor();
    g_seekSettling = true;
    g_settleUntil  = now + COAST_SETTLE_MS;
    return;
  }

  // Ease off for the last few counts. Only write on a real change: reloading
  // the timer every pass is wasted work.
  const long remaining = labs(g_target - here);
  const uint8_t want = (remaining <= SLOW_ZONE_COUNTS) ? SPEED_TRIM : g_seekSpeed;
  if (want != g_applied) motorApply(g_dir, want);
}

// Pulses have stopped while motion is still commanded. Whether that is good
// news depends entirely on what we were trying to do.
void onStall() {
  haltMotor();
  g_hitHardLimit = true;

  switch (g_state) {
    case ST_HOME:
      setPosition(0);
      g_homed  = true;
      g_target = 0;
      Serial.println(F("  reached the near stop -- this is zero"));

      if (g_calibrating) {
        g_state = ST_MEASURE;
        Serial.println(F("  MEASURING -- extending into the far stop."));
        startMotion('e', SPEED_HOMING);
      } else {
        g_state  = ST_IDLE;
        g_travel = 0;      // origin moved; any old span no longer applies
      }
      return;

    case ST_MEASURE:
      g_travel      = position();
      g_calibrating = false;
      g_state       = ST_IDLE;

      Serial.print(F("  reached the far stop -- travel = "));
      Serial.print(g_travel);
      Serial.println(F(" counts"));
      Serial.print(F("  soft limits now "));
      Serial.print(softMin());
      Serial.print(F(" .. "));
      Serial.println(softMax());

      if (g_travel < 10) {
        Serial.println(F("  That is implausibly short. Either it never left the"));
        Serial.println(F("  near stop, or the sensor is not producing pulses."));
      }
      return;

    case ST_JOG:
      // Expected, and useful: this is how a jog finds an end of travel.
      g_state = ST_IDLE;
      Serial.print(F("  stopped: pulses ceased while still driving "));
      Serial.print(g_dir == 'e' ? F("extend") : F("retract"));
      Serial.print(F(", pos="));
      Serial.println(position());
      Serial.println(F("  End of travel, or a jam. Nothing out here can tell"));
      Serial.println(F("  those apart, so the motor is off either way."));
      return;

    default:
      // Mid-seek this is unambiguous trouble: the target was inside the soft
      // limits, so the carriage should not have run out of anywhere to go.
      enterFault(FAULT_STALL);
      return;
  }
}

void updateMotion() {
  if (g_state == ST_IDLE || g_state == ST_FAULT) return;

  const unsigned long now = millis();

  if (g_dir != 0) {
    if (now - g_motionStartMs >= MAX_RUN_MS) {
      Serial.println(F("  MAX_RUN_MS -- nothing else stopped it. A full stroke"));
      Serial.println(F("  should be well short of this."));
      enterFault(FAULT_TIMEOUT);
      return;
    }

    if (motionStalled(now)) {
      onStall();
      return;
    }

    if (g_telem && now >= g_nextTelemMs) {
      g_nextTelemMs = now + TELEMETRY_MS;
      Serial.print(F("  pos="));    Serial.print(position());
      Serial.print(F(" target=")); Serial.print(g_target);
      Serial.print(F(" pwm="));    Serial.print(g_applied);
      Serial.print(F(" hz="));     Serial.println(pulseHz(), 1);
    }
  }

  switch (g_state) {
    case ST_SEEK:
      updateSeek(now);
      break;

    case ST_JOG:
      if (g_jogEndMs != 0 && now >= g_jogEndMs) stopNow(F("jog complete"));
      break;

    default:
      break;    // HOME and MEASURE end at a stall, handled above
  }
}

// ---------------------------------------------------------------------------
//  Noise floor
// ---------------------------------------------------------------------------
//  With the carriage stationary every edge is electrical. This is the single
//  most useful number the sensor can produce, because noise counts are
//  indistinguishable from real ones downstream: they corrupt position and
//  nothing later can detect or undo it.
//
//  Worth re-running now in a way reed_switch_test never could -- with the
//  bridge powered and in circuit, which is the configuration that matters.

const unsigned long NOISE_TEST_MS = 10000;

void noiseFloorTest() {
  if (g_state != ST_IDLE) {
    Serial.println(F("  not while it is moving"));
    return;
  }

  Serial.println(F("  Noise floor: 10 s, carriage must be STILL. Leave the"));
  Serial.println(F("  driver powered -- an idle bridge still radiates, and"));
  Serial.println(F("  that is exactly what wants measuring."));

  const long before = position();
  const unsigned long start   = millis();
  const unsigned long edges0  = pulseTotal();

  while (millis() - start < NOISE_TEST_MS) {
    if ((millis() - start) % 1000 < 20) {
      Serial.print('.');
      delay(25);
    }
  }
  Serial.println();

  const unsigned long edges = pulseTotal() - edges0;

  Serial.print(F("  edges in 10 s: "));
  Serial.println(edges);

  if (edges == 0) {
    Serial.println(F("  Clean. No pickup with the carriage stopped."));
  } else {
    Serial.println(F("  PICKUP. The controller believes these. Position will"));
    Serial.println(F("  drift and nothing downstream can see it happen."));
    Serial.println(F("  Fix before trusting any move: 4.7k pull-up to 5V, 220R"));
    Serial.println(F("  in series with the reed, 220nF to ground at the pin,"));
    Serial.println(F("  and a ground return that does not carry motor current."));

    // Nothing moved, so whatever those counts did to the position is wrong by
    // definition. Undoing it is the only honest option.
    setPosition(before);
    Serial.print(F("  position restored to "));
    Serial.println(before);
  }
}

// ---------------------------------------------------------------------------
//  Console
// ---------------------------------------------------------------------------

void printStatus() {
  Serial.print(F("state="));    Serial.print(stateName());
  Serial.print(F("  pos="));    Serial.print(position());
  Serial.print(F("  target=")); Serial.print(g_target);

  Serial.print(F("  travel="));
  if (g_travel > 0) Serial.print(g_travel);
  else              Serial.print(F("? (run c)"));

  Serial.print(F("  homed="));  Serial.print(g_homed ? F("yes") : F("NO"));
  Serial.print(F("  dir="));
  Serial.print(g_dir == 'e' ? F("ext") : g_dir == 'r' ? F("ret") : F("--"));
  Serial.print(F("  pwm="));    Serial.print(g_applied);
  Serial.print(F("  hz="));     Serial.print(pulseHz(), 1);
  Serial.print(F("  pulses=")); Serial.print(pulseTotal());
  Serial.print(F("  step="));   Serial.print(g_step);
  Serial.print(F("  speed="));  Serial.print(g_speed);

  if (g_hitHardLimit && g_state == ST_IDLE) Serial.print(F("  [at hard stop]"));
  if (g_fault != FAULT_NONE) {
    Serial.print(F("  FAULT: "));
    Serial.print(faultName());
  }
  Serial.println();
}

void printHelp() {
  Serial.println(F("--- fine positioning -----------------------------------"));
  Serial.println(F("  +<n> / -<n>  move n counts, closed loop, then report"));
  Serial.println(F("  + / -        move one step (see 'i'), same thing"));
  Serial.println(F("  g <n>        go to absolute count n (needs 'h' or 'c')"));
  Serial.println(F("  i <n>        set the step used by a bare + or -"));
  Serial.println(F("--- open loop, no sensor needed ------------------------"));
  Serial.println(F("  e [ms]       jog EXTEND, default from 'j'"));
  Serial.println(F("  r [ms]       jog RETRACT"));
  Serial.println(F("  E / R        run until stopped"));
  Serial.println(F("  j <ms>       set the default jog time"));
  Serial.println(F("--- reference -------------------------------------------"));
  Serial.println(F("  h            home: retract into the near stop, call it 0"));
  Serial.println(F("  c            calibrate: home, then learn the full travel"));
  Serial.println(F("  z            zero here (forgets travel -- see the code)"));
  Serial.println(F("--- everything else ------------------------------------"));
  Serial.println(F("  <Enter>      STOP  (also 'x')"));
  Serial.println(F("  s            status"));
  Serial.println(F("  v <n>        speed, 0-255"));
  Serial.println(F("  d [us]       reed debounce, live"));
  Serial.println(F("  n            noise floor, 10 s, carriage still"));
  Serial.println(F("  t            telemetry while moving on/off"));
  Serial.println(F("  k            clear a fault"));
  Serial.println(F("  ?            this help"));
  Serial.println(F("--------------------------------------------------------"));
}

const char* skipSpace(const char* s) {
  while (*s == ' ' || *s == '\t') s++;
  return s;
}

// Rejects trailing rubbish rather than quietly taking the leading digits, and
// rejects an empty argument rather than calling it zero. "v" and "v 12x" are
// both mistakes, and a speed of 0 is a very different instruction from a typo.
bool parseLong(const char* s, long* out) {
  s = skipSpace(s);
  if (*s == '\0') return false;

  char* end;
  const long v = strtol(s, &end, 10);
  if (end == s) return false;
  if (*skipSpace(end) != '\0') return false;

  *out = v;
  return true;
}

void cmdRelative(const char* line) {
  char* end;
  const long v = strtol(line, &end, 10);

  long delta;
  if (end == line) {
    delta = (line[0] == '+') ? g_step : -g_step;   // bare sign
  } else if (*skipSpace(end) != '\0') {
    Serial.println(F("  garbage after the number"));
    return;
  } else {
    delta = v;
  }

  if (delta == 0) {
    Serial.println(F("  zero counts -- nothing to do"));
    return;
  }
  startSeek(position() + delta, g_speed);
}

void cmdJog(char dir, const char* arg, bool continuous) {
  if (continuous) {
    startJog(dir, 0, g_speed);
    return;
  }

  long ms = g_jogMs;
  if (*skipSpace(arg) != '\0' && !parseLong(arg, &ms)) {
    Serial.println(F("  jog time must be a number of ms"));
    return;
  }
  startJog(dir, (unsigned long)constrain(ms, (long)JOG_MS_MIN, (long)JOG_MS_MAX),
           g_speed);
}

void cmdSpeed(const char* arg) {
  long v;
  if (!parseLong(arg, &v)) {
    Serial.print(F("  speed = ")); Serial.println(g_speed);
    return;
  }

  v = constrain(v, 0L, 255L);
  // Below the floor the actuator buzzes without moving, so snap to off rather
  // than leave it sitting in the dead band drawing current for nothing.
  if (v > 0 && v < SPEED_FLOOR) {
    Serial.print(F("  under SPEED_FLOOR ("));
    Serial.print(SPEED_FLOOR);
    Serial.println(F(") -- it would buzz, not move. Snapped up."));
    v = SPEED_FLOOR;
  }

  g_speed = (uint8_t)v;
  Serial.print(F("  speed = ")); Serial.println(g_speed);

  // Only open-loop jogs follow the console speed live; a seek is running its
  // own approach profile and would fight this.
  if (g_state == ST_JOG && g_dir != 0) motorApply(g_dir, g_speed);
}

void cmdDebounce(const char* arg) {
  long v;
  if (!parseLong(arg, &v)) {
    Serial.print(F("  debounce = ")); Serial.print(g_debounceUs);
    Serial.println(F(" us"));
    return;
  }
  if (g_state != ST_IDLE) {
    Serial.println(F("  not while it is moving"));
    return;
  }

  v = constrain(v, 0L, 100000L);
  noInterrupts();
  g_debounceUs = (unsigned long)v;
  interrupts();

  Serial.print(F("  debounce = ")); Serial.print(g_debounceUs);
  Serial.println(F(" us"));
  Serial.println(F("  Copy the value you settle on into Config.h."));
}

void handleCommand(char* line) {
  // Bare Enter is the panic stop, and it is the reason this parser reads a
  // whole line rather than single characters: it is the one input that must
  // work with no argument, in every state, without a second thought.
  if (line[0] == '\0') {
    stopNow(F("Enter"));
    return;
  }

  if (line[0] == '+' || line[0] == '-') {
    cmdRelative(line);
    return;
  }

  const char cmd = line[0];
  const char* arg = line + 1;

  long v;

  switch (cmd) {
    case '?': printHelp();   break;
    case 's': printStatus(); break;
    case 'x': stopNow(F("commanded")); break;

    case 'e': cmdJog('e', arg, false); break;
    case 'r': cmdJog('r', arg, false); break;
    case 'E': cmdJog('e', arg, true);  break;
    case 'R': cmdJog('r', arg, true);  break;

    case 'g':
      if (!parseLong(arg, &v)) {
        Serial.println(F("  g needs a target: 'g 200'"));
        break;
      }
      if (!g_homed) {
        // Absolute means nothing without an origin, and guessing one silently
        // is how a dish ends up driven into its own stop.
        Serial.println(F("  no origin -- run 'h' or 'c' first"));
        enterFault(FAULT_NOT_HOMED);
        break;
      }
      startSeek(v, g_speed);
      break;

    case 'i':
      if (!parseLong(arg, &v) || v == 0) {
        Serial.print(F("  step = ")); Serial.println(g_step);
        break;
      }
      g_step = labs(v);
      Serial.print(F("  step = ")); Serial.println(g_step);
      break;

    case 'j':
      if (!parseLong(arg, &v)) {
        Serial.print(F("  jog time = ")); Serial.print(g_jogMs);
        Serial.println(F(" ms"));
        break;
      }
      g_jogMs = (uint16_t)constrain(v, (long)JOG_MS_MIN, (long)JOG_MS_MAX);
      Serial.print(F("  jog time = ")); Serial.print(g_jogMs);
      Serial.println(F(" ms"));
      break;

    case 'v': cmdSpeed(arg);    break;
    case 'd': cmdDebounce(arg); break;
    case 'n': noiseFloorTest(); break;

    case 'h': startHome(false); break;
    case 'c': startHome(true);  break;

    case 'z':
      if (g_state != ST_IDLE) {
        Serial.println(F("  not while it is moving"));
        break;
      }
      zeroHere();
      break;

    case 't':
      g_telem = !g_telem;
      Serial.print(F("  telemetry "));
      Serial.println(g_telem ? F("on") : F("off"));
      break;

    case 'k':
      if (g_fault == FAULT_NONE) {
        Serial.println(F("  no fault to clear"));
        break;
      }
      g_fault = FAULT_NONE;
      g_state = ST_IDLE;
      Serial.println(F("  fault cleared"));
      break;

    default:
      // A typo while the motor is running is far more likely a slip than an
      // instruction, and stopping is the cheap outcome.
      if (g_state != ST_IDLE && g_state != ST_FAULT) stopNow(F("unknown command"));
      Serial.println(F("  unknown -- ? for help"));
      break;
  }
}

char    g_line[32];
uint8_t g_len = 0;

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

// ---------------------------------------------------------------------------

void setup() {
  // Outputs low before anything that can block, so a reset brings the bridge
  // down immediately rather than after the serial handshake.
#if MOTOR_DRIVER == DRV_L298N
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
#else
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
#endif
  motorOff();

  pinMode(PIN_REED, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_REED), onReedEdge, FALLING);

  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println();
  Serial.println(F("=== Actuator System Bench Console ==="));
  Serial.print(F("v1, driver = "));
#if MOTOR_DRIVER == DRV_L298N
  Serial.println(F("L298N (ENA D9, IN1 D6, IN2 D5)"));
#else
  Serial.println(F("HW-039 (RPWM D9, LPWM D10, EN D8)"));
#endif
  Serial.println(F("Reed on D2. Position is counted, not measured -- it is"));
  Serial.println(F("only right while nothing back-drives the actuator."));
  Serial.println();
  Serial.println(F("Nothing is homed. Relative moves (+3, -1) work anyway;"));
  Serial.println(F("absolute moves need 'h' or 'c' first."));
  Serial.println();
  Serial.println(F("Supply 24 V, current limit 3-5 A. If the CC light comes on,"));
  Serial.println(F("the supply is limiting and the motor is not getting 24 V."));
  Serial.println(F("Bare Enter stops. Use it whenever anything looks wrong."));
  Serial.println();
  printHelp();
}

void loop() {
  pollSerial();
  updateMotion();
}
