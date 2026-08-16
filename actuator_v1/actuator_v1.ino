#include "Config.h"

#if USE_LCD
  #include <Wire.h>
#endif
#if USE_EEPROM
  #include <EEPROM.h>
#endif

// Defined down with the persistence code, called from motion code above it.
// The IDE would synthesise this prototype; saying it out loud means the file
// also compiles with a plain avr-g++ invocation.
void saveState();

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

// Angle readout. Anchored to the retract stop, not to position zero, so
// moving the origin does not silently invalidate it.
float g_degPerCount  = DEG_PER_COUNT_DEFAULT;
float g_degAtRetract = DEG_AT_RETRACT_DEFAULT;

// Two-point angle calibration, held between 'a' and 'b'.
bool g_havePointA = false;
long g_posA       = 0;
float g_degA      = 0.0f;

// Drift check. A saved position is loaded at boot and believed provisionally;
// the next home is what tests it, and only if nothing has moved since.
bool g_driftArmed = false;
bool g_virginBoot = true;

// ---------------------------------------------------------------------------
//  Where zero sits
// ---------------------------------------------------------------------------
//  The retract cam is always the reference. This is only where we choose to
//  put the number zero relative to it.

long retractStopPos() {
#if ORIGIN_AT_MIDPOINT
  if (g_travel > 0) return -(g_travel / 2);
#endif
  return 0;
}

bool haveAngle() { return g_degPerCount != 0.0f; }

float degreesNow() {
  return g_degAtRetract + (position() - retractStopPos()) * g_degPerCount;
}

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
  g_virginBoot    = false;    // the drift check only survives an untouched boot
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
  saveState();
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
long softMin() { return retractStopPos() + SOFT_LIMIT_MARGIN; }
long softMax() { return retractStopPos() + g_travel - SOFT_LIMIT_MARGIN; }

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

  // Only a home that is the first motion since boot can test the saved
  // position -- anything else has already moved the carriage out from under it.
  g_driftArmed  = g_virginBoot && g_driftArmed;

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

  // Unlike 'h', this invents an origin with no physical reference behind it.
  // Everything anchored to the retract stop is therefore now meaningless.
  if (g_travel > 0) {
    g_travel = 0;
    Serial.println(F("  travel forgotten -- it was measured from the old"));
    Serial.println(F("  origin, so the soft limits no longer line up. Run 'c'."));
  }
  if (haveAngle()) {
    g_degPerCount = 0.0f;
    Serial.println(F("  angle calibration dropped -- it was anchored to the"));
    Serial.println(F("  retract stop, which this no longer locates. Redo a/b."));
  }
  Serial.println(F("  zero = here"));
  saveState();
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
      // The drift check, and the only chance to make it. If a saved position
      // was loaded and nothing has moved since boot, the counter should have
      // wound down to exactly the retract stop. Whatever is left over is how
      // far the dish moved while the power was off.
      if (g_driftArmed) {
        const long drift = position() - retractStopPos();
        Serial.print(F("  drift since last save: "));
        Serial.print(drift);
        Serial.println(F(" counts"));
        if (drift == 0) {
          Serial.println(F("  Nothing moved while the power was off."));
        } else {
          Serial.println(F("  Something back-drove the actuator, or the cam does"));
          Serial.println(F("  not trip in the same place twice. Either way this"));
          Serial.println(F("  is the error an un-checked saved position would"));
          Serial.println(F("  have inherited silently."));
        }
        g_driftArmed = false;
      }

      // During a calibration the midpoint is not known yet, so the home leg
      // always lands on zero and the origin is shifted afterwards.
      setPosition(g_calibrating ? 0 : retractStopPos());
      g_homed  = true;
      g_target = position();

      Serial.print(F("  reached the near stop -- pos = "));
      Serial.println(position());

      if (g_calibrating) {
        g_state = ST_MEASURE;
        Serial.println(F("  MEASURING -- extending into the far stop."));
        startMotion('e', SPEED_HOMING);
      } else {
        // Homing re-establishes the SAME reference 'c' measured against, so a
        // known travel still applies. Only 'z' invents a new origin.
        g_state = ST_IDLE;
        saveState();
      }
      return;

    case ST_MEASURE:
      g_travel      = position();      // home leg left us at zero
      g_calibrating = false;
      g_state       = ST_IDLE;

      Serial.print(F("  reached the far stop -- travel = "));
      Serial.print(g_travel);
      Serial.println(F(" counts"));

#if ORIGIN_AT_MIDPOINT
      // Now the midpoint is known, so shift the whole coordinate system onto
      // it. The near stop sits at 0 right now and needs to land on
      // retractStopPos(), so that offset is exactly what everything moves by.
      // Deriving it from retractStopPos() rather than halving travel again is
      // what keeps the two agreeing when travel is an odd number of counts.
      setPosition(position() + retractStopPos());
      Serial.print(F("  origin moved to the midpoint -- pos = "));
      Serial.println(position());
#endif
      g_target = position();

      Serial.print(F("  soft limits now "));
      Serial.print(softMin());
      Serial.print(F(" .. "));
      Serial.println(softMax());

      if (g_travel < 10) {
        Serial.println(F("  That is implausibly short. Either it never left the"));
        Serial.println(F("  near stop, or the sensor is not producing pulses."));
      }
      saveState();
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
//  Persistence
// ---------------------------------------------------------------------------
//  Saved so the next home can CHECK it, not so homing can be skipped. See the
//  note in Config.h.

#if USE_EEPROM
struct SavedState {
  uint16_t magic;
  long     position;
  long     travel;
  float    degPerCount;
  float    degAtRetract;
  uint16_t sum;
};

const uint16_t SAVE_MAGIC = 0xAC71;

uint16_t saveSum(const SavedState& s) {
  const uint8_t* p = (const uint8_t*)&s;
  uint16_t sum = 0;
  for (size_t i = 0; i < sizeof(SavedState) - sizeof(uint16_t); i++) sum += p[i];
  return sum;
}

void saveState() {
  SavedState s;
  s.magic        = SAVE_MAGIC;
  s.position     = position();
  s.travel       = g_travel;
  s.degPerCount  = g_degPerCount;
  s.degAtRetract = g_degAtRetract;
  s.sum          = saveSum(s);

  // update() rather than write(), so an unchanged byte costs no wear.
  const uint8_t* p = (const uint8_t*)&s;
  for (size_t i = 0; i < sizeof(s); i++) EEPROM.update(EEPROM_BASE_ADDR + i, p[i]);
}

bool loadState(SavedState* out) {
  SavedState s;
  uint8_t* p = (uint8_t*)&s;
  for (size_t i = 0; i < sizeof(s); i++) p[i] = EEPROM.read(EEPROM_BASE_ADDR + i);

  if (s.magic != SAVE_MAGIC) return false;
  if (s.sum != saveSum(s))   return false;
  *out = s;
  return true;
}
#else
void saveState() {}
#endif

// ---------------------------------------------------------------------------
//  LCD  --  HD44780 behind a PCF8574, 4-bit, straight onto Wire
// ---------------------------------------------------------------------------

#if USE_LCD
// The near-universal backpack bit map.
const uint8_t LCD_RS = 0x01;   // P0
const uint8_t LCD_EN = 0x04;   // P2
const uint8_t LCD_BL = 0x08;   // P3, backlight -- held on

bool g_lcdOk = false;          // false if nothing acknowledged at the address

void lcdExpander(uint8_t bits) {
  Wire.beginTransmission(LCD_I2C_ADDR);
  Wire.write(bits | LCD_BL);
  Wire.endTransmission();
}

void lcdPulse(uint8_t bits) {
  lcdExpander(bits | LCD_EN);
  delayMicroseconds(2);
  lcdExpander(bits & ~LCD_EN);
  delayMicroseconds(50);
}

void lcdSend(uint8_t value, uint8_t mode) {
  lcdPulse((value & 0xF0) | mode);
  lcdPulse(((value << 4) & 0xF0) | mode);
}

void lcdCommand(uint8_t c) { lcdSend(c, 0); }

void lcdBegin() {
  Wire.begin();
  Wire.setClock(400000);       // ~2 ms for a full refresh instead of ~7

  Wire.beginTransmission(LCD_I2C_ADDR);
  g_lcdOk = (Wire.endTransmission() == 0);
  if (!g_lcdOk) return;        // nothing there; every call below turns into a no-op

  delay(50);
  lcdExpander(0);
  delay(20);

  // The HD44780 4-bit wake-up sequence, which is not optional and not pretty.
  lcdPulse(0x30); delay(5);
  lcdPulse(0x30); delayMicroseconds(150);
  lcdPulse(0x30); delayMicroseconds(150);
  lcdPulse(0x20); delayMicroseconds(150);

  lcdCommand(0x28);   // 4-bit, 2 lines, 5x8
  lcdCommand(0x0C);   // on, no cursor
  lcdCommand(0x06);   // entry mode: advance right
  lcdCommand(0x01);   // clear
  delay(2);
}

// Writes exactly LCD_COLS characters, so a shorter line erases what was under
// it without a clear() and the flicker that brings.
void lcdLine(uint8_t row, const char* s) {
  if (!g_lcdOk) return;
  lcdCommand(0x80 | (row ? 0x40 : 0x00));
  bool ended = false;
  for (uint8_t i = 0; i < LCD_COLS; i++) {
    if (s[i] == '\0') ended = true;
    lcdSend(ended ? ' ' : (uint8_t)s[i], LCD_RS);
  }
}

unsigned long g_lcdNextMs = 0;

void lcdUpdate(unsigned long now) {
  if (!g_lcdOk || now < g_lcdNextMs) return;
  g_lcdNextMs = now + LCD_REFRESH_MS;

  char l1[LCD_COLS + 1];
  char l2[LCD_COLS + 1];
  char deg[8];

  if (haveAngle()) dtostrf(degreesNow(), 6, 1, deg);
  else             strcpy(deg, "     ?");

  snprintf(l1, sizeof(l1), "%6ldct %6s", position(), deg);

  switch (g_state) {
    case ST_SEEK:
      snprintf(l2, sizeof(l2), "SEEK  ->%7ld", g_target);
      break;
    case ST_JOG:
      snprintf(l2, sizeof(l2), "JOG %s", g_dir == 'e' ? "EXTEND" : "RETRACT");
      break;
    case ST_HOME:
      strcpy(l2, "HOMING...");
      break;
    case ST_MEASURE:
      strcpy(l2, "MEASURING...");
      break;
    case ST_FAULT:
      snprintf(l2, sizeof(l2), "FAULT %s",
               g_fault == FAULT_STALL   ? "STALL" :
               g_fault == FAULT_TIMEOUT ? "TIMEOUT" : "NOT HOMED");
      break;
    default:
      strcpy(l2, g_homed ? "IDLE  homed" : "IDLE  NOT homed");
      break;
  }

  lcdLine(0, l1);
  lcdLine(1, l2);
}
#else
void lcdBegin() {}
void lcdUpdate(unsigned long) {}
#endif

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

  Serial.print(F("  deg="));
  if (haveAngle()) Serial.print(degreesNow(), 2);
  else             Serial.print(F("? (run a/b)"));

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
  Serial.println(F("  a <tenths>   angle calibration point A, e.g. 'a 1652'"));
  Serial.println(F("  b <tenths>   point B -- solves and saves the fit"));
  Serial.println(F("  h            home: retract into the near stop"));
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

// Two-point angle calibration, the same linear fit host/geometry.py uses. Sight
// the boom, type the heading, drive somewhere else, sight it again.
void cmdAnglePoint(char which, const char* arg) {
  long millideg;
  if (!parseLong(arg, &millideg)) {
    Serial.print(which);
    Serial.println(F(" needs a heading in tenths of a degree: 'a 1652' = 165.2"));
    return;
  }
  if (!g_homed) {
    Serial.println(F("  home first -- the fit is anchored to the retract stop"));
    return;
  }
  if (g_state != ST_IDLE) {
    Serial.println(F("  not while it is moving"));
    return;
  }

  const float deg = millideg / 10.0f;
  const long  pos = position();

  if (which == 'a') {
    g_posA = pos;
    g_degA = deg;
    g_havePointA = true;
    Serial.print(F("  point A: "));
    Serial.print(deg, 1);
    Serial.print(F(" deg at "));
    Serial.println(pos);
    Serial.println(F("  Now drive well along the stroke and sight again with 'b'."));
    return;
  }

  if (!g_havePointA) {
    Serial.println(F("  no point A yet -- 'a <heading>' first"));
    return;
  }
  if (pos == g_posA) {
    Serial.println(F("  same position as point A -- move first, or the slope"));
    Serial.println(F("  is a division by zero"));
    return;
  }

  g_degPerCount  = (deg - g_degA) / (float)(pos - g_posA);
  g_degAtRetract = g_degA + (retractStopPos() - g_posA) * g_degPerCount;
  g_havePointA   = false;

  Serial.print(F("  point B: "));
  Serial.print(deg, 1);
  Serial.print(F(" deg at "));
  Serial.println(pos);
  Serial.print(F("  fit: "));
  Serial.print(g_degPerCount, 4);
  Serial.print(F(" deg/count, "));
  Serial.print(g_degAtRetract, 2);
  Serial.println(F(" deg at the retract stop"));
  Serial.print(F("  one count = "));
  Serial.print(fabs(g_degPerCount), 4);
  Serial.println(F(" deg -- that is the hard floor on pointing"));

  if (fabs(g_degPerCount) < 1e-5) {
    Serial.println(F("  That slope is implausibly flat. Check the two headings"));
    Serial.println(F("  were actually different."));
  }
  saveState();
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

    case 'a': cmdAnglePoint('a', arg); break;
    case 'b': cmdAnglePoint('b', arg); break;

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

  lcdBegin();
#if USE_LCD
  Serial.print(F("LCD: "));
  Serial.println(g_lcdOk ? F("found") : F("nothing at that address -- running without it"));
#endif

#if USE_EEPROM
  SavedState saved;
  if (loadState(&saved)) {
    g_travel       = saved.travel;
    g_degPerCount  = saved.degPerCount;
    g_degAtRetract = saved.degAtRetract;
    setPosition(saved.position);
    g_driftArmed   = true;

    Serial.print(F("Restored: pos="));   Serial.print(saved.position);
    Serial.print(F(" travel="));         Serial.print(saved.travel);
    Serial.println();
    Serial.println(F("This is NOT trusted -- homed is still false and absolute"));
    Serial.println(F("moves still refuse. It is loaded so the next 'h' can"));
    Serial.println(F("measure how far things drifted while the power was off."));
  } else {
    Serial.println(F("No saved state. Nothing is homed."));
  }
#else
  Serial.println(F("Nothing is homed."));
#endif
  Serial.println(F("Relative moves (+3, -1) work anyway; absolute needs h or c."));
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
  lcdUpdate(millis());
}
