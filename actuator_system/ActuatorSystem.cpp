#include "ActuatorSystem.h"
#include "ActuatorSim.h"

#if USE_EEPROM
  #include <EEPROM.h>
#endif

static const long LIMIT_UNKNOWN = 0x7FFFFFFFL;

ActuatorSystem* ActuatorSystem::instance_ = nullptr;

// ===========================================================================
//  Motor driver layer
//  The rest of the class only ever calls driverDrive() / driverOff(), so
//  swapping hardware means editing ActuatorConfig.h and nothing else.
// ===========================================================================

void ActuatorSystem::driverBegin() {
#if MOTOR_DRIVER == DRV_PWM_DIR
  pinMode(PIN_MOTOR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_DIR, OUTPUT);
  analogWrite(PIN_MOTOR_PWM, 0);
  digitalWrite(PIN_MOTOR_DIR, LOW);

#elif MOTOR_DRIVER == DRV_DUAL_PWM
  pinMode(PIN_MOTOR_RPWM, OUTPUT);
  pinMode(PIN_MOTOR_LPWM, OUTPUT);
  pinMode(PIN_MOTOR_EN,   OUTPUT);
  analogWrite(PIN_MOTOR_RPWM, 0);
  analogWrite(PIN_MOTOR_LPWM, 0);
  digitalWrite(PIN_MOTOR_EN, HIGH);      // bridge enabled, both halves at 0

#elif MOTOR_DRIVER == DRV_RELAY
  // Drive the pins to the inactive level *before* making them outputs, so a
  // reset cannot pulse the relays on for a few microseconds.
  digitalWrite(PIN_RELAY_EXTEND,  RELAY_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(PIN_RELAY_RETRACT, RELAY_ACTIVE_LOW ? HIGH : LOW);
  pinMode(PIN_RELAY_EXTEND,  OUTPUT);
  pinMode(PIN_RELAY_RETRACT, OUTPUT);
#endif
}

void ActuatorSystem::driverOff() {
#if SIMULATE_ACTUATOR
  Sim.onDrive(0, 0);
#endif

#if MOTOR_DRIVER == DRV_PWM_DIR
  analogWrite(PIN_MOTOR_PWM, 0);

#elif MOTOR_DRIVER == DRV_DUAL_PWM
  analogWrite(PIN_MOTOR_RPWM, 0);
  analogWrite(PIN_MOTOR_LPWM, 0);

#elif MOTOR_DRIVER == DRV_RELAY
  const uint8_t off = RELAY_ACTIVE_LOW ? HIGH : LOW;
  digitalWrite(PIN_RELAY_EXTEND,  off);
  digitalWrite(PIN_RELAY_RETRACT, off);
#endif
}

void ActuatorSystem::driverDrive(Dir d, uint8_t duty) {
  if (d == STOPPED || duty == 0) { driverOff(); return; }

#if SIMULATE_ACTUATOR
  // The real pins are still driven below, so you can put an LED or a scope on
  // them and watch the direction logic and the ramp with no driver attached.
  Sim.onDrive((int8_t)d, duty);
#endif

#if MOTOR_DRIVER == DRV_PWM_DIR
  digitalWrite(PIN_MOTOR_DIR, d == EXTEND ? HIGH : LOW);
  analogWrite(PIN_MOTOR_PWM, duty);

#elif MOTOR_DRIVER == DRV_DUAL_PWM
  // Never drive both halves at once -- zero the opposite side first.
  if (d == EXTEND) {
    analogWrite(PIN_MOTOR_LPWM, 0);
    analogWrite(PIN_MOTOR_RPWM, duty);
  } else {
    analogWrite(PIN_MOTOR_RPWM, 0);
    analogWrite(PIN_MOTOR_LPWM, duty);
  }

#elif MOTOR_DRIVER == DRV_RELAY
  // No speed control here -- duty is ignored, the actuator runs at supply
  // voltage. REVERSAL_DELAY_MS is what keeps us from cross-energising.
  const uint8_t on  = RELAY_ACTIVE_LOW ? LOW  : HIGH;
  const uint8_t off = RELAY_ACTIVE_LOW ? HIGH : LOW;
  if (d == EXTEND) {
    digitalWrite(PIN_RELAY_RETRACT, off);
    digitalWrite(PIN_RELAY_EXTEND,  on);
  } else {
    digitalWrite(PIN_RELAY_EXTEND,  off);
    digitalWrite(PIN_RELAY_RETRACT, on);
  }
#endif
}

// ===========================================================================
//  Sensor / interrupt
// ===========================================================================

void ActuatorSystem::isrTrampoline() {
  if (instance_) instance_->onPulse();
}

void ActuatorSystem::injectPulse() {
  noInterrupts();          // onPulse() normally runs with interrupts off
  onPulse();
  interrupts();
}

void ActuatorSystem::onPulse() {
  const unsigned long now = micros();
  const unsigned long gap = now - lastPulseMicros_;
  if (gap < REED_DEBOUNCE_US) return;          // contact bounce, not a count

  lastPulseGapUs_  = gap;
  lastPulseMicros_ = now;
  pulses_++;
  counts_ += countSign_;                        // signed by commanded direction
}

// ===========================================================================
//  Lifecycle
// ===========================================================================

void ActuatorSystem::begin() {
  instance_ = this;

  driverBegin();
  driverOff();

  pinMode(PIN_REED, INPUT_PULLUP);
  lastPulseMicros_ = micros();
  attachInterrupt(digitalPinToInterrupt(PIN_REED), isrTrampoline, FALLING);

  stopTimeMs_ = millis();

  loadPosition();      // restores counts_/travel/homed if EEPROM looks valid
}

// ===========================================================================
//  Motion primitives
// ===========================================================================

void ActuatorSystem::haltMotor() {
  driverOff();
  dir_        = STOPPED;
  pwmOut_     = 0;
  stopTimeMs_ = millis();
}

// haltMotor() only kills the output. It does NOT cancel a reversal that is
// already armed, so on its own it is not enough to leave the system stopped --
// update() would happily start the pending direction a moment later.
//
// Every path that ends motion must therefore go through disarm() (or
// enterFault(), which wraps it). The invariant being protected:
//
//     state_ == IDLE || state_ == FAULTED   =>   dir_ == STOPPED
//                                           and  pendingDir_ == STOPPED
//
// Break that and the bridge stays energised while isBusy() reports false --
// on real hardware, that is a silent run into a mechanical stop.
void ActuatorSystem::disarm() {
  haltMotor();
  pendingDir_ = STOPPED;
}

void ActuatorSystem::enterFault(Fault f) {
  disarm();
  calPhase_ = 0;
  fault_    = f;
  state_    = FAULTED;
  savePosition();
}

// Note: at the default SPEED_MAX of 255 the upper half of the constrain()
// below is dead code, and -Wtype-limits says so. It stays because lowering
// SPEED_MAX in ActuatorConfig.h is a supported thing to do (e.g. running a
// 12 V actuator off a higher supply), and that is exactly when the ceiling
// starts mattering. Expect the warning; do not "fix" it by dropping the clamp.
void ActuatorSystem::startMotion(Dir d, uint8_t duty) {
  noInterrupts();
  countSign_       = (int8_t)d;
  lastPulseMicros_ = micros();      // restart the stall timer
  interrupts();

  dir_           = d;
  pwmTarget_     = constrain(duty, SPEED_MIN, SPEED_MAX);
  pwmOut_        = 0;
  hitHardLimit_  = false;
  motionStartMs_ = millis();
}

void ActuatorSystem::commandMotion(Dir d, uint8_t duty) {
  if (d == STOPPED) { stop(); return; }

  if (dir_ == d) {                              // already going that way
    pwmTarget_ = constrain(duty, SPEED_MIN, SPEED_MAX);
    return;
  }
  if (dir_ != STOPPED) haltMotor();             // stamps stopTimeMs_

  pendingDir_ = d;                              // update() starts it once the
  pendingPwm_ = duty;                           // reversal dead time expires
}

void ActuatorSystem::applyRamp(unsigned long now) {
  uint8_t out;
  const unsigned long t = now - motionStartMs_;

  if (t >= RAMP_MS || pwmTarget_ <= SPEED_MIN) {
    out = pwmTarget_;
  } else {
    out = SPEED_MIN + (uint8_t)(((uint32_t)(pwmTarget_ - SPEED_MIN) * t) / RAMP_MS);
  }

  if (out != pwmOut_) {
    driverDrive(dir_, out);
    pwmOut_ = out;
  }
}

unsigned long ActuatorSystem::msSinceLastPulse() const {
  noInterrupts();
  const unsigned long last = lastPulseMicros_;
  interrupts();
  return (unsigned long)(micros() - last) / 1000UL;
}

// ===========================================================================
//  Commands
// ===========================================================================

void ActuatorSystem::jog(Dir d, uint8_t duty) {
  if (state_ == FAULTED) return;
  state_ = JOGGING;
  commandMotion(d, duty);
}

bool ActuatorSystem::moveTo(long counts, uint8_t duty) {
  if (state_ == FAULTED) return false;

  // Rejecting the command is not enough: if the actuator was jogging when this
  // arrived, refusing it while leaving the motor on is worse than obeying it.
  if (!homed_) { enterFault(FAULT_NOT_HOMED); return false; }

  // Clamp into the safe band so a typo cannot drive us into a hard stop.
  long lo = softMin();
  long hi = softMax();
  if (counts < lo) counts = lo;
  if (hi != LIMIT_UNKNOWN && counts > hi) counts = hi;

  targetCounts_ = counts;
  const long err = targetCounts_ - position();

  if (err > -DEADBAND_COUNTS && err < DEADBAND_COUNTS) {   // already there
    // Unconditional: dir_ can be STOPPED while a reversal is still armed, and
    // that reversal has to die here too.
    disarm();
    state_ = IDLE;
    return true;
  }

  state_ = SEEKING;
  commandMotion(err > 0 ? EXTEND : RETRACT, duty);
  return true;
}

bool ActuatorSystem::moveBy(long delta, uint8_t duty) {
  return moveTo(position() + delta, duty);
}

void ActuatorSystem::stop() {
  disarm();
  calPhase_   = 0;
  if (state_ != FAULTED) state_ = IDLE;
  savePosition();
}

void ActuatorSystem::home() {
  if (state_ == FAULTED) return;
  calPhase_ = 0;
  state_    = HOMING;
  commandMotion(RETRACT, SPEED_HOMING);
}

void ActuatorSystem::calibrate() {
  if (state_ == FAULTED) return;
  calPhase_ = 1;                    // phase 1: retract to the near hard stop
  state_    = CALIBRATING;
  commandMotion(RETRACT, SPEED_HOMING);
}

void ActuatorSystem::zeroHere() {
  noInterrupts();
  counts_ = 0;
  interrupts();
  homed_        = true;
  targetCounts_ = 0;
  savePosition();
}

void ActuatorSystem::clearFault() {
  disarm();
  calPhase_   = 0;
  fault_      = FAULT_NONE;
  state_      = IDLE;
}

// ===========================================================================
//  The loop
// ===========================================================================

void ActuatorSystem::finishMove() {
  haltMotor();
  state_ = IDLE;
  savePosition();
}

void ActuatorSystem::onStall() {
  // Pulses stopped while we were commanding motion. Either an internal cam
  // limit switch opened (normal at the extremes) or something is jammed.
  const Dir stalledDir = dir_;
  disarm();
  hitHardLimit_ = true;

  // A hard stop is the one absolutely known position on this actuator, so
  // take the reference whenever we reach one -- it cancels accumulated
  // miscounts even when the stall was not what we wanted.
  if (stalledDir == RETRACT) {
    noInterrupts();
    counts_ = 0;
    interrupts();
    homed_ = true;
  } else if (stalledDir == EXTEND && homed_) {
    travelCounts_ = position();
  }

  switch (state_) {
    case HOMING:
      state_ = IDLE;
      break;

    case CALIBRATING:
      if (calPhase_ == 1) {
        calPhase_ = 2;                        // now find the far hard stop
        state_    = CALIBRATING;
        commandMotion(EXTEND, SPEED_HOMING);
        return;                               // stay busy, do not save yet
      }
      calPhase_ = 0;
      state_    = IDLE;
      break;

    case JOGGING:
      state_ = IDLE;                          // reaching a limit is expected
      break;

    case SEEKING:
      // We expected to reach the target before any hard stop -- something is
      // wrong (bad travel figure, jam, or lost counts). Stop and complain.
      enterFault(FAULT_STALL);
      return;                                   // enterFault() already saved

    default:
      // Do not fall out of FAULTED here. Stalling into a hard stop while
      // already faulted is how a rejected command ends, and silently
      // returning to IDLE would leave state_ and fault_ disagreeing.
      if (state_ != FAULTED) state_ = IDLE;
      break;
  }

  savePosition();
}

void ActuatorSystem::update() {
  const unsigned long now = millis();

  // Waiting out the reversal dead time before energising the other direction.
  if (pendingDir_ != STOPPED) {
    if (now - stopTimeMs_ < REVERSAL_DELAY_MS) return;
    startMotion(pendingDir_, pendingPwm_);
    pendingDir_ = STOPPED;
  }

  if (dir_ == STOPPED) return;                // nothing is moving

  applyRamp(now);

  if (now - motionStartMs_ > MOVE_TIMEOUT_MS) {
    enterFault(FAULT_TIMEOUT);
    return;
  }

  if (now - motionStartMs_ > START_GRACE_MS && msSinceLastPulse() > STALL_TIMEOUT_MS) {
    onStall();
    return;
  }

  const long pos = position();

  switch (state_) {
    case SEEKING: {
      const long err = targetCounts_ - pos;

      // Stop on overshoot too: if we are extending, any err <= deadband means
      // we are at or past the target.
      if ((dir_ == EXTEND  && err <=  DEADBAND_COUNTS) ||
          (dir_ == RETRACT && err >= -DEADBAND_COUNTS)) {
        finishMove();
        break;
      }

      const long absErr = err >= 0 ? err : -err;
      if (absErr <= SLOW_ZONE_COUNTS && pwmTarget_ > SPEED_APPROACH) {
        pwmTarget_ = SPEED_APPROACH;
      }
      break;
    }

    case JOGGING:
      if (homed_) {                            // respect the soft limits
        const long hi = softMax();
        if (dir_ == EXTEND  && hi != LIMIT_UNKNOWN && pos >= hi) stop();
        else if (dir_ == RETRACT && pos <= softMin())            stop();
      }
      break;

    case HOMING:
    case CALIBRATING:
      // Both end when onStall() fires -- driving into the hard stop is the
      // whole point, so there is nothing to check here.
      break;

    case IDLE:
    case FAULTED:
      // Unreachable if the disarm() invariant holds. Reaching it means
      // something energised the motor without owning it, and nothing in this
      // switch is watching the position -- so cut the motor rather than let
      // it run unmanaged to a hard stop.
      disarm();
      break;
  }
}

// ===========================================================================
//  Accessors
// ===========================================================================

long ActuatorSystem::position() const {
  noInterrupts();
  const long c = counts_;
  interrupts();
  return c;
}

unsigned long ActuatorSystem::pulseTotal() const {
  noInterrupts();
  const unsigned long p = pulses_;
  interrupts();
  return p;
}

float ActuatorSystem::pulseHz() const {
  noInterrupts();
  const unsigned long gap = lastPulseGapUs_;
  interrupts();
  if (gap == 0) return 0.0f;
  if (msSinceLastPulse() > STALL_TIMEOUT_MS) return 0.0f;   // stale, not moving
  return 1000000.0f / (float)gap;
}

long ActuatorSystem::softMin() const {
  return SOFT_LIMIT_MARGIN;
}

long ActuatorSystem::softMax() const {
  if (travelCounts_ <= 0) return LIMIT_UNKNOWN;
  return travelCounts_ - SOFT_LIMIT_MARGIN;
}

const __FlashStringHelper* ActuatorSystem::stateName() const {
  switch (state_) {
    case IDLE:        return F("IDLE");
    case JOGGING:     return F("JOGGING");
    case SEEKING:     return F("SEEKING");
    case HOMING:      return F("HOMING");
    case CALIBRATING: return F("CALIBRATING");
    case FAULTED:     return F("FAULTED");
  }
  return F("?");
}

const __FlashStringHelper* ActuatorSystem::faultName() const {
  switch (fault_) {
    case FAULT_NONE:      return F("none");
    case FAULT_STALL:     return F("STALL (hard stop or jam mid-move)");
    case FAULT_TIMEOUT:   return F("TIMEOUT (move took too long)");
    case FAULT_NOT_HOMED: return F("NOT_HOMED (run 'h' first)");
  }
  return F("?");
}

// ===========================================================================
//  EEPROM persistence
// ===========================================================================

#if USE_EEPROM

namespace {
  struct PersistBlob {
    uint32_t magic;
    long     counts;
    long     travel;
    uint8_t  homed;
    uint16_t sum;
  };

  const uint32_t PERSIST_MAGIC = 0xAC7A0101UL;

  uint16_t blobSum(const PersistBlob& b) {
    uint16_t s = 0;
    const uint8_t* p = (const uint8_t*)&b;
    // everything except the trailing sum field itself
    for (size_t i = 0; i < sizeof(PersistBlob) - sizeof(uint16_t); i++) {
      s = (uint16_t)(s * 31 + p[i]);
    }
    return s;
  }
}

void ActuatorSystem::savePosition() {
  PersistBlob b;
  b.magic  = PERSIST_MAGIC;
  b.counts = position();
  b.travel = travelCounts_;
  b.homed  = homed_ ? 1 : 0;
  b.sum    = blobSum(b);
  EEPROM.put(EEPROM_BASE_ADDR, b);     // put() skips bytes that already match
}

bool ActuatorSystem::loadPosition() {
  PersistBlob b;
  EEPROM.get(EEPROM_BASE_ADDR, b);
  if (b.magic != PERSIST_MAGIC || b.sum != blobSum(b)) return false;

  noInterrupts();
  counts_ = b.counts;
  interrupts();
  travelCounts_ = b.travel;
  homed_        = b.homed != 0;
  targetCounts_ = b.counts;
  return true;
}

void ActuatorSystem::forgetPosition() {
  PersistBlob b;
  memset(&b, 0, sizeof(b));
  EEPROM.put(EEPROM_BASE_ADDR, b);
  homed_        = false;
  travelCounts_ = 0;
}

#else   // ---- no EEPROM: everything is volatile, home after every power-up --

void ActuatorSystem::savePosition() {}
bool ActuatorSystem::loadPosition() { return false; }
void ActuatorSystem::forgetPosition() { homed_ = false; travelCounts_ = 0; }

#endif
