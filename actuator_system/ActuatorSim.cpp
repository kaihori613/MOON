#include "ActuatorSim.h"

#if SIMULATE_ACTUATOR

#include "ActuatorSystem.h"

ActuatorSim Sim;

void ActuatorSim::begin(ActuatorSystem* act) {
  act_ = act;
#if SIM_LOOPBACK_PIN != 255
  // Idle high: the reed input uses INPUT_PULLUP and triggers on FALLING, so
  // a fake pulse is a brief pull low. Set the level before the pinMode so the
  // pin never glitches low on startup and counts a phantom pulse.
  digitalWrite(SIM_LOOPBACK_PIN, HIGH);
  pinMode(SIM_LOOPBACK_PIN, OUTPUT);
  digitalWrite(SIM_LOOPBACK_PIN, HIGH);
#endif
}

unsigned long ActuatorSim::pulseIntervalUs() const {
  if (duty_ == 0 || SIM_PULSES_PER_SEC == 0) return 0xFFFFFFFFUL;
  // rate = SIM_PULSES_PER_SEC * duty/255
  const unsigned long rateMilliHz =
      (unsigned long)SIM_PULSES_PER_SEC * 1000UL * duty_ / 255UL;
  if (rateMilliHz == 0) return 0xFFFFFFFFUL;
  return 1000000000UL / rateMilliHz;
}

void ActuatorSim::scheduleFirstPulse() {
  nextPulseUs_ = micros() + pulseIntervalUs();
}

void ActuatorSim::onDrive(int8_t d, uint8_t duty) {
  if (d == 0 || duty == 0) {                  // motor off
    dir_  = 0;
    duty_ = 0;
    return;
  }

  if (d != dir_) {                            // a genuinely new move
    dir_          = d;
    duty_         = duty;
    driveStartMs_ = millis();                 // restart the breakaway lag
    scheduleFirstPulse();
    return;
  }

  // Same direction, new duty -- this happens every update() while the soft
  // start ramps. Deliberately do NOT reschedule the next pulse here: doing so
  // would push it forward on every call and the carriage would never move.
  duty_ = duty;
}

void ActuatorSim::update() {
  const unsigned long now = micros();

  // Injected noise fires regardless of whether the motor is running, because
  // that is how real interference behaves.
  if (noiseLeft_ && (long)(now - nextNoiseUs_) >= 0) {
    emitPulse();
    noiseLeft_--;
    nextNoiseUs_ = now + SIM_NOISE_SPACING_US;
  }

  if (dir_ == 0 || duty_ == 0) return;
  if (jammed_) return;                        // energised, but nothing turns
  if (millis() - driveStartMs_ < SIM_STARTUP_LAG_MS) return;

  // At a hard stop the cam switch has cut motor current, so no more pulses.
  // Note we do NOT stop the motor here -- the whole point is that the control
  // code has to work that out for itself from the absence of pulses.
  if (dir_ > 0 && truePos_ >= travel_) return;
  if (dir_ < 0 && truePos_ <= 0)       return;

  if ((long)(now - nextPulseUs_) < 0) return;

  truePos_ += (dir_ > 0) ? 1 : -1;
  emitPulse();
  nextPulseUs_ = now + pulseIntervalUs();
}

void ActuatorSim::emitPulse() {
#if SIM_LOOPBACK_PIN != 255
  digitalWrite(SIM_LOOPBACK_PIN, LOW);
  delayMicroseconds(SIM_PULSE_WIDTH_US);
  digitalWrite(SIM_LOOPBACK_PIN, HIGH);
#else
  if (act_) act_->injectPulse();
#endif
}

// ---------------------------------------------------------------------------

void ActuatorSim::setTravel(long counts) {
  if (counts < 10) counts = 10;
  travel_ = counts;
  if (truePos_ > travel_) truePos_ = travel_;
}

void ActuatorSim::setJammed(bool j) {
  jammed_ = j;
  if (!j) scheduleFirstPulse();               // don't fire a burst on release
}

void ActuatorSim::injectNoise(uint8_t pulses) {
  noiseLeft_   = pulses;
  nextNoiseUs_ = micros();
}

void ActuatorSim::teleport(long counts) {
  if (counts < 0)       counts = 0;
  if (counts > travel_) counts = travel_;
  truePos_ = counts;
}

#endif  // SIMULATE_ACTUATOR
