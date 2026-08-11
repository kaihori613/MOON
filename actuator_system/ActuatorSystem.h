/*
  ActuatorSystem.h
  ----------------
  Closed-loop control for a linear actuator whose only feedback is a reed
  switch pulsing once per magnet pass (the HTS TVRO actuator).

  The important consequence of that sensor: it counts, but it does not tell
  you which way you moved. Position is therefore tracked as
  "pulses, signed by the direction we last commanded", which is correct as
  long as nothing back-drives the actuator while the motor is off.

  The other consequence: the cam limit switches at each end of travel cut
  motor current internally, they are not wired back to the Arduino. So the
  way we detect "we hit the end" is that pulses stop arriving while we are
  still commanding motion. That is the same signature as a jam, which is why
  every stall stops the motor immediately.

  Usage:
      ActuatorSystem act;
      void setup() { act.begin(); }
      void loop()  { act.update(); }        // non-blocking, call often

      act.home();              // find the retract hard stop, call it zero
      act.moveTo(1200);        // absolute, in reed counts
      while (act.isBusy()) act.update();
*/

#pragma once
#include <Arduino.h>
#include "ActuatorConfig.h"

class ActuatorSystem {
public:
  enum Dir : int8_t {
    RETRACT = -1,
    STOPPED =  0,
    EXTEND  = +1
  };

  enum State : uint8_t {
    IDLE,          // motor off, nothing pending
    JOGGING,       // running open-loop until told to stop or until a hard limit
    SEEKING,       // closed-loop move toward targetCounts_
    HOMING,        // retracting into the hard stop to establish zero
    CALIBRATING,   // homing, then extending into the far stop to learn travel
    FAULTED        // motor off, needs clearFault()
  };

  enum Fault : uint8_t {
    FAULT_NONE,
    FAULT_STALL,       // pulses stopped mid-move (jam, or an unexpected limit)
    FAULT_TIMEOUT,     // move ran longer than MOVE_TIMEOUT_MS
    FAULT_NOT_HOMED    // absolute move requested before homing
  };

  void begin();
  void update();                                  // call from loop(), often

  // --- Commands ------------------------------------------------------------
  void jog(Dir d, uint8_t pwm = SPEED_MAX);       // open loop until stop/limit
  bool moveTo(long counts, uint8_t pwm = SPEED_MAX);  // absolute, needs homing
  bool moveBy(long delta, uint8_t pwm = SPEED_MAX);   // relative, needs homing
  void stop();                                    // controlled stop, no fault
  void home();                                    // retract to hard stop -> 0
  void calibrate();                               // home, then learn full travel
  void zeroHere();                                // declare current spot as 0
  void clearFault();

  // --- State ---------------------------------------------------------------
  long  position() const;                         // signed reed counts
  long  target() const        { return targetCounts_; }
  long  travelCounts() const  { return travelCounts_; }   // 0 = not calibrated
  bool  isHomed() const       { return homed_; }
  bool  isBusy() const        { return state_ != IDLE && state_ != FAULTED; }
  State state() const         { return state_; }
  Fault fault() const         { return fault_; }
  Dir   direction() const     { return dir_; }
  uint8_t pwm() const         { return pwmOut_; }
  bool  hitHardLimit() const  { return hitHardLimit_; }   // last stop was a limit
  unsigned long pulseTotal() const;               // pulses since boot, unsigned
  float pulseHz() const;                          // instantaneous, from last gap

  long softMin() const;
  long softMax() const;                           // == LONG_MAX if uncalibrated

  const __FlashStringHelper* stateName() const;
  const __FlashStringHelper* faultName() const;

  // --- Persistence ---------------------------------------------------------
  void savePosition();
  bool loadPosition();                            // true if valid data found
  void forgetPosition();

  // Simulator hook only -- feeds a pulse in as though the ISR had fired.
  // Real sensor pulses never come through here.
  void injectPulse();

private:
  // Interrupt plumbing. Only one actuator is supported; the ISR needs a
  // plain function, so we trampoline through a static instance pointer.
  static ActuatorSystem* instance_;
  static void isrTrampoline();
  void onPulse();

  // Motor driver abstraction (implementation picked by MOTOR_DRIVER).
  void driverBegin();
  void driverDrive(Dir d, uint8_t pwm);
  void driverOff();

  void commandMotion(Dir d, uint8_t pwm);
  void startMotion(Dir d, uint8_t pwm);
  void haltMotor();
  void disarm();                    // cut the motor AND drop any armed reversal
  void enterFault(Fault f);         // the only way into FAULTED
  void applyRamp(unsigned long now);
  void onStall();
  void finishMove();
  unsigned long msSinceLastPulse() const;

  // --- Sensor state (touched by the ISR) -----------------------------------
  volatile long          counts_            = 0;
  volatile unsigned long pulses_            = 0;
  volatile unsigned long lastPulseMicros_   = 0;
  volatile unsigned long lastPulseGapUs_    = 0;
  volatile int8_t        countSign_         = +1;

  // --- Motion state --------------------------------------------------------
  State  state_       = IDLE;
  Fault  fault_       = FAULT_NONE;
  Dir    dir_         = STOPPED;
  Dir    pendingDir_  = STOPPED;      // waiting out REVERSAL_DELAY_MS
  uint8_t pendingPwm_ = SPEED_MAX;
  uint8_t pwmTarget_  = SPEED_MAX;    // what the ramp is climbing toward
  uint8_t pwmOut_     = 0;            // what is actually on the pin

  long   targetCounts_ = 0;
  long   travelCounts_ = 0;           // learned by calibrate(); 0 = unknown
  bool   homed_        = false;
  bool   hitHardLimit_ = false;
  uint8_t calPhase_    = 0;           // 0 none, 1 retracting, 2 extending

  unsigned long motionStartMs_ = 0;
  unsigned long stopTimeMs_    = 0;
};
