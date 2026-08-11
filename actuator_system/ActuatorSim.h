/*
  ActuatorSim.h
  -------------
  A virtual HTS actuator, so the control system can be tested with no motor,
  no driver and no power unit -- just the Arduino.

  It hooks the motor driver layer: whenever ActuatorSystem energises the
  "motor", the simulator starts producing reed pulses at a rate proportional
  to the commanded duty, advancing a virtual carriage. When that carriage
  reaches either end of its travel the pulses stop -- which is exactly what
  the real actuator does, because its cam limit switches cut motor current
  internally. The control code cannot tell the difference, and that is the
  whole point.

  WHAT THIS PROVES
    - the interrupt, debounce and signed counting all work
    - homing finds zero and calibration measures travel
    - stall detection distinguishes "hit the end" from "still moving"
    - soft limits hold, and moves land inside the deadband
    - position survives a power cycle through EEPROM

  WHAT IT CANNOT PROVE
    - anything physical: current draw, real travel, real pulse timing, EMI,
      whether your driver wiring is right, whether the reed switch works.
    Simulation passing is necessary, not sufficient. The bench still happens.
*/

#pragma once
#include <Arduino.h>
#include "ActuatorConfig.h"

#if SIMULATE_ACTUATOR

class ActuatorSystem;

class ActuatorSim {
public:
  void begin(ActuatorSystem* act);
  void update();                              // call from loop()

  // Called by the driver layer of ActuatorSystem. Not for user code.
  void onDrive(int8_t dir, uint8_t duty);

  // --- Bench controls ------------------------------------------------------
  void setTravel(long counts);
  void setJammed(bool j);                     // motor on, carriage does not move
  void injectNoise(uint8_t pulses);           // fake EMI on the sensor line
  void teleport(long counts);                 // move the carriage with no pulses
                                              // (models it being shifted while
                                              //  the Arduino was powered off)

  long   travel() const       { return travel_; }
  long   truePosition() const { return truePos_; }
  bool   jammed() const       { return jammed_; }
  bool   atRetractStop() const { return truePos_ <= 0; }
  bool   atExtendStop() const  { return truePos_ >= travel_; }
  int8_t dir() const          { return dir_; }

private:
  void emitPulse();
  void scheduleFirstPulse();
  unsigned long pulseIntervalUs() const;

  ActuatorSystem* act_ = nullptr;

  long    truePos_ = 0;                       // ground truth, in counts
  long    travel_  = SIM_TRAVEL_COUNTS;
  int8_t  dir_     = 0;
  uint8_t duty_    = 0;
  bool    jammed_  = false;

  unsigned long driveStartMs_ = 0;
  unsigned long nextPulseUs_  = 0;

  uint8_t       noiseLeft_    = 0;
  unsigned long nextNoiseUs_  = 0;
};

extern ActuatorSim Sim;

#endif  // SIMULATE_ACTUATOR
