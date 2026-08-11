/*
  ActuatorConfig.h
  ----------------
  Every pin number and tuning constant for the actuator system lives here.
  This is the only file you should need to edit when the bench wiring changes.
*/

#pragma once
#include <Arduino.h>

// ===========================================================================
//  1. MOTOR DRIVER SELECTION
// ===========================================================================
//  The HTS actuator is just a brushed DC motor, so anything that can reverse
//  polarity at its stall current will work. Pick whichever you have on the
//  bench and set MOTOR_DRIVER to match.
//
//    DRV_PWM_DIR   one PWM pin + one direction pin   (Cytron MD10C, MD13S)
//    DRV_DUAL_PWM  RPWM / LPWM + shared enable       (BTS7960 / IBT-2)
//    DRV_RELAY     two relays, no speed control      (classic TVRO wiring)

#define DRV_PWM_DIR   1
#define DRV_DUAL_PWM  2
#define DRV_RELAY     3

#define MOTOR_DRIVER  DRV_DUAL_PWM      // <-- set this to match your hardware

// ===========================================================================
//  2. PINS
// ===========================================================================

// --- Position sensor (reed switch inside the actuator tube) ---------------
//  Must be an external-interrupt pin: D2 or D3 on Uno/Nano, most pins on
//  Mega/Leonardo/ESP32. Wire one leg to this pin, the other leg to GND --
//  the sketch enables the internal pull-up.
const uint8_t PIN_REED = 2;

// --- Motor driver ---------------------------------------------------------
#if MOTOR_DRIVER == DRV_PWM_DIR
  const uint8_t PIN_MOTOR_PWM = 9;      // PWM output
  const uint8_t PIN_MOTOR_DIR = 8;      // HIGH = extend, LOW = retract

#elif MOTOR_DRIVER == DRV_DUAL_PWM
  const uint8_t PIN_MOTOR_RPWM = 9;     // extend
  const uint8_t PIN_MOTOR_LPWM = 10;    // retract
  const uint8_t PIN_MOTOR_EN   = 8;     // tie R_EN and L_EN together to this

#elif MOTOR_DRIVER == DRV_RELAY
  const uint8_t PIN_RELAY_EXTEND  = 8;
  const uint8_t PIN_RELAY_RETRACT = 7;
  const bool    RELAY_ACTIVE_LOW  = true;   // most cheap relay boards are

#else
  #error "Set MOTOR_DRIVER to one of DRV_PWM_DIR / DRV_DUAL_PWM / DRV_RELAY"
#endif

// ===========================================================================
//  3. SENSOR TUNING
// ===========================================================================

// Minimum time between two pulses that we will believe. Reed contacts bounce
// mechanically; raise this if counts come in pairs, lower it if fast travel
// seems to drop counts. Use the sensor bench sketch to find the real pulse
// rate at full speed, then keep DEBOUNCE well under 1/rate.
const unsigned long REED_DEBOUNCE_US = 3000;

// ===========================================================================
//  4. MOTION TUNING
// ===========================================================================

const uint8_t SPEED_MAX      = 255;   // full speed
const uint8_t SPEED_APPROACH = 120;   // speed once inside SLOW_ZONE_COUNTS
const uint8_t SPEED_MIN      =  90;   // below this the actuator stalls under load
const uint8_t SPEED_HOMING   = 140;   // deliberately slow -- we drive into a hard stop

const uint16_t RAMP_MS = 250;         // soft-start time from SPEED_MIN to target

// Closed-loop tolerances, in reed counts.
//  With a relay driver there is no speed control, so the actuator coasts
//  further past the target -- start with DEADBAND_COUNTS = 3 or 4 there.
const long DEADBAND_COUNTS  = 1;
const long SLOW_ZONE_COUNTS = 8;      // start easing off this far from target

// Keep this many counts away from each mechanical hard stop during normal
// moves. Homing and calibration deliberately ignore it.
const long SOFT_LIMIT_MARGIN = 5;

// ===========================================================================
//  5. SAFETY TIMERS
// ===========================================================================

// If the motor is commanded on but no reed pulse arrives for this long, the
// actuator has either reached an internal cam limit switch (which cuts motor
// current) or it is jammed. Set this to ~3x the slowest expected pulse period.
const uint16_t STALL_TIMEOUT_MS = 700;

// Ignore stall detection for this long after starting -- the motor needs time
// to break away and produce a first pulse.
const uint16_t START_GRACE_MS = 900;

// Dead time inserted between opposite directions, so we never slam the bridge
// (or a relay pair) from full forward to full reverse.
const uint16_t REVERSAL_DELAY_MS = 300;

// Absolute ceiling on any single move. Full travel at full speed should be
// well under this.
const unsigned long MOVE_TIMEOUT_MS = 120000UL;

// ===========================================================================
//  6. PERSISTENCE
// ===========================================================================
//  The reed switch is an incremental sensor -- it has no idea where it is at
//  power-up. Saving position to EEPROM on every stop means you only have to
//  home after a move that happened with the Arduino off.
//  Set to 0 on boards without EEPROM.h.
#define USE_EEPROM 1
const int EEPROM_BASE_ADDR = 0;

// ===========================================================================
//  7. SIMULATOR  --  test the whole control system with no motor hardware
// ===========================================================================
//  With this set to 1 the sketch models a virtual actuator: it watches what
//  the motor driver is being told to do and feeds back reed pulses as though
//  a real carriage were moving, including hard stops that cut pulses the way
//  the cam limit switches cut motor current.
//
//  This exercises the real interrupt, the real debounce, the real state
//  machine, homing, calibration, soft limits and stall detection. What it
//  cannot tell you is anything about the physical world: motor current,
//  actual travel, real pulse timing, EMI. Those still need the hardware.
//
//  SET THIS BACK TO 0 BEFORE CONNECTING A REAL ACTUATOR.
#define SIMULATE_ACTUATOR 1

//  How the fake pulses get in:
//    a pin number -- jumper that pin to PIN_REED. Pulses go through the real
//                    interrupt and the real debounce filter. Preferred.
//    255          -- no wire; pulses are injected in software. Use this if you
//                    have literally nothing but the board.
#define SIM_LOOPBACK_PIN 6

const uint16_t SIM_PULSE_WIDTH_US = 200;   // how long each fake pulse holds low

// The virtual actuator's own properties. Make these roughly match the real
// unit so the timing constants you tune in simulation are in the right range.
const long     SIM_TRAVEL_COUNTS  = 800;   // pulses from hard stop to hard stop
const uint16_t SIM_PULSES_PER_SEC = 20;    // pulse rate at full duty
const uint16_t SIM_STARTUP_LAG_MS = 400;   // motor-on to first pulse (breakaway)

// Spacing for injected noise pulses. Deliberately wider than REED_DEBOUNCE_US
// so they DO get counted -- that is the point of the test. This is what EMI
// from a switching bridge looks like to the sensor input.
const unsigned long SIM_NOISE_SPACING_US = 8000;
