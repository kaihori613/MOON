#pragma once
#include <Arduino.h>

// motor driver 
#define DRV_L298N  1
#define DRV_HW039  2

#define MOTOR_DRIVER  DRV_L298N      // <-- match your hardware

#if MOTOR_DRIVER == DRV_L298N
  const uint8_t PIN_ENA = 9;
  const uint8_t PIN_IN1 = 6;
  const uint8_t PIN_IN2 = 5;

#elif MOTOR_DRIVER == DRV_HW039
  const uint8_t PIN_RPWM = 9;    // extend
  const uint8_t PIN_LPWM = 10;   // retract
  const uint8_t PIN_EN   = 8;    

#else
  #error "Set MOTOR_DRIVER to DRV_L298N or DRV_HW039"
#endif

const uint8_t PIN_REED = 2;

const unsigned long REED_DEBOUNCE_US = 3000;

// speed
const uint8_t SPEED_RUN  = 180;   // normal moves and jogs
const uint8_t SPEED_TRIM = 150;   // final approach, and every correction
const uint8_t SPEED_FLOOR = 60;
const uint8_t SPEED_HOMING = 150;

const long DEADBAND_COUNTS = 1;
const long SLOW_ZONE_COUNTS = 8;

const uint16_t COAST_SETTLE_MS = 300;

// How many times a single move may re-approach after settling short of
// target. Corrections only ever run in the original direction of travel --
// see the anti-backlash note in the .ino -- so this is a budget for creeping
// up on the target, not for hunting around it.
const uint8_t MAX_CORRECTIONS = 2;

// Stay this far off each mechanical stop during normal moves. Homing and
// calibration deliberately ignore it; driving into the stop is their job.
const long SOFT_LIMIT_MARGIN = 5;

// ===========================================================================
//  5. SAFETY
// ===========================================================================

// Motor commanded on but no pulse for this long: the actuator has either hit
// an internal cam limit switch (which cuts its own motor current) or it is
// jammed. Nothing distinguishes those two from out here, which is why both
// stop the motor at once. Roughly 3x the slowest expected pulse period.
const uint16_t STALL_TIMEOUT_MS = 700;

// Grace from motor-on to the first pulse, for breakaway. Only applies while
// no pulse has arrived yet this run.
const uint16_t START_GRACE_MS = 900;

// Reversing a loaded DC motor instantly is the worst moment the supply and
// the bridge will ever see. Coast between directions.
const uint16_t REVERSE_DEAD_MS = 200;

// Absolute ceiling on any single stretch of motion. A full stroke at speed is
// well under this -- reaching it means nothing else stopped the motor, which
// is a fault, not a normal ending.
const unsigned long MAX_RUN_MS = 30000;

// ===========================================================================
//  6. CONSOLE
// ===========================================================================

const uint16_t JOG_MS_DEFAULT = 1000;   // 'e' / 'r' with no argument
const uint16_t JOG_MS_MIN     = 50;
const uint16_t JOG_MS_MAX     = 10000;

const long     STEP_COUNTS_DEFAULT = 1; // bare '+' / '-'
const uint16_t TELEMETRY_MS        = 250;
