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

// ===========================================================================
//  7. WHERE ZERO SITS
// ===========================================================================
//  Homing always drives into the retract cam, because that is the only
//  direction-unambiguous move there is: from anywhere in the stroke, retract
//  far enough and you arrive. That is the homing REFERENCE and it is not
//  negotiable.
//
//  Where you then call zero is a separate, free choice. With this set, zero
//  lands at the middle of the stroke: negative is retracted, positive is
//  extended, and the symmetric range makes the remaining headroom obvious.
//
//  It costs one thing -- the midpoint cannot be located until the travel is
//  known, so 'c' becomes mandatory. A plain 'h' before any calibration falls
//  back to putting zero at the retract stop.
#define ORIGIN_AT_MIDPOINT 1

// ===========================================================================
//  8. ANGLE READOUT
// ===========================================================================
//  A straight-line fit of heading against reed counts, for display only. The
//  host keeps the physically-correct triangle model and remains the authority
//  on where to point; this exists so the bench can read out degrees without a
//  PC attached.
//
//  Calibrate on the bench with 'a' and 'b' -- no reflash, no tape measure:
//     1. home, drive somewhere, sight the boom, type  a <heading>
//     2. drive well along the stroke, sight again,    b <heading>
//  That solves both constants and saves them. Until then the readout shows
//  '?' rather than a confident wrong number.
//
//  Anchored to the retract stop rather than to position zero, so choosing the
//  midpoint origin above -- or recalibrating travel -- does not silently
//  invalidate it.
const float DEG_PER_COUNT_DEFAULT = 0.0f;   // 0 means "not calibrated"
const float DEG_AT_RETRACT_DEFAULT = 0.0f;  // heading with the carriage homed

// ===========================================================================
//  9. PERSISTENCE
// ===========================================================================
//  What is saved is deliberately NOT used to skip homing. A stored position is
//  silently wrong exactly when something back-drove the dish while the power
//  was off -- wind on a dish being the obvious case -- and that failure has no
//  symptom.
//
//  Instead the saved position is loaded, believed provisionally, and then
//  CHECKED by the next home: the difference between where the cam actually was
//  and where the saved position said it would be is drift, and it is a number
//  there is currently no other way to see. Homing still happens every boot.
#define USE_EEPROM 1
const int EEPROM_BASE_ADDR = 0;

// ===========================================================================
//  10. LCD  --  16x2 character display on an I2C backpack
// ===========================================================================
//  Driven straight off Wire rather than through a library: the several
//  LiquidCrystal_I2C forks disagree about constructor arguments, and the whole
//  driver is under a hundred lines. SDA/SCL are A4/A5 on an Uno; nothing else
//  in this sketch touches them.
//
//  Most backpacks are 0x27. A few are 0x3F. If the display stays blank but the
//  backlight is on, try the other one.
#define USE_LCD 1
const uint8_t  LCD_I2C_ADDR   = 0x27;
const uint8_t  LCD_COLS       = 16;
const uint16_t LCD_REFRESH_MS = 200;
