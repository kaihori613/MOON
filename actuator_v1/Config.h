#pragma once
#include <Arduino.h>

// motor driver 
#define DRV_L298N  1
#define DRV_HW039  2

#define MOTOR_DRIVER  DRV_L298N      // <-- match your hardware

#if MOTOR_DRIVER == DRV_L298N
  // The L298N carries TWO independent H-bridges on one die. Channel A died on
  // this bench -- extend worked, retract did not, and swapping the motor leads
  // moved the fault with the transistor pair rather than with the direction,
  // which is what a dead output stage looks like. Channel B is the spare.
  //
  // Set to 1 or 2. If you simply move ENB/IN3/IN4 onto the channel-1 pins
  // below, leave this at 1 -- the sketch cannot tell the difference. Use 2
  // only if channel B is wired to its own pins as listed.
  //
  // Both channels share one die and one heat slug, so B has run as hot as A
  // did. This is a spare, not a repair.
  #define L298N_CHANNEL 1

  #if L298N_CHANNEL == 1
    const uint8_t PIN_ENA = 9;    // ENA
    const uint8_t PIN_IN1 = 6;    // IN1
    const uint8_t PIN_IN2 = 5;    // IN2
  #elif L298N_CHANNEL == 2
    const uint8_t PIN_ENA = 10;   // ENB  -- must be a PWM pin
    const uint8_t PIN_IN1 = 8;    // IN3
    const uint8_t PIN_IN2 = 12;   // IN4
  #else
    #error "Set L298N_CHANNEL to 1 or 2"
  #endif

#elif MOTOR_DRIVER == DRV_HW039
  const uint8_t PIN_RPWM = 9;    // extend
  const uint8_t PIN_LPWM = 10;   // retract
  const uint8_t PIN_EN   = 8;    

#else
  #error "Set MOTOR_DRIVER to DRV_L298N or DRV_HW039"
#endif

const uint8_t PIN_REED = 2;

// Measured on the bench, not guessed: 1 s of extend at duty 200 gave 250
// accepted pulses at a 4.0 ms mean gap, with 260 edges seen -- so 3.5% of the
// edges were contact bounce, all of it under 1 ms, and one bounce got through
// at 1.60 ms. Real travel and bounce are two clean clusters with an empty band
// between them, and 1500 us sits in that band: it rejects every bounce seen and
// still leaves 2.6x margin on the real gaps.
//
// That margin is a function of speed. At duty 255 the mean gap falls to about
// 3 ms and the fast tail moves toward this filter, so raising the duty means
// re-running 's' before trusting the count. Discarded pulses never come back.
const unsigned long REED_DEBOUNCE_US = 1500;

// speed
//
// BREAKAWAY. On the bench this actuator did not move AT ALL from a standstill
// at duty 140 -- neither direction, both cut by the grace timer with zero
// pulses -- and moved cleanly at 200. So breakaway sits somewhere in (140, 200]
// and is NOT yet pinned down. Every number below that falls in that bracket is
// provisional until someone walks 'w' up in steps and finds the real threshold.
const uint8_t SPEED_RUN  = 180;   // normal moves and jogs
// Was 150, which is below the measured 140 failure only by luck -- it is inside
// the unresolved bracket and cannot be trusted to start the carriage.
//
// The distinction that matters: breaking away from rest needs more duty than
// staying in motion does. As a DECELERATING final approach, 150 was probably
// fine, because the carriage is already moving when it gets there. As a
// CORRECTION -- a fresh re-approach from a standstill, which is what
// MAX_CORRECTIONS budgets for -- it would simply fail to move, and the stall
// watchdog would read that as a jam.
//
// Raised so corrections can start at all. The cost is that it now equals
// SPEED_RUN, so there is no slow approach left, and overshoot will be whatever
// coast at this duty gives. The real fix is a breakaway kick -- start a
// correction at SPEED_RUN and drop to a trim duty once pulses are arriving --
// rather than one constant trying to do both jobs.
const uint8_t SPEED_TRIM = 180;   // final approach, and every correction
// Known optimistic: 140 is proven not to move this actuator, so a floor of 60
// refuses nothing that matters. Left alone rather than replaced with another
// unmeasured number -- close the bracket above first.
const uint8_t SPEED_FLOOR = 60;
const uint8_t SPEED_HOMING = 150; // also inside the unresolved bracket

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
#define USE_LCD 0
const uint8_t  LCD_I2C_ADDR   = 0x27;
const uint8_t  LCD_COLS       = 16;
const uint16_t LCD_REFRESH_MS = 200;

// ===========================================================================
//  11. MANUAL CONTROL  --  hold-to-run buttons and a kill switch
// ===========================================================================
//  Three panel inputs, all INPUT_PULLUP, all wired to GND. None of these pins
//  is used by either driver option or by the reed, so they cost nothing.
const uint8_t PIN_BTN_EXTEND  = 3;
const uint8_t PIN_BTN_RETRACT = 4;
const uint8_t PIN_KILL        = 7;

//  KILL SWITCH WIRING. Set to 0 for a normally-OPEN button -- the common
//  pushbutton -- wired between D7 and GND. Unpressed reads HIGH and is
//  healthy; pressing pulls D7 LOW and kills. That is how this bench is wired.
//
//  Set to 1 if the switch is normally CLOSED, held closed to GND so D7 reads
//  LOW while healthy. That version is strictly safer: a broken wire, a pulled
//  connector or a failed switch all read HIGH and all kill, where a
//  normally-open button kills only when pressed and a severed wire disarms it
//  silently. Worth moving to if the switch has an NC contact block.
#define KILL_IS_NORMALLY_CLOSED 0

//  Mechanical contacts, so tens of milliseconds rather than the microseconds
//  the reed needs. Nothing here is timing-critical: a button is not a sensor.
//  The kill switch is deliberately NOT debounced on assert -- it acts on the
//  first sample and latches, and chatter on release cannot un-latch it.
const uint8_t BTN_DEBOUNCE_MS = 25;

//  Duty for a held button. Follows 'v' so the panel and the console agree
//  about what "speed" means; set it to a fixed number here if you would rather
//  the buttons always ran at one duty regardless of the console.
#define MANUAL_SPEED  g_speed
