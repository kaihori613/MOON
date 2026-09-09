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

// D2 is the only external interrupt the position loop can use. See WIRING.md
// for the pull-up, series resistor and filter cap that go with it.
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
//  driver is under a hundred lines. SDA/SCL are A4/A5 on an Uno, and the only
//  other thing that may share them is the IMU in section 12 -- 0x27 and 0x68
//  do not collide, so no arbitration is needed.
//
//  Most backpacks are 0x27. A few are 0x3F. If the display stays blank but the
//  backlight is on, try the other one.
#define USE_LCD 0
const uint8_t  LCD_I2C_ADDR   = 0x27;
const uint8_t  LCD_COLS       = 16;
const uint16_t LCD_REFRESH_MS = 200;

// ===========================================================================
//  11. FRONT PANEL  --  buttons and buzzer
// ===========================================================================
//  Staged the way the LCD is: the pins are reserved here so that WIRING.md and
//  the firmware cannot drift apart. NO CODE READS THEM YET. Setting either
//  flag to 1 today changes nothing except the documentation being true.
//
//  Buttons short to GND and rely on INPUT_PULLUP, so a disconnected button
//  reads as not-pressed rather than floating. They are on the analog pins
//  because D4/D7/D12/D13 are worth keeping contiguous and free, and because
//  D8 and D10 stay reserved for the HW-039 whichever driver is selected.
//
//  Debounce here is a human-timescale problem and wants nothing like the
//  reed's treatment -- no series resistor, no capacitor, no interrupt.
#define USE_BUTTONS 0
const uint8_t  PIN_BTN_EXTEND  = A0;
const uint8_t  PIN_BTN_RETRACT = A1;
const uint8_t  PIN_BTN_STOP    = A2;
const uint16_t BTN_DEBOUNCE_MS = 25;

//  Held-button jogs, not timed ones. A button has a key-up event, which is the
//  whole reason actuator_test/ preferred it to the serial console.
const uint8_t BTN_JOG_SPEED = SPEED_RUN;

//  tone() owns Timer2, which is PWM on D3 and D11. Motor PWM is Timer1 under
//  both drivers -- D9 for the L298N, D9/D10 for the HW-039 -- so sounding the
//  buzzer cannot disturb speed control. That is why D11 is safe to give away.
//
//  BUZZER_ACTIVE 1 means the buzzer makes its own tone and the pin just gates
//  it; 0 means it is passive and wants tone(). Drive either through a
//  transistor unless it is a bare piezo -- see WIRING.md.
#define USE_BUZZER 0
const uint8_t  PIN_BUZZER    = 11;
#define BUZZER_ACTIVE 1
const uint16_t BUZZER_HZ     = 2400;   // passive buzzers only
const uint16_t BUZZER_BEEP_MS = 60;

// ===========================================================================
//  12. IMU  --  MPU6050 on the same I2C bus as the LCD
// ===========================================================================
//  0x27 and 0x68 do not collide, so the display and the sensor share A4/A5
//  with no arbitration needed. AD0 left floating gives 0x68; tie it high for
//  0x69 if something else ever wants the low address.
//
//  READ THIS BEFORE WIRING IT. The MPU6050 has a gyro and an accelerometer and
//  NO MAGNETOMETER, and the single actuator here is assigned to yaw -- the one
//  axis a six-axis part cannot measure absolutely. The accelerometer finds
//  gravity, which gives pitch and roll outright, but rotation about the gravity
//  vector leaves it unchanged. The gyro gives yaw RATE, and integrating that
//  drifts without bound with nothing magnetic to correct against.
//
//  So this part cannot replace the compass sighting behind 'a' and 'b', and
//  cannot check the degrees readout. What it can do -- pitch as a real
//  readout, back-drive detection with the motor off, and an independent
//  witness that the dish is moving when the reed says it is not -- is written
//  up in WIRING.md. None of it is implemented.
//
//  Note that actuator_v1.ino guards <Wire.h> on USE_LCD alone. Setting
//  USE_IMU without USE_LCD will not pull the library in until that guard
//  becomes USE_LCD || USE_IMU, which is a job for whoever writes the driver.
#define USE_IMU 0
const uint8_t MPU6050_I2C_ADDR = 0x68;
const uint8_t PIN_IMU_INT      = 3;   // INT1. Optional -- polling works fine
