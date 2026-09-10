#pragma once
#include <Arduino.h>

// ===========================================================================
//  MOON actuator_v2 -- Rev B
// ===========================================================================
//  What changed from v1, and why every constant below looks different:
//
//  v1 counted reed pulses and signed them by the direction it had last
//  commanded. That is a relative measurement taken on the WRONG SIDE of the
//  linkage -- upstream of the leadscrew backlash -- with no direction of its
//  own, and it went blind the moment wind back-drove the dish with the motor
//  off. Everything expensive in v1 (homing, the EEPROM position and its
//  check-on-next-home, the travel calibration, the midpoint origin, the a/b
//  compass fit) existed to work around not knowing where the boom was.
//
//  An absolute encoder on the pivot knows where the boom is at power-on, and
//  all of that goes away. See WIRING.md.
//
//  PLACEHOLDER marks a constant that has not been measured. Every PID gain
//  here is a placeholder -- they are a starting point for the tuning
//  procedure in the README, not values anybody has run.

// ===========================================================================
//  1. PINS
// ===========================================================================
//  Pololu G2 24v13. Two-pin interface -- a direction bit and one PWM -- which
//  is why IN1/IN2 are gone. /SLP must be driven HIGH or the driver stays
//  asleep and the motor never moves however good the PWM looks.
const uint8_t PIN_PWM = 9;      // Timer1. Never pins 5/6: analogWrite(0) on
const uint8_t PIN_DIR = 7;      //   Timer0 may not fully release the output.
const uint8_t PIN_SLP = 8;      // HIGH = awake. LOW is the hard off switch.
const uint8_t PIN_CS  = A3;     // driver current sense, analog

// Reed switch. Still on INT0, but it is no longer a position sensor -- see
// section 5. Nothing here integrates it.
const uint8_t PIN_REED = 2;

// Limit cams at the pivot. NC to GND with INPUT_PULLUP, so a broken wire
// reads as TRIPPED rather than as permission to keep going. These also cut
// motor current in hardware; the pins only tell the firmware what the wiring
// has already done.
const uint8_t PIN_LIM_POS = 3;
const uint8_t PIN_LIM_NEG = 4;

// Reference switch, mid-travel. NOT one of the hard stops -- see section 4b
// for why that distinction is the whole point. D5 is free for it because
// motor PWM is Timer1 (D9), so nothing here needs Timer0's pins.
const uint8_t PIN_REF = 5;

const uint8_t PIN_BTN_EXTEND  = A0;
const uint8_t PIN_BTN_RETRACT = A1;
const uint8_t PIN_BTN_STOP    = A2;
const uint8_t PIN_BUZZER      = 11;

#define USE_BUTTONS 0           // reserved; no code reads these yet
#define USE_BUZZER  0

// ===========================================================================
//  2. ENCODER  --  AS5600 on I2C
// ===========================================================================
//  12 bits over one turn: 4096 counts, 0.0879 deg each. The address is fixed
//  in silicon at 0x36 and cannot be strapped, which is fine -- there is one.
//
//  MOUNTING: the working range must not straddle the 0/4095 wrap. Fit the
//  magnet so that the middle of travel lands near raw 2048. angleDiff() is
//  wrap-safe regardless, but a wrap inside the travel makes every printed
//  number confusing to read on the bench.
const uint8_t  AS5600_ADDR      = 0x36;
const uint16_t AS5600_COUNTS    = 4096;
const float    DEG_PER_COUNT    = 360.0f / (float)AS5600_COUNTS;

// Raw count that reads as 0.000 deg. Set it on the bench with 'z' at the
// aimed position; it is saved to EEPROM.
const int16_t  ENCODER_ZERO_DEFAULT = 2048;

// Set to 1 if the encoder counts DOWN when the boom moves in the direction
// the driver calls positive. Getting this backwards turns the loop into
// positive feedback and it will run to a limit at full duty on the first
// move -- check it with 'm' before ever enabling the loop.
#define ENCODER_INVERT 0

// I2C read failures are not survivable mid-move: feeding a stale or garbage
// angle to the PID is worse than stopping. This many consecutive failures
// faults the axis.
const uint8_t ENCODER_MAX_FAILS = 3;

// ===========================================================================
//  3. CONTROL LOOP
// ===========================================================================
//  50 Hz. The mechanism is nowhere near that fast -- at a full stroke in tens
//  of seconds the encoder only produces a new count every few ticks -- so the
//  sensor, not the loop rate, is the bandwidth limit here. Going faster would
//  buy nothing and only sharpen the quantisation noise the D term sees.
const uint16_t LOOP_MS = 20;

// --- gains -------------------------------------------------------------
//  All PLACEHOLDER. Tune in this order, live, with 'p'/'i'/'d'/'f' -- no
//  reflash needed, and 't' runs a step and prints the response as CSV.
//
//  Start with KI and KD at zero. On a friction-dominated plant the term that
//  actually earns its place is the feedforward, not the integrator.
const float KP_DEFAULT  = 12.0f;    // PLACEHOLDER  duty per degree of error
const float KI_DEFAULT  =  0.0f;    // PLACEHOLDER  add only if offset persists
const float KD_DEFAULT  =  0.0f;    // PLACEHOLDER  add only if it overshoots

// Friction feedforward. Below the breakaway duty the motor buzzes and does
// not turn, so every correction starts there and the gains only trim on top.
// Set this to the breakaway duty measured on the bench -- it is the same
// number v1 called SPEED_FLOOR.
const float KFF_DEFAULT = 60.0f;    // PLACEHOLDER

// Integrator clamp, in duty units, so a wound-up I term can never own more
// than a fraction of the output on its own.
const float I_MAX = 60.0f;

// --- deadband -----------------------------------------------------------
//  This is the cheapest fix in the whole design, and it works because the
//  pointing requirement is loose. Half-power beamwidth on a 1 m dish at
//  1694 MHz is about 12 deg, and 0.1 dB of loss is +/-1.1 deg off boresight.
//  So a deadband far larger than the linkage backlash still costs a pointing
//  error nobody can measure -- and a deadband larger than the backlash is
//  exactly what stops an integrator hunting across the dead zone all night.
//
//  Set it wider than the measured backlash, not tighter.
const float DEADBAND_DEG   = 0.25f;   // PLACEHOLDER -- measure backlash first
const float SLOW_ZONE_DEG  = 1.50f;   // approach below this at reduced ceiling

const uint8_t SPEED_MAX   = 200;      // ceiling on |duty|
const uint8_t SPEED_SLOW  = 110;      // ceiling inside SLOW_ZONE_DEG
const uint8_t SPEED_FLOOR = 60;       // PLACEHOLDER breakaway; below = buzz

// ===========================================================================
//  4. TRAVEL LIMITS
// ===========================================================================
//  Three layers, outermost last:
//    1. soft limit here, in firmware, from the encoder
//    2. the cam microswitches at +/-10 deg, which cut current in hardware
//    3. the actuator's own internal cams at the ends of the stroke
//
//  Keep the soft limit inside the cams so normal operation never reaches
//  them; a cam trip should mean something went wrong, not "Tuesday".
const float SOFT_LIMIT_DEG = 9.0f;

// ===========================================================================
//  4b. REFERENCE SWITCH  --  a THIRD switch, mid-travel
// ===========================================================================
//  The +/-10 deg cams are HARD STOPS. They exist to protect the antenna, they
//  cut motor current in hardware, and normal operation must never reach them
//  -- a cam trip is a fault, not a step in a procedure.
//
//  So the reference does NOT reuse one of them. An earlier revision did, and
//  it was wrong for three reasons that all matter more than saving a pin:
//
//    * every home drove deliberately into a safety device, wearing the switch
//      and, worse, the cam-to-lever alignment that IS the protection;
//    * "the minus cam is tripped" stopped being an alarm condition, because
//      it also meant "we are homing" -- so a genuine runaway looked normal;
//    * the escape diodes cut current the instant the cam opens, so every home
//      ended with the mechanism hard-cut by a safety circuit rather than
//      decelerating under control. Twice per home, forever.
//
//  Instead there is a third switch somewhere inside the travel, crossed in
//  transit and never rested on. It carries no safety duty at all, so driving
//  across it is free.
//
//  WHAT IT IS FOR
//
//  The encoder is absolute, but its ZERO is not: zero is a raw count in
//  EEPROM, and nothing in the encoder can reveal that the magnet has crept on
//  its hub or that the EEPROM was wiped. Every angle would be wrong by a
//  constant, confidently, with no symptom. The reference switch is the
//  physical fact you check that against.
//
//  Being mid-travel turns that from a procedure into passive monitoring. The
//  boom crosses the switch on ordinary moves, so the firmware captures the
//  crossing every time and compares it -- no homing cycle to remember to run.
//  'h' forces a deliberate slow pass when you want the authoritative number.
//
//  WHY TWO STORED REFERENCES
//
//  A microswitch's trip point and release point differ, so the crossing is
//  only repeatable per DIRECTION. Crossing while moving positive and crossing
//  while moving negative are two different, individually stable numbers, and
//  each is compared against its own reference. Their difference is the lobe
//  width plus hysteresis -- a constant of the mechanism, and a free diagnostic
//  in its own right: if it changes, the lever is bending or the cam is loose.
//
//  Both are captured on the INACTIVE->ACTIVE edge, so the lobe only has to be
//  entered, never traversed at a known speed.
#define USE_REF_SWITCH 1

// Roughly where the switch sits. Only used to plan the deliberate pass; the
// stored raw counts are what is actually believed. Put it off the angle the
// dish normally parks at, so the boom is never left resting on the lever.
const float REF_ANGLE_DEG  = -5.0f;    // PLACEHOLDER
const float REF_MARGIN_DEG =  1.5f;    // how far clear of it a pass starts

// The deliberate pass crosses slowly, because the capture is polled at the
// loop rate: at 50 Hz and this duty the boom moves far less than one encoder
// count between polls, so the latency costs nothing measurable.
const uint8_t  REF_CROSS_SPEED = 70;   // must still be above breakaway
const uint16_t REF_TIMEOUT_MS  = 30000;

// Raw counts seen at the entering edge, per direction. -1 = never adopted.
const int16_t REF_RAW_POS_DEFAULT = -1;   // captured while moving positive
const int16_t REF_RAW_NEG_DEFAULT = -1;   // captured while moving negative

// Passive crossings during ordinary moves are compared against the stored
// reference and reported above this. Set it above the switch's own
// repeatability or every move will cry wolf.
const float REF_DRIFT_WARN_DEG = 0.5f;    // PLACEHOLDER -- measure the switch

// ===========================================================================
//  5. HEALTH  --  what the reed is for now
// ===========================================================================
//  The reed no longer measures anything. It witnesses. Two failures that
//  neither sensor can see alone:
//
//    reed silent, motor commanded on   -> stall, cam cut, or blown fuse
//    reed pulsing, encoder not moving  -> the LINKAGE HAS BROKEN, and the
//                                         motor is happily driving nothing
//
//  The second one is the reason to keep the reed wired at all. An encoder on
//  the pivot cannot distinguish a broken coupling from a stalled motor; a
//  reed on the motor side cannot tell you where the boom is. Together they
//  can do both.
const unsigned long REED_DEBOUNCE_US = 3000;   // measured -- see README

const uint16_t STALL_TIMEOUT_MS = 700;   // commanded on, no pulse this long
const uint16_t START_GRACE_MS   = 900;   // breakaway grace before the above

// Linkage check: this many reed pulses arriving while the boom moves less
// than LINKAGE_MIN_DEG means the motor is turning and the dish is not.
const uint8_t  LINKAGE_PULSES  = 12;
const float    LINKAGE_MIN_DEG = 0.15f;  // PLACEHOLDER -- needs mm_per_count

// Reversing a loaded DC motor instantly is the worst moment the supply and
// the bridge will ever see. Coast between directions.
const uint16_t REVERSE_DEAD_MS = 200;

// Absolute ceiling on any one stretch of motion. Reaching it means nothing
// else stopped the motor, which is a fault, not a normal ending.
const unsigned long MAX_RUN_MS = 30000;

// Driver current sense. The G2's CS output is roughly 20 mV per amp into a
// 5 V ADC; the real scale factor belongs here once it has been measured
// against a clamp meter.
#define USE_CURRENT_LIMIT 0
const uint16_t CURRENT_TRIP_ADC = 700;   // PLACEHOLDER raw ADC counts

// ===========================================================================
//  6. CONSOLE
// ===========================================================================
const uint16_t TELEMETRY_MS = 250;
const uint16_t JOG_MS_DEFAULT = 500;
const uint16_t JOG_MS_MAX     = 5000;

// 't' step test: how long to stream the response, and how often.
const uint16_t STEP_TEST_MS   = 6000;
const uint16_t STEP_SAMPLE_MS = 40;

// ===========================================================================
//  7. PERSISTENCE
// ===========================================================================
//  Only the encoder zero and the gains. Position is NOT saved and never
//  needs to be -- that was v1's whole problem, and an absolute sensor makes
//  the question meaningless.
#define USE_EEPROM 1
const int EEPROM_BASE_ADDR = 64;   // clear of v1's block at 0
const uint32_t EEPROM_MAGIC = 0x4D4F4E44UL;   // "MOND" -- bumped for the two reference counts
