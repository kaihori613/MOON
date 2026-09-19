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
//  Two drivers are supported. Set MOTOR_DRIVER to whichever is actually
//  wired, the way v1 did -- switching should be a config change, not a
//  rewrite.
//
//  DRV_L298N is what is on the bench. At 25 V it is inside its voltage
//  rating, but read section 1b before trusting it: the current and the heat
//  are the problems, not the volts.
//
//  DRV_G2 is the Pololu G2 24v13, the intended eventual part. Two-pin
//  interface -- one direction bit and one PWM -- plus /SLP, which MUST be
//  driven HIGH or the bridge stays asleep and a perfectly correct PWM moves
//  nothing while looking exactly like a dead motor.
#define DRV_L298N 1
#define DRV_G2    2

#define MOTOR_DRIVER  DRV_L298N      // <-- match your hardware

#if MOTOR_DRIVER == DRV_L298N
  // Channel B, the same pins v1 and reed_switch_test use. Channel A on this
  // board is DEAD, and dangerously so: OUT1's high side is shorted on. It
  // held ~20 V after motorOff() and no firmware can switch it off. Leave
  // OUT1/OUT2 unconnected and do not put this back to 1.
  //
  // The names stay ENA/IN1/IN2 so the driver code reads the same either way;
  // on channel 2 they are wired to ENB/IN3/IN4. ENB is Timer1 (OC1B), the
  // same timer and ~490 Hz that D9 gave channel A.
  #define L298N_CHANNEL 2

  #if L298N_CHANNEL == 1
    const uint8_t PIN_ENA = 9;    // ENA
    const uint8_t PIN_IN1 = 6;    // IN1
    const uint8_t PIN_IN2 = 5;    // IN2
  #elif L298N_CHANNEL == 2
    const uint8_t PIN_ENA = 10;   // ENB  -- must be a PWM pin
    const uint8_t PIN_IN1 = 8;    // IN3
    const uint8_t PIN_IN2 = 12;   // IN4

    // The dead channel's inputs, held LOW from the first line of setup().
    // Left floating, ENA can drift high while IN1 drifts low, which turns
    // OUT1's low side on underneath the shorted high side: a dead short from
    // the 25 V rail through the die, taking channel B with it. The module's
    // ENA jumper must be OFF for this to mean anything.
    const uint8_t PIN_DEAD_ENA = 9;
    const uint8_t PIN_DEAD_IN1 = 6;
    const uint8_t PIN_DEAD_IN2 = 5;
  #else
    #error "Set L298N_CHANNEL to 1 or 2"
  #endif
#elif MOTOR_DRIVER == DRV_G2
  const uint8_t PIN_PWM = 9;   // Timer1. Never 5/6: analogWrite(0) on Timer0
                               //   may not fully release the output.
  const uint8_t PIN_DIR = 12;  // not D7: D7 is the kill input on both drivers
  const uint8_t PIN_SLP = 8;   // HIGH = awake. LOW is the hard off switch.
#else
  #error "Set MOTOR_DRIVER to DRV_L298N or DRV_G2"
#endif

// Current sense. The G2 provides one; the L298N module's sense pins are
// usually jumpered to ground, so leave USE_CURRENT_LIMIT off with it. A3 is
// otherwise free on this panel.
const uint8_t PIN_CS = A3;

// ===========================================================================
//  1b. RUNNING AN L298N AT 25 V
// ===========================================================================
//  It is within the part's voltage rating -- Vs goes to 46 V -- so the volts
//  are not the issue. Three other things are, and all three are survivable
//  for bring-up on a duty cycle this low: a move takes seconds and then the
//  dish sits still for hours, so thermal mass matters more than the
//  continuous rating.
//
//    1. CURRENT. 2 A per channel. On a healthy module the two bridges can be
//       paralleled for about 4 A. NOT ON THIS ONE: channel A's high side is
//       shorted, and tying OUT1 to OUT3 would put that short on the motor
//       lead. This board runs channel B alone, so 2 A is the ceiling.
//
//    2. HEAT. The outputs are Darlingtons, so they drop 2-3 V regardless of
//       load. At 3 A that is 7 W in the package, and the postage-stamp
//       heatsink on the red modules will not carry it. Fit a real one.
//
//    3. THE DROP COMES OFF THE MOTOR. 25 V in, roughly 22 V at the actuator.
//       Harmless here, but it means the duty numbers found during tuning do
//       not transfer unchanged to a MOSFET bridge later.
//
//  Measure the stall current before deciding this is fine. If it is over
//  about 3 A even paralleled, the L298N is the wrong part and the G2 branch
//  above is waiting.

// Reed switch. Still on INT0, but it is no longer a position sensor -- see
// section 5. Nothing here integrates it.
const uint8_t PIN_REED = 2;

// Limit switches at the pivot -- "limit switch 1" on A0, "limit switch 2" on
// A1. SIGNAL ONLY: they do not cut motor current, so stopping at +/-15 deg is
// this firmware's job, backed only by the actuator's own internal cams.
//
// LIMITS_NORMALLY_CLOSED 1 is for a microswitch wired COM-NC to GND: a broken
// wire then reads as TRIPPED rather than as permission to keep going. Set it
// to 0 if they are wired COM-NO, and know that a broken wire then silently
// removes that limit.
#define LIMITS_NORMALLY_CLOSED 1

// Software glitch filter on the limit and home switches -- the second half of
// the RC filter in WIRING.md, not a replacement for it. The limits are
// deliberately lopsided: a trip must hold LIMIT_TRIP_MS to count, but a
// tripped limit must read clear for LIMIT_CLEAR_MS before it is believed.
// A spike on the cable can then neither fake a trip nor, worse, make a
// tripped limit look clear for an instant. Both are far below anything the
// boom does in a 20 ms tick.
const uint8_t LIMIT_TRIP_MS  = 2;
const uint8_t LIMIT_CLEAR_MS = 20;
const uint8_t REF_STABLE_MS  = 2;       // home switch, both ways
//
// POS is the switch the boom reaches while EXTENDING. Check it before any
// powered move: trip each by hand and watch 'lim=' in 's' -- the + must light
// for the extend-side switch. If it is the other way round, swap these two.
const uint8_t PIN_LIM_POS = A0;
const uint8_t PIN_LIM_NEG = A1;

// Reference ("home") switch, mid-travel, on A2. NOT one of the hard stops --
// see section 4b for why that distinction is the whole point. NO to GND.
const uint8_t PIN_REF = A2;

// Panel buttons on the pins v1 and reed_switch_test use, so the bench does not
// have to be rewired between sketches. There is no separate STOP button on
// this panel; the kill switch on D7 stops everything.
const uint8_t PIN_BTN_EXTEND  = 3;
const uint8_t PIN_BTN_RETRACT = 4;
#define USE_STOP_BUTTON 0
const uint8_t PIN_BTN_STOP    = A3;     // only read when USE_STOP_BUTTON is 1
const uint8_t PIN_BUZZER      = 11;

// KILL BUTTON on D7: a normally-open push button to GND, the same as
// reed_switch_test and v1. It is a latching fault: the motor stops on the
// first sample and stays stopped after the button is let go -- release it,
// then 'k'. 'k' refuses while it is still held.
//
// It is only a request to the firmware and cannot stop a shorted bridge, so
// keep a hardware way to cut the 25 V within reach.
//
// Set KILL_IS_NORMALLY_CLOSED to 1 only if it is ever swapped for an NC
// contact held closed to GND. Get it backwards and the sketch boots killed.
const uint8_t PIN_KILL = 7;
#define USE_KILL                1
#define KILL_IS_NORMALLY_CLOSED 0

// MANUAL MODE. Hold-to-run: released is stopped, that instant. A serial
// console has no key-up event, which is why the bring-up sketches jogged on a
// timer and why a physical button is worth the three pins.
//
// A button press ALWAYS wins. It aborts whatever the host had commanded and
// takes ownership of the axis, because the person holding the button is
// standing next to the dish and the host is not.
#define USE_BUTTONS 1

// BUZZER on D11 through an NPN (WIRING.md). tone() owns Timer2, which only
// costs PWM on D3 and D11 -- D3 is a button input, so nothing is lost; the
// motor PWM is Timer1. Three sounds, none of which ever blocks the loop:
//   one short    booted and the encoder answered
//   two short    reference pass finished (the "homing" pass, 'h' / 'H')
//   three long   any fault, the kill switch included
// A passive piezo sounds at BEEP_HZ. An active buzzer sounds at its own pitch
// and just follows the on/off pattern.
#define USE_BUZZER  1
const uint16_t BEEP_HZ       = 2000;
const uint16_t BEEP_SHORT_MS = 70;
const uint16_t BEEP_LONG_MS  = 400;

const uint16_t BTN_DEBOUNCE_MS = 25;

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

// Manual jogs are deliberate, not fast. Tied to SPEED_SLOW rather than given
// its own number so the two cannot drift apart during tuning.
const uint8_t SPEED_BUTTON = SPEED_SLOW;

// ===========================================================================
//  4. TRAVEL LIMITS
// ===========================================================================
//  Three layers, outermost last:
//    1. soft limit here, in firmware, from the encoder
//    2. the limit switches at +/-15 deg -- signal only, so the firmware
//       stops the motor when one trips; they cut nothing themselves
//    3. the actuator's own internal cams at the ends of the stroke, the
//       only layer that cuts motor current in hardware
//
//  Keep the soft limit inside the switches so normal operation never reaches
//  them; a limit trip should mean something went wrong, not "Tuesday". Two
//  degrees of margin covers coast plus backlash with room to spare.
const float HARD_STOP_DEG  = 15.0f;   // where the limit switches physically are
const float SOFT_LIMIT_DEG = 13.0f;   // where the firmware refuses to go

// ===========================================================================
//  4b. REFERENCE SWITCH  --  designed, and probably not needed
// ===========================================================================
//  An earlier revision planned a third switch, mid-travel, on the argument
//  that the encoder's zero can creep with no symptom: zero is a raw count in
//  EEPROM, and a magnet that slips on its hub makes every angle wrong by a
//  constant.
//
//  The first half of that is true. The second half is not, and it is why
//  USE_REF_SWITCH is 0 and likely to stay there. THE SATELLITE IS A
//  REFERENCE, continuously available and far more sensitive than any
//  microswitch: if the zero drifts, the signal at the aimed angle falls, and
//  the receiver is being watched anyway.
//
//  Better still, the drift repairs itself. A calibration run does not care
//  what "zero" means -- it finds the encoder reading with the best signal and
//  stores that -- so a shifted zero is absorbed into the next run.
//
//  What the switch would add over that is the ability to tell "the mount
//  moved" from "the sensor moved". Both look like "signal is worse than it
//  was", and for RECOVERY the distinction does not matter, because you
//  re-peak either way. It is a diagnostic convenience costing a switch, a
//  pin, a cam lobe, outdoor wiring and one more thing to fail on a mast.
//
//  host/peak.py's check_pointing() does the detecting instead, and catches a
//  shifted mount, a slipped magnet, a wet LNA and a failing feed in one test.
//
//  A reference switch WOULD earn its place on a system that cannot verify
//  against its payload -- a telescope that only observes at night, a machine
//  tool where a bad zero ruins the work before anyone notices. None of that
//  is this. The code below stays because it is written and guarded and costs
//  nothing at 0; fit the switch only if a reason appears that the signal
//  cannot cover.
//
//  WHAT IS LOST WITHOUT IT, precisely: nothing about RELATIVE angle. The
//  encoder still reads the boom continuously and absolutely, to 0.0879 deg,
//  and "move 3 degrees" still means three degrees. What is arbitrary is only
//  the LABEL on the scale -- 0.0 means wherever the boom was when 'z' was
//  typed. Pointing at a geostationary bird needs only the relative part.
//
//  THIS BENCH HAS ONE FITTED -- the "home switch" on A2 -- so it is on. It
//  does nothing until asked: 'h' crosses it and reports drift, 'H' crosses it
//  and adopts the crossing as zero, then 'w' saves. Set REF_ANGLE_DEG to
//  where the switch actually sits before trusting 'H'.
#define USE_REF_SWITCH 1

// Only read when USE_REF_SWITCH is 1.
const float REF_ANGLE_DEG  = -5.0f;    // PLACEHOLDER  where the switch sits
const float REF_MARGIN_DEG =  1.5f;    // how far clear of it a pass starts

const uint8_t  REF_CROSS_SPEED = 70;   // must still be above breakaway
const uint16_t REF_TIMEOUT_MS  = 30000;

// Raw counts at the entering edge, per direction. -1 = never adopted. Two of
// them because a microswitch's trip and release points differ, so a crossing
// is only repeatable per direction; their difference is the lobe width plus
// hysteresis, which is its own diagnostic.
const int16_t REF_RAW_POS_DEFAULT = -1;
const int16_t REF_RAW_NEG_DEFAULT = -1;

const float REF_DRIFT_WARN_DEG = 0.5f;  // PLACEHOLDER -- measure the switch

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

#if USE_CURRENT_LIMIT && USE_STOP_BUTTON
  #error "A3 cannot be both the current sense and the stop button -- move PIN_BTN_STOP"
#endif

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
