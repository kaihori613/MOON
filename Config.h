#pragma once
#include <Arduino.h>

// ===========================================================================
//  MOON controller_v1
// ===========================================================================
//  An absolute encoder on the pivot knows where the boom is at power-on, so
//  there is no homing and no stored position. Everything below is either a
//  pin, a switch for an optional feature, or a number that has to be measured.
//
//  PLACEHOLDER marks a constant that has not been measured on the dish.
//
//  Sections
//    1. Pins and motor driver
//    2. Feature switches
//    3. Encoder
//    4. Control loop
//    5. Manual control (panel buttons, serial jog)
//    6. Auto-calibration
//    7. Safety: stall, timeouts, I2C, watchdog
//    8. Console and persistence
//    9. Retained but DISABLED hardware (reed, limit switches, ref switch, ...)

// ===========================================================================
//  1. PINS AND MOTOR DRIVER
// ===========================================================================
//  Set MOTOR_DRIVER to whichever board is actually wired.
//
//  DRV_HW039  BTS7960 43A half-bridge module. RPWM/LPWM are PWM inputs, one
//             per direction; R_EN and L_EN are tied together into one EN line,
//             HIGH to enable, LOW the hard off switch. THE CURRENT PART.
//  DRV_G2     Pololu G2 24v13. One direction bit, one PWM, and /SLP, which
//             MUST be HIGH or the bridge stays asleep and a correct PWM moves
//             nothing while looking exactly like a dead motor.
//  DRV_L298N  RETIRED. Both channels of the bench module have burned out.
//             The branch stays only in case a fresh module goes back on.
#define DRV_L298N 1
#define DRV_G2    2
#define DRV_HW039 3

#define MOTOR_DRIVER  DRV_HW039      // <-- match your hardware

#if MOTOR_DRIVER == DRV_L298N
  // Channel A of the bench module is dead: OUT1's high side is shorted on and
  // no firmware can switch it off. Channel B is used; the names stay ENA/IN1/
  // IN2 so the driver code reads the same either way. The module's ENA jumper
  // must be OFF.
  #define L298N_CHANNEL 2

  #if L298N_CHANNEL == 1
    const uint8_t PIN_ENA = 9;    // ENA
    const uint8_t PIN_IN1 = 6;    // IN1
    const uint8_t PIN_IN2 = 5;    // IN2
  #elif L298N_CHANNEL == 2
    const uint8_t PIN_ENA = 10;   // ENB -- must be a PWM pin
    const uint8_t PIN_IN1 = 8;    // IN3
    const uint8_t PIN_IN2 = 12;   // IN4

    // The dead channel's inputs, held LOW from the first line of setup().
    // Floating, ENA can drift high while IN1 drifts low, which puts a dead
    // short from the supply through the die and takes channel B with it.
    const uint8_t PIN_DEAD_ENA = 9;
    const uint8_t PIN_DEAD_IN1 = 6;
    const uint8_t PIN_DEAD_IN2 = 5;
  #else
    #error "Set L298N_CHANNEL to 1 or 2"
  #endif
#elif MOTOR_DRIVER == DRV_HW039
  const uint8_t PIN_RPWM = 9;    // extend
  const uint8_t PIN_LPWM = 10;   // retract
  const uint8_t PIN_EN   = 8;    // both R_EN and L_EN, tied together
#elif MOTOR_DRIVER == DRV_G2
  const uint8_t PIN_PWM = 9;     // Timer1. Never 5/6: analogWrite(0) on Timer0
                                 //   may not fully release the output.
  const uint8_t PIN_DIR = 12;    // not D7: D7 is the kill input
  const uint8_t PIN_SLP = 8;     // HIGH = awake. LOW is the hard off switch.
#else
  #error "Set MOTOR_DRIVER to DRV_L298N, DRV_HW039 or DRV_G2"
#endif

// Panel. There is no separate STOP button; the kill switch on D7 stops
// everything.
const uint8_t PIN_BTN_EXTEND  = 3;
const uint8_t PIN_BTN_RETRACT = 4;
const uint8_t PIN_BTN_STOP    = A3;     // only read when USE_STOP_BUTTON is 1
const uint8_t PIN_KILL        = 7;      // NO push button to GND
const uint8_t PIN_BUZZER      = 11;     // through an NPN; tone() owns Timer2

// Current sense (USE_CURRENT_LIMIT). The G2 has one; the HW-039's R_IS/L_IS
// outputs are not wired here.
const uint8_t PIN_CS = A3;

// ===========================================================================
//  2. FEATURE SWITCHES
// ===========================================================================
//  Features that are fitted and in use.
#define USE_BUTTONS    1
#define USE_STOP_BUTTON 0
#define USE_BUZZER     1
#define USE_EEPROM     1
#define USE_CURRENT_LIMIT 0           // needs a measured CURRENT_TRIP_ADC

// KILL: a latching fault. The motor stops on the first sample and stays
// stopped after the button is let go -- release it, then 'k'. 'k' refuses
// while it is still held. It is a request to the firmware and cannot stop a
// shorted bridge, so keep a hardware way to cut the supply within reach.
// Set KILL_IS_NORMALLY_CLOSED to 1 only for an NC contact held closed to GND;
// backwards, the sketch boots killed.
#define USE_KILL                1
#define KILL_IS_NORMALLY_CLOSED 0

// BUZZER. None of these ever block the loop:
//   one short    booted and the encoder answered
//   two short    auto-calibration finished
//   three long   any fault, the kill switch included, or a watchdog reset
// A passive piezo sounds at BEEP_HZ; an active buzzer sounds at its own pitch.
const uint16_t BEEP_HZ       = 2000;
const uint16_t BEEP_SHORT_MS = 70;
const uint16_t BEEP_LONG_MS  = 400;

// Robustness. See section 7 for what each does and why it is on.
#ifndef USE_WATCHDOG
#define USE_WATCHDOG 1
#endif

// Features that are NOT in use on this build. The code is kept, compiled out,
// so it can be re-enabled -- see section 9 for each one's constants. The
// #ifndef lets a build flag (-DUSE_REED=1) override these without editing.
//
//   USE_REED          reed switch on the motor side, as a stall/linkage witness
//   USE_LIMIT_SWITCHES  signal-only microswitches at the pivot (A0/A1)
//   USE_REF_SWITCH    mid-travel reference ("home") switch on A2; 'h' / 'H'
//   USE_STEP_TEST     't <deg>' step response streamed as CSV, for PID tuning
//
//  Why off: the reed was not reliable, and the limit switches are not
//  practical to fit. The internal cams are the hard stops, and the encoder
//  plus auto-calibration replace homing. Position-based pointing will be
//  trimmed by the SNR search on the host after installation.
#ifndef USE_REED
#define USE_REED            0
#endif
#ifndef USE_LIMIT_SWITCHES
#define USE_LIMIT_SWITCHES  0
#endif
#ifndef USE_REF_SWITCH
#define USE_REF_SWITCH      0
#endif
#ifndef USE_STEP_TEST
#define USE_STEP_TEST       0
#endif

#if USE_CURRENT_LIMIT && USE_STOP_BUTTON
  #error "A3 cannot be both the current sense and the stop button -- move PIN_BTN_STOP"
#endif

// ===========================================================================
//  3. ENCODER  --  AS5600 on I2C
// ===========================================================================
//  12 bits over one turn: 4096 counts, 0.0879 deg each. The address is fixed
//  in silicon at 0x36.
//
//  MOUNTING: wrap-safe arithmetic is used throughout, so a working range that
//  straddles the 0/4095 wrap is handled -- but it makes printed raw counts
//  confusing on the bench. Aim for the middle of travel near raw 2048.
const uint8_t  AS5600_ADDR      = 0x36;
const uint16_t AS5600_COUNTS    = 4096;
const float    DEG_PER_COUNT    = 360.0f / (float)AS5600_COUNTS;

// Raw count that reads as 0.000 deg until a zero is set ('z') or found
// ('c y'), both of which are saved with 'w'.
const int16_t  ENCODER_ZERO_DEFAULT = 2048;

// Set to 1 if the encoder counts DOWN when the boom moves in the direction
// the driver calls positive. Backwards, the loop is positive feedback and runs
// to a stop at full duty on the first move. Auto-calibration checks this: it
// refuses to adopt a zero if the travel it measured has the wrong sign.
#define ENCODER_INVERT 1

// An I2C read that fails mid-move would feed a stale angle to the PID, which
// is worse than stopping. This many consecutive failures faults the axis.
const uint8_t ENCODER_MAX_FAILS = 3;

// ===========================================================================
//  4. CONTROL LOOP
// ===========================================================================
//  50 Hz. The mechanism is far slower than that, so the sensor, not the loop
//  rate, is the bandwidth limit.
const uint16_t LOOP_MS = 20;

// --- gains ---------------------------------------------------------------
//  Tune live with 'p'/'i'/'d'/'f', then save with 'w'. These defaults apply
//  only until something is saved. Start with KI and KD at zero: on a
//  friction-dominated plant the term that earns its place is the feedforward.
const float KP_DEFAULT  = 12.0f;    // duty per degree of error
const float KI_DEFAULT  =  0.0f;    // add only if offset persists
const float KD_DEFAULT  =  0.0f;    // add only if it overshoots

// Friction feedforward: the duty that actually moves the dish. Below
// breakaway the motor buzzes and does not turn, so every correction starts
// here and the gains only trim on top.
const float KFF_DEFAULT = 180.0f;

// Integrator clamp, in duty units.
const float I_MAX = 60.0f;

// --- deadband ------------------------------------------------------------
//  The pointing requirement is loose (half-power beamwidth of a 1 m dish at
//  1694 MHz is about 12 deg; 0.1 dB of loss is +/-1.1 deg off boresight), so a
//  deadband wider than the linkage backlash costs nothing measurable and is
//  exactly what stops an integrator hunting across the dead zone.
const float DEADBAND_DEG   = 0.25f;
const float SLOW_ZONE_DEG  = 1.50f;   // approach below this at reduced ceiling

const uint8_t SPEED_MAX   = 255;      // ceiling on |duty|
const uint8_t SPEED_SLOW  = 110;      // ceiling inside SLOW_ZONE_DEG; also 'e'/'r' jogs
const uint8_t SPEED_FLOOR = 60;       // PLACEHOLDER breakaway; below = buzz

// Largest target the console will accept. This is NOT a travel limit -- there
// is no soft limit, the internal cams are the stops. It only rejects numbers
// the wrap-safe angle frame cannot represent. A target past the real travel
// pushes on the cam until the stall check faults (section 7). Set it to
// TRAVEL_HALF_DEG if a typo guard is wanted.
const float TARGET_LIMIT_DEG = 179.0f;

// ===========================================================================
//  5. MANUAL CONTROL
// ===========================================================================
//  Hold-to-run: released is stopped, that instant. A serial console has no
//  key-up event, which is why a physical button is worth the pins.
//
//  A press ALWAYS wins. It aborts whatever the host or a calibration was
//  doing and takes ownership of the axis, because the person holding the
//  button is standing next to the dish and the host is not.
//
//  Each press starts slow, for fine trimming and short taps, and goes to full
//  duty if the button is still held after BTN_SLOW_MS -- one pair of buttons,
//  both fine adjustment and long travel.
#define USE_BUTTON_RAMP 1
const uint8_t  SPEED_BUTTON_FAST = SPEED_MAX;
const uint8_t  SPEED_BUTTON_SLOW = 180;   // PLACEHOLDER. Starts at KFF because
                                          //   that duty is known to move the
                                          //   dish. Lower it until it stops
                                          //   starting reliably, then go back
                                          //   up one step.
const uint16_t BTN_SLOW_MS     = 1200;    // hold this long before full duty
const uint16_t BTN_DEBOUNCE_MS = 25;

// Serial jogs ('e' / 'r').
const uint16_t JOG_MS_DEFAULT = 500;
const uint16_t JOG_MS_MAX     = 5000;

// ===========================================================================
//  6. AUTO-CALIBRATION  ('c y')
// ===========================================================================
//  Finds the zero from the mechanism itself, for when the stored zero has
//  been lost. It drives to each internal cam stop at full duty, takes the
//  midpoint of the measured travel as 0 deg, and centres the dish there.
//  There are no limit switches; an end stop is recognised by the encoder
//  ceasing to move.
//
//  Nothing is adopted unless the measured span looks like the real one, so a
//  sweep that stalls early leaves the old zero untouched instead of saving
//  garbage.
const uint8_t  SPEED_CALIB = SPEED_MAX;

// Half the travel, measured: centred at 0 deg the dish extends and retracts
// this far. The span a sweep must measure is twice it, within +/- the
// tolerance.
const float TRAVEL_HALF_DEG  = 59.0f;
const float CAL_SPAN_TOL_DEG = 12.0f;

// End stop = fewer than CAL_MOVE_COUNTS of encoder movement in CAL_STALL_MS.
// 4 counts is 0.35 deg, clear of the encoder's +/-1 count noise. The window
// has to outlast breakaway and any stick-slip pause on the way.
const uint8_t  CAL_MOVE_COUNTS = 4;
const uint16_t CAL_STALL_MS    = 4000;

// After a stop is found: rest, push once more for CAL_PUSH_MS to prove it,
// rest again before reversing.
const uint16_t CAL_SETTLE_MS = 1000;
const uint16_t CAL_PUSH_MS   = 3000;

// One leg of a full sweep can take about 5 minutes at full duty, so this is
// generous. Exceeding it faults the axis.
const unsigned long CAL_SWEEP_TIMEOUT_MS = 420000UL;

// Save to EEPROM automatically on success. Nobody may be at a keyboard after
// a roof install, and a calibration that fails validation saves nothing.
#define CAL_AUTOSAVE 1

// ===========================================================================
//  7. SAFETY: STALL, TIMEOUTS, I2C, WATCHDOG
// ===========================================================================
//  Reversing a loaded DC motor instantly is the worst moment the supply and
//  the bridge will ever see. Coast between directions.
const uint16_t REVERSE_DEAD_MS = 200;

// STALL CHECK, from the encoder, while the motor is commanded on. If the dish
// moves fewer than STALL_MIN_COUNTS in STALL_WINDOW_MS, it is not going
// anywhere: a jam, a target past the cam, or a cam that has cut its current.
//   moves ('g', '+', '-')  -> fault F_STALL
//   panel buttons, jogs    -> stop quietly; holding a button into the end of
//                             travel is normal, not a fault
// The window has to outlast breakaway from cold grease.
const uint16_t STALL_WINDOW_MS = 5000;
const uint8_t  STALL_MIN_COUNTS = 4;

// Absolute ceiling on any one stretch of motion. The stall check is what
// normally ends a stuck move; this is only for a move that keeps creeping
// forever. It is long because a full-range move at this actuator's speed is
// minutes, not seconds.
const unsigned long MAX_RUN_MS = 420000UL;

// I2C. A stuck bus (motor noise, a loose SDA on the roof) makes the default
// Wire library wait forever. The loop then freezes with the motor still
// driving at its last duty, deaf to kill and serial. With a timeout, a stuck
// read gives up and counts as an ordinary encoder failure, which faults and
// stops the motor. Needs an AVR core that has it (1.8.3+); ignored otherwise.
const uint16_t I2C_TIMEOUT_US = 3000;

// WATCHDOG: resets the chip if loop() stops running for this long. Anything
// that still manages to hang the sketch then ends in a reboot, in which
// setup() drives the bridge low as its first act. Needs the Optiboot
// bootloader that Uno-type boards ship with. If the board ever sits in a
// reboot loop, set USE_WATCHDOG to 0 and reflash.
#define WDT_TIMEOUT_CODE WDTO_1S

// Driver current sense (USE_CURRENT_LIMIT): the G2's CS is roughly 20 mV per
// amp into a 5 V ADC; the real scale belongs here once measured.
const uint16_t CURRENT_TRIP_ADC = 700;   // PLACEHOLDER raw ADC counts

// ===========================================================================
//  8. CONSOLE AND PERSISTENCE
// ===========================================================================
//  While moving, the console prints one short line per TELEMETRY_MS. Set
//  TELEMETRY_VERBOSE to 1 for the full multi-line status block instead (a
//  long print blocks the loop on the 64-byte serial buffer). 's' always
//  prints the full block. Calibration, which runs for minutes, prints at its
//  own slower rate.
#define TELEMETRY_VERBOSE 0
const uint16_t TELEMETRY_MS     = 250;
const uint16_t CAL_TELEMETRY_MS = 2000;

// Only the encoder zero and the gains are saved. Position is never saved --
// an absolute sensor makes the question meaningless.
const int EEPROM_BASE_ADDR = 64;
const uint32_t EEPROM_MAGIC = 0x4D4F4E44UL;   // "MOND"

// ===========================================================================
//  9. RETAINED BUT DISABLED HARDWARE
// ===========================================================================
//  Everything below belongs to a feature switched off in section 2. The code
//  is still in the sketch under #if, and may be re-enabled in the future; the
//  constants stay here so it can be.

// --- reed switch (USE_REED) ------------------------------------------------
//  No longer a position sensor. It witnesses:
//    reed silent, motor commanded on   -> stall, cam cut, or blown fuse
//    reed pulsing, encoder not moving  -> the LINKAGE has broken and the motor
//                                         is turning nothing
//  Not reliable on this build, so off; the encoder stall check above replaces
//  the first, and nothing replaces the second.
const uint8_t       PIN_REED         = 2;     // INT0
const unsigned long REED_DEBOUNCE_US = 3000;
const uint16_t STALL_TIMEOUT_MS = 2000;       // commanded on, no pulse this long
const uint16_t START_GRACE_MS   = 1500;       // breakaway grace before the above
const uint8_t  LINKAGE_PULSES   = 150;        // pulses with the boom still...
const float    LINKAGE_MIN_DEG  = 1.0f;       // ...under this many degrees

// --- limit switches (USE_LIMIT_SWITCHES) -------------------------------------
//  Signal only: they cut no motor current, so driveSigned() refuses to drive
//  into a tripped one. POS is the switch reached while EXTENDING. LIMITS_
//  NORMALLY_CLOSED 1 is for COM-NC to GND, so a broken wire reads as TRIPPED;
//  0 is for COM-NO, where a broken wire silently removes that limit.
#define LIMITS_NORMALLY_CLOSED 0
const uint8_t PIN_LIM_POS = A0;
const uint8_t PIN_LIM_NEG = A1;

// Software glitch filter on the limit and ref switches. Deliberately lopsided:
// a trip must hold LIMIT_TRIP_MS, but a tripped limit must read clear for
// LIMIT_CLEAR_MS before it is believed.
const uint8_t LIMIT_TRIP_MS  = 2;
const uint8_t LIMIT_CLEAR_MS = 20;
const uint8_t REF_STABLE_MS  = 2;

// --- reference switch (USE_REF_SWITCH) ---------------------------------------
//  Mid-travel on A2, NO to GND, not a hard stop. 'h' crosses it and reports
//  drift; 'H' crosses it and adopts the crossing as zero, then 'w' saves. Set
//  REF_ANGLE_DEG to where it actually sits before trusting 'H'.
const uint8_t PIN_REF = A2;
const float REF_ANGLE_DEG  = -5.0f;     // PLACEHOLDER
const float REF_MARGIN_DEG =  1.5f;     // how far clear of it a pass starts
const uint8_t  REF_CROSS_SPEED = 70;    // must still be above breakaway
const uint16_t REF_TIMEOUT_MS  = 60000;
// Raw counts at the entering edge, per direction. -1 = never adopted.
const int16_t REF_RAW_POS_DEFAULT = -1;
const int16_t REF_RAW_NEG_DEFAULT = -1;
const float REF_DRIFT_WARN_DEG = 0.5f;  // PLACEHOLDER

// --- step test (USE_STEP_TEST) ------------------------------------------------
const uint16_t STEP_TEST_MS   = 6000;
const uint16_t STEP_SAMPLE_MS = 40;
