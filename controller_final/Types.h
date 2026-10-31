#pragma once
#include "Config.h"

// The types live here, not in the .ino, because the Arduino builder writes a
// prototype for every function in the sketch and puts them all above the
// first function body -- which is above any enum or struct declared further
// down. A function that takes a Fault then fails with "'Fault' was not
// declared". A header is included before those prototypes.

// ---------------------------------------------------------------------------
//  PID
// ---------------------------------------------------------------------------

struct PidState {
  float integral;
  float prevMeas;
  bool  primed;
  bool  holdIntegral;   // set when the output saturated last tick
};

// ---------------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------------

// ST_STEPTEST and ST_REFPASS only run when USE_STEP_TEST / USE_REF_SWITCH are
// set in Config.h. They stay in the enum either way so the switches below can
// list every state and the compiler can check them.
enum State : uint8_t {
  ST_IDLE, ST_MOVING, ST_JOG, ST_MANUAL, ST_STEPTEST, ST_REFPASS, ST_CALIB, ST_FAULT
};

enum Fault : uint8_t {
  F_NONE = 0, F_STALL, F_LINKAGE, F_ENCODER, F_LIMIT, F_TIMEOUT, F_CURRENT, F_RANGE, F_REF,
  F_KILL, F_CAL
};

// Who last commanded the axis.
enum Owner : uint8_t { OWN_LOCAL, OWN_REMOTE };

// reference switch (USE_REF_SWITCH)
enum RefPhase : uint8_t { RP_GOTO_START, RP_CROSS_POS, RP_GOTO_END, RP_CROSS_NEG };

// ---------------------------------------------------------------------------
//  Auto-calibration
// ---------------------------------------------------------------------------
//  Each leg (extend, then retract) runs SWEEP until the encoder stops moving,
//  rests with the motor off, then PUSHes once more to prove it really is the
//  end stop and not a stick-slip pause. Only a leg that survives the push is
//  believed. REST is a coast before the direction changes.
enum CalPhase : uint8_t { CAL_SWEEP, CAL_SETTLE, CAL_PUSH, CAL_REST };

struct CalState {
  CalPhase      phase;
  uint8_t       leg;          // 0 = extending, 1 = retracting
  unsigned long legStart;     // for the per-leg timeout
  unsigned long phaseStart;
  unsigned long markMs;       // when the encoder last moved CAL_MOVE_COUNTS
  uint16_t      markRaw;      // ...and where it was then
  uint16_t      lastRaw;      // previous tick, for the travel accumulator
  uint16_t      rawA;         // raw count at the extend stop
  int32_t       travel;       // signed raw counts, unwrapped, since the leg began
};

// ---------------------------------------------------------------------------
//  EEPROM
// ---------------------------------------------------------------------------

#if USE_EEPROM
struct Saved {
  uint32_t magic;
  int16_t  zero;
  int16_t  refPos;
  int16_t  refNeg;
  float    kp, ki, kd, kff;
};
#endif

// ---------------------------------------------------------------------------
//  Panel buttons
// ---------------------------------------------------------------------------

#if USE_BUTTONS
struct Button {
  uint8_t  pin;
  bool     stable;        // debounced state, true = pressed
  bool     last;          // last raw read
  unsigned long changed;  // when the raw read last differed
};
#endif
