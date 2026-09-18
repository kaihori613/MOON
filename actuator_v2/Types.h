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

enum State : uint8_t { ST_IDLE, ST_MOVING, ST_JOG, ST_MANUAL, ST_STEPTEST, ST_REFPASS, ST_FAULT };

enum Fault : uint8_t {
  F_NONE = 0, F_STALL, F_LINKAGE, F_ENCODER, F_LIMIT, F_TIMEOUT, F_CURRENT, F_RANGE, F_REF,
  F_KILL
};

// Who last commanded the axis.
enum Owner : uint8_t { OWN_LOCAL, OWN_REMOTE };

// reference switch
enum RefPhase : uint8_t { RP_GOTO_START, RP_CROSS_POS, RP_GOTO_END, RP_CROSS_NEG };

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
