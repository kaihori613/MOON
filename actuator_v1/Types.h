#pragma once
#include "Config.h"

// The types live here, not in the .ino, because the Arduino builder writes a
// prototype for every function in the sketch and puts them all above the
// first function body -- which is above any enum or struct declared further
// down. A function that takes a Fault then fails with "'Fault' was not
// declared". A header is included before those prototypes.

// ---------------------------------------------------------------------------
//  Machine state
// ---------------------------------------------------------------------------

enum State : uint8_t {
  ST_IDLE,        // motor off, nothing pending
  ST_JOG,         // open loop, until time runs out or something stops it
  ST_SEEK,        // closed loop, toward g_target
  ST_HOME,        // retracting into the near stop to establish zero
  ST_MEASURE,     // extending into the far stop to learn the travel
  ST_MANUAL,      // a panel button is held down
  ST_FAULT        // motor off, needs 'k'
};

enum Fault : uint8_t {
  FAULT_NONE,
  FAULT_STALL,      // pulses stopped mid-move: a jam, or an unexpected end
  FAULT_TIMEOUT,    // ran past MAX_RUN_MS with nothing else stopping it
  FAULT_NOT_HOMED,  // absolute move asked for before there was an origin
  FAULT_KILLED      // kill switch thrown -- latched until cleared
};

// ---------------------------------------------------------------------------
//  Persistence
// ---------------------------------------------------------------------------

#if USE_EEPROM
struct SavedState {
  uint16_t magic;
  long     position;
  long     travel;
  float    degPerCount;
  float    degAtRetract;
  uint16_t sum;
};
#endif
