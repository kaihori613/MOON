#pragma once
// ===========================================================================
//  Buzzer.h  --  audible state, for when you are at the dish and not the PC
// ===========================================================================
//
//  The calibration sweep has you standing at an 8 ft dish reading a compass
//  while the laptop is indoors. Everything this file does could be read off
//  the serial console instead, if you were in front of it. You are not.
//
//  Idea borrowed from SARCnet's rotator, which beeps whenever a new
//  calibration extreme is captured so you can rotate the antenna by hand and
//  hear the calibration working. Same trick here, including the cheat of
//  driving an adjacent pin permanently LOW so an active buzzer can plug
//  straight into two neighbouring headers with no flying ground lead.
//
//  ACTIVE buzzer, not passive: it makes its own tone from DC, so this is a
//  digitalWrite and never a timer. That matters -- Timer0 is millis(), Timer1
//  and Timer2 are motor PWM, and there is no spare timer to hand a tone() to.
//
//  Nothing here blocks. The player is a small state machine ticked from
//  loop(); a beep during a move must never delay a stall check.

#include <Arduino.h>
#include "Config.h"

#if USE_BUZZER

// Patterns are a count of on/off pairs plus the two durations. Deliberately
// few and deliberately distinguishable across a yard: you have to tell them
// apart by ear, from a distance, without counting carefully.
enum BeepPattern : uint8_t {
  BEEP_NONE = 0,
  BEEP_TICK,     // one short blip   -- a calibration sample landed
  BEEP_DONE,     // two short        -- move finished on target
  BEEP_HOME,     // three short      -- homed, origin established
  BEEP_FAULT     // one long         -- stall, limit, anything gone wrong
};

static uint8_t       g_beepsLeft  = 0;
static uint16_t      g_beepOnMs   = 0;
static uint16_t      g_beepOffMs  = 0;
static bool          g_beepHigh   = false;
static unsigned long g_beepNextAt = 0;

void buzzerBegin() {
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
#if BUZZER_GROUND_PIN >= 0
  // A convenient ground next door, so the buzzer is a two-pin part.
  pinMode((uint8_t)BUZZER_GROUND_PIN, OUTPUT);
  digitalWrite((uint8_t)BUZZER_GROUND_PIN, LOW);
#endif
}

void beep(BeepPattern p) {
  switch (p) {
    case BEEP_TICK:  g_beepsLeft = 1; g_beepOnMs = 40;  g_beepOffMs = 60;  break;
    case BEEP_DONE:  g_beepsLeft = 2; g_beepOnMs = 70;  g_beepOffMs = 90;  break;
    case BEEP_HOME:  g_beepsLeft = 3; g_beepOnMs = 70;  g_beepOffMs = 90;  break;
    case BEEP_FAULT: g_beepsLeft = 1; g_beepOnMs = 600; g_beepOffMs = 0;   break;
    default:         g_beepsLeft = 0; break;
  }
  if (g_beepsLeft) {
    digitalWrite(PIN_BUZZER, HIGH);
    g_beepHigh   = true;
    g_beepNextAt = millis() + g_beepOnMs;
  }
}

void buzzerUpdate(unsigned long now) {
  if (!g_beepsLeft) return;
  if ((long)(now - g_beepNextAt) < 0) return;

  if (g_beepHigh) {
    digitalWrite(PIN_BUZZER, LOW);
    g_beepHigh = false;
    g_beepsLeft--;
    g_beepNextAt = now + g_beepOffMs;
  } else {
    digitalWrite(PIN_BUZZER, HIGH);
    g_beepHigh   = true;
    g_beepNextAt = now + g_beepOnMs;
  }
}

// Kill any pattern in flight. Used by the panic stop: if something has gone
// wrong the last thing wanted is a cheerful two-tone finishing its sequence.
void buzzerSilence() {
  g_beepsLeft = 0;
  g_beepHigh  = false;
  digitalWrite(PIN_BUZZER, LOW);
}

#else   // !USE_BUZZER

enum BeepPattern : uint8_t { BEEP_NONE = 0, BEEP_TICK, BEEP_DONE, BEEP_HOME, BEEP_FAULT };
inline void buzzerBegin() {}
inline void beep(BeepPattern) {}
inline void buzzerUpdate(unsigned long) {}
inline void buzzerSilence() {}

#endif  // USE_BUZZER
