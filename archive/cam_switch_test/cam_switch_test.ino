/*
  Cam Switch Test  --  do the mechanical end stops fire in the same place?
  ------------------------------------------------------------------------
  One question only: when the actuator runs into either end of its travel, does
  the cam limit switch cut the motor at the SAME count every time?

  Run this after reed_switch_test passes. It assumes the sensor is already
  trustworthy -- if it is not, every total below is wrong in a way this sketch
  cannot see.

  WHY IT MATTERS
    Homing works by driving into the retract stop and calling that spot zero.
    Every absolute position afterwards is measured from it. If that stop lands
    somewhere slightly different each time, then zero moves, and the whole
    coordinate system moves with it. A dish that was peaked yesterday points
    somewhere else today, and nothing in the system can tell you why.

  THE THING THIS SKETCH CANNOT SEE
    The cam switches are not wired to the Arduino -- they cut motor current
    inside the actuator. So "the cam switch fired" is inferred from pulses
    stopping. A sensor that quits while the motor keeps running looks exactly
    the same from here, and means something completely different.

    That is why the sketch asks you to confirm the supply current after every
    stroke. You are the missing sensor. Answer honestly: a run you mark as a
    real cam stop when it was actually a sensor dropout will quietly poison the
    repeatability figure, which is the one number this whole sketch exists to
    produce.

  WIRING
    Reed switch one leg -> D2, other leg -> GND. Nothing else. The motor still
    runs from the bench supply; this sketch does not drive it.

  PROCEDURE
    1. Type  e  to declare the next stroke is EXTEND.
    2. Run the actuator to full extension and leave the supply ON.
    3. When pulses stop, the run ends automatically and asks about the supply
       current. Look at the supply's ammeter and answer y or n.
    4. Type  r  and run it back to full retraction. Answer again.
    5. Repeat for at least 3 strokes each way, then  s  for the report.
*/

const uint8_t REED_PIN = 2;

// Should match REED_DEBOUNCE_US as settled by reed_switch_test.
const unsigned long DEBOUNCE_US = 3000;

// A stroke is over once pulses stop for this long. Must be comfortably longer
// than the slowest gap at the slowest travel speed, or a slow stroke will be
// chopped into several "runs".
const unsigned long RUN_IDLE_MS = 1500;

// ---------------------------------------------------------------------------

volatile unsigned long g_pulses = 0;
volatile unsigned long g_lastPulseUs = 0;

void onPulse() {
  const unsigned long now = micros();
  if (now - g_lastPulseUs < DEBOUNCE_US) return;
  g_lastPulseUs = now;
  g_pulses++;
}

// ---------------------------------------------------------------------------
//  Per-end statistics
// ---------------------------------------------------------------------------
//  The reed switch cannot tell direction, so it cannot know which end it just
//  hit. You declare it. The two ends are different physical switches and there
//  is no reason to assume they are equally repeatable, so they are kept apart.

enum End : uint8_t { END_EXTEND = 0, END_RETRACT = 1, END_NONE = 2 };

const uint8_t MAX_STROKES = 8;
unsigned long g_totals[2][MAX_STROKES];
uint8_t       g_count[2] = {0, 0};

const char* endName(uint8_t e) {
  return e == END_EXTEND ? "EXTEND" : "RETRACT";
}

void recordStroke(uint8_t which, unsigned long total) {
  if (g_count[which] == MAX_STROKES) {
    for (uint8_t i = 0; i < MAX_STROKES - 1; i++) {
      g_totals[which][i] = g_totals[which][i + 1];
    }
    g_count[which]--;
  }
  g_totals[which][g_count[which]++] = total;
}

bool endStats(uint8_t which, unsigned long* lo, unsigned long* hi, float* mean) {
  const uint8_t n = g_count[which];
  if (n == 0) return false;

  unsigned long minv = g_totals[which][0];
  unsigned long maxv = g_totals[which][0];
  unsigned long sum  = 0;

  for (uint8_t i = 0; i < n; i++) {
    const unsigned long v = g_totals[which][i];
    if (v < minv) minv = v;
    if (v > maxv) maxv = v;
    sum += v;
  }

  if (lo)   *lo = minv;
  if (hi)   *hi = maxv;
  if (mean) *mean = (float)sum / n;
  return true;
}

// ---------------------------------------------------------------------------
//  Run state
// ---------------------------------------------------------------------------

uint8_t       g_declared    = END_NONE;   // which end the current stroke ends at
bool          g_runActive   = false;
unsigned long g_lastSeenMs  = 0;
unsigned long g_printedTo   = 0;

// After a run ends we stop and wait for the supply-current answer, because
// recording the stroke before knowing what stopped it is how a sensor dropout
// gets filed as a good cam stop.
bool          g_awaitingConfirm = false;
unsigned long g_pendingTotal    = 0;

float g_strokeMm = 0.0;    // measured with a tape, entered with 'm'

// ---------------------------------------------------------------------------

void armFor(uint8_t which) {
  noInterrupts();
  g_pulses = 0;
  interrupts();

  g_declared   = which;
  g_runActive  = false;
  g_printedTo  = 0;
  g_awaitingConfirm = false;

  Serial.println();
  Serial.print(F(">>> armed for a ")); Serial.print(endName(which));
  Serial.println(F(" stroke."));
  Serial.println(F("    Run the actuator into that end and leave the supply ON."));
}

void endRun(unsigned long total) {
  g_runActive = false;

  Serial.println();
  Serial.print(F("--- stroke finished: ")); Serial.print(total);
  Serial.print(F(" pulses to the ")); Serial.print(endName(g_declared));
  Serial.println(F(" end"));

  if (total < 3) {
    Serial.println(F("    Too few pulses to be a full stroke -- discarded."));
    armFor(g_declared);
    return;
  }

  g_pendingTotal    = total;
  g_awaitingConfirm = true;

  Serial.println();
  Serial.println(F("    Look at the supply's current display NOW."));
  Serial.println(F("    Did the current drop to near zero?    y / n"));
  Serial.println(F("      y = the cam switch cut the motor. That is the switch"));
  Serial.println(F("          working, and the stroke counts."));
  Serial.println(F("      n = current is still flowing, so the motor is running"));
  Serial.println(F("          and the SENSOR stopped counting instead. Different"));
  Serial.println(F("          fault entirely, and the stroke is discarded."));
}

void resolveConfirm(bool wasCamStop) {
  g_awaitingConfirm = false;

  if (!wasCamStop) {
    Serial.println();
    Serial.println(F("    DISCARDED -- recorded as a sensor dropout, not a cam"));
    Serial.println(F("    stop. Something interrupted the pulse train while the"));
    Serial.println(F("    motor was still turning: a marginal reed contact, a"));
    Serial.println(F("    connector, or interference. Go back to"));
    Serial.println(F("    reed_switch_test before trusting anything here."));
    armFor(g_declared);
    return;
  }

  recordStroke(g_declared, g_pendingTotal);
  Serial.print(F("    recorded. "));
  Serial.print(g_count[g_declared]);
  Serial.print(F(" stroke(s) stored for the "));
  Serial.print(endName(g_declared));
  Serial.println(F(" end."));

  // Arm the opposite end automatically -- that is the next stroke in practice,
  // and it saves reaching for the keyboard mid-procedure.
  armFor(g_declared == END_EXTEND ? END_RETRACT : END_EXTEND);
}

// ---------------------------------------------------------------------------

void printEndReport(uint8_t which) {
  unsigned long lo, hi;
  float mean;

  Serial.print(F("  "));
  Serial.print(endName(which));
  Serial.print(which == END_EXTEND ? F("  end : ") : F(" end : "));

  if (!endStats(which, &lo, &hi, &mean)) {
    Serial.println(F("no strokes recorded"));
    return;
  }

  const unsigned long spread = hi - lo;

  Serial.print(F("n=")); Serial.print(g_count[which]);
  Serial.print(F("  min=")); Serial.print(lo);
  Serial.print(F("  max=")); Serial.print(hi);
  Serial.print(F("  mean=")); Serial.print(mean, 1);
  Serial.print(F("  spread=")); Serial.print(spread);

  if (g_count[which] < 3) {
    Serial.println(F("   (fewer than 3 strokes -- not yet conclusive)"));
  } else if (spread <= 2) {
    Serial.println(F("   REPEATABLE"));
  } else if (spread <= 5) {
    Serial.println(F("   marginal"));
  } else {
    Serial.println(F("   TOO WIDE"));
  }

  Serial.print(F("             strokes: "));
  for (uint8_t i = 0; i < g_count[which]; i++) {
    if (i) Serial.print(F(", "));
    Serial.print(g_totals[which][i]);
  }
  Serial.println();
}

void printReport() {
  Serial.println();
  Serial.println(F("--- cam switch report ----------------------------------"));

  printEndReport(END_EXTEND);
  printEndReport(END_RETRACT);

  float meanExt = 0, meanRet = 0;
  const bool haveExt = endStats(END_EXTEND, nullptr, nullptr, &meanExt);
  const bool haveRet = endStats(END_RETRACT, nullptr, nullptr, &meanRet);

  // Cross-check. Both ends measure the same physical distance, just travelled
  // in opposite directions, so the two means must agree. If they do not, the
  // sensor is losing counts in one direction -- which no amount of end-stop
  // repeatability will save you from, because position is integrated from
  // those same counts.
  if (haveExt && haveRet) {
    const float diff = meanExt > meanRet ? meanExt - meanRet : meanRet - meanExt;
    Serial.println();
    Serial.print(F("  direction cross-check: extend "));
    Serial.print(meanExt, 1);
    Serial.print(F(" vs retract "));
    Serial.print(meanRet, 1);
    Serial.print(F("  ->  "));

    if (diff <= 2.0) {
      Serial.println(F("agree"));
    } else {
      Serial.println(F("DISAGREE"));
      Serial.println(F("  The same physical travel is measuring differently in"));
      Serial.println(F("  the two directions, so counts are being lost one way."));
      Serial.println(F("  End-stop repeatability cannot rescue that: position is"));
      Serial.println(F("  integrated from these counts, so the error accumulates"));
      Serial.println(F("  with every reversal."));
    }
  }

  // --- millimetres per count ---------------------------------------------
  if (g_strokeMm > 0.0 && (haveExt || haveRet)) {
    float mean;
    if (haveExt && haveRet)      mean = (meanExt + meanRet) / 2.0;
    else if (haveExt)            mean = meanExt;
    else                         mean = meanRet;

    Serial.println();
    Serial.print(F("  stroke length : ")); Serial.print(g_strokeMm, 1);
    Serial.println(F(" mm (entered)"));
    Serial.print(F("  mm per count  : ")); Serial.println(g_strokeMm / mean, 4);
    Serial.println(F("  That is the resolution floor for the whole system. Feed"));
    Serial.println(F("  it into host/geometry.py with the linkage arm lengths to"));
    Serial.println(F("  get degrees per count, which is what decides whether yaw"));
    Serial.println(F("  can point accurately enough."));
  } else if (haveExt || haveRet) {
    Serial.println();
    Serial.println(F("  No stroke length entered. Measure the mount-to-mount"));
    Serial.println(F("  distance retracted and extended, then type  m <mm>  with"));
    Serial.println(F("  the difference to get millimetres per count."));
  }

  Serial.println(F("--------------------------------------------------------"));
  Serial.println();
}

// ---------------------------------------------------------------------------

void printHelp() {
  Serial.println(F("--- commands -------------------------------------------"));
  Serial.println(F("  ?        this help"));
  Serial.println(F("  e        arm: the next stroke ends at the EXTEND stop"));
  Serial.println(F("  r        arm: the next stroke ends at the RETRACT stop"));
  Serial.println(F("  y / n    answer the supply-current question after a stroke"));
  Serial.println(F("  s        report"));
  Serial.println(F("  m <mm>   enter the measured stroke length in millimetres"));
  Serial.println(F("  z        discard all recorded strokes"));
  Serial.println(F("--------------------------------------------------------"));
}

void handleCommand(char* line) {
  if (line[0] == '\0') return;

  const char cmd = tolower(line[0]);
  const char* arg = line + 1;
  while (*arg == ' ' || *arg == '\t') arg++;
  const bool hasArg = (*arg != '\0');

  // While a stroke is awaiting confirmation, y and n are the only things that
  // make sense -- anything else would file the run under a guess.
  if (g_awaitingConfirm) {
    if (cmd == 'y') { resolveConfirm(true);  return; }
    if (cmd == 'n') { resolveConfirm(false); return; }
    Serial.println(F("    waiting on the supply-current answer: y or n"));
    return;
  }

  switch (cmd) {
    case '?': printHelp(); break;
    case 's': printReport(); break;
    case 'e': armFor(END_EXTEND); break;
    case 'r': armFor(END_RETRACT); break;

    case 'm':
      if (!hasArg) {
        Serial.print(F("stroke length = "));
        Serial.print(g_strokeMm, 1);
        Serial.println(F(" mm"));
        break;
      }
      g_strokeMm = atof(arg);
      Serial.print(F("stroke length = ")); Serial.print(g_strokeMm, 1);
      Serial.println(F(" mm"));
      break;

    case 'z':
      g_count[0] = g_count[1] = 0;
      Serial.println(F("all recorded strokes discarded"));
      break;

    case 'y':
    case 'n':
      Serial.println(F("nothing to confirm -- run a stroke first"));
      break;

    default:
      Serial.println(F("unknown command -- ? for help"));
      break;
  }
}

char    g_line[24];
uint8_t g_len = 0;

void pollSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      g_line[g_len] = '\0';
      handleCommand(g_line);
      g_len = 0;
    } else if (g_len < sizeof(g_line) - 1) {
      g_line[g_len++] = c;
    }
  }
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(REED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REED_PIN), onPulse, FALLING);

  Serial.println();
  Serial.println(F("=== Cam Switch Test ==="));
  Serial.println(F("Measures whether the end stops fire at the same count every"));
  Serial.println(F("time. Run reed_switch_test first -- this sketch assumes the"));
  Serial.println(F("sensor is already trustworthy."));
  Serial.println();
  Serial.println(F("The motor still runs from the bench supply. Leave the supply"));
  Serial.println(F("ON when the actuator reaches an end, so you can watch the"));
  Serial.println(F("current drop when the cam switch cuts it."));
  Serial.println();
  printHelp();

  armFor(END_EXTEND);
}

void loop() {
  pollSerial();

  if (g_awaitingConfirm) return;    // hold everything until the run is filed

  noInterrupts();
  const unsigned long count = g_pulses;
  interrupts();

  if (count != g_printedTo) {
    g_printedTo  = count;
    g_lastSeenMs = millis();
    g_runActive  = true;

    Serial.print(F("  pulse "));
    Serial.print(count);
    Serial.print(F("   -> "));
    Serial.println(endName(g_declared));
  }

  if (g_runActive && millis() - g_lastSeenMs > RUN_IDLE_MS) {
    endRun(count);
  }
}
