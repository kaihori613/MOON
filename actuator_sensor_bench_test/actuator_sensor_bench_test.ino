/*
  Actuator Sensor Bench Test  --  the combined acceptance test
  -------------------------------------------------------------
  Everything reed_switch_test and cam_switch_test measure, in one session, plus
  the numbers those two cannot produce on their own: millimetres per count, the
  resolution that follows from it, and a paste-ready block for
  ActuatorConfig.h.

  This is the gate. Its job is to answer one question -- is the sensor good
  enough to close a position loop around? -- and to answer it out loud, with a
  PASS or FAIL per criterion, rather than leaving you to eyeball a wall of
  numbers and talk yourself into it.

  WHICH SKETCH TO RUN
    reed_switch_test/     Is the sensor clean? Gap histogram, bounce
                          detection, live debounce tuning. Diagnostic detail
                          this sketch does not carry.
    cam_switch_test/      Do the end stops repeat? Per-end statistics.
    this one              Both, condensed, plus the config block and the gate.

    The focused sketches exist for when something fails here and you need to
    find out why. Start here; drop to those when this reports a problem.

  WHAT THIS STILL DOES NOT DO
    Drive the motor. Keep using the bench supply. The Arduino takes over in
    actuator_system, and only after this reports a clean pass.

  WIRING
    Reed switch one leg -> D2, other leg -> GND. Nothing else.

  PROCEDURE
    1.  f      noise floor, actuator STOPPED. Must come back zero.
    2.  e      arm for an extend stroke; run it, answer the current question.
    3.         it arms the other end automatically -- run it back.
    4.         repeat until at least 3 strokes each way.
    5.  m <mm> the measured stroke length (extended minus retracted,
               mount-to-mount, with a tape).
    6.  a <mm> distance from the yaw pivot to the actuator's moving mount.
    7.  s      the report, the config block, and the gate.
*/

const uint8_t REED_PIN = 2;

// Start from whatever reed_switch_test settled on. Adjustable here with 'd'.
unsigned long g_debounceUs = 3000;

// A stroke ends once pulses stop for this long -- which is what happens when a
// cam limit switch cuts motor current. Must be comfortably longer than the
// slowest gap at the slowest speed you will run.
const unsigned long RUN_IDLE_MS = 1500;

// ---------------------------------------------------------------------------
//  Sensor
// ---------------------------------------------------------------------------
//  Debounce happens in the ISR here, unlike reed_switch_test, which defers it
//  so it can show you the rejects in detail. This sketch only needs the
//  headline count of them.

volatile unsigned long g_pulses     = 0;
volatile unsigned long g_edges      = 0;   // before debounce
volatile unsigned long g_rejects    = 0;
volatile unsigned long g_lastEdgeUs = 0;
volatile unsigned long g_minGapUs   = 0xFFFFFFFFUL;
volatile unsigned long g_maxGapUs   = 0;

void onEdge() {
  const unsigned long now = micros();
  const unsigned long gap = now - g_lastEdgeUs;
  g_lastEdgeUs = now;
  g_edges++;

  if (gap < g_debounceUs) {
    g_rejects++;
    return;
  }

  // The first pulse of a stroke has no meaningful gap -- it is measured from
  // whenever the previous stroke ended, which is however long you took to
  // reach for the supply switch.
  if (g_pulses > 0) {
    if (gap < g_minGapUs) g_minGapUs = gap;
    if (gap > g_maxGapUs) g_maxGapUs = gap;
  }
  g_pulses++;
}

void zeroStroke() {
  noInterrupts();
  g_pulses = 0;
  interrupts();
}

void zeroSensorStats() {
  noInterrupts();
  g_pulses   = 0;
  g_edges    = 0;
  g_rejects  = 0;
  g_minGapUs = 0xFFFFFFFFUL;
  g_maxGapUs = 0;
  interrupts();
}

// ---------------------------------------------------------------------------
//  Per-end statistics
// ---------------------------------------------------------------------------

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

unsigned long spreadOf(uint8_t which) {
  unsigned long lo, hi;
  if (!endStats(which, &lo, &hi, nullptr)) return 0xFFFFFFFFUL;
  return hi - lo;
}

// ---------------------------------------------------------------------------
//  Session state
// ---------------------------------------------------------------------------

uint8_t       g_declared   = END_NONE;
bool          g_runActive  = false;
unsigned long g_lastSeenMs = 0;
unsigned long g_printedTo  = 0;

bool          g_awaitingConfirm = false;
unsigned long g_pendingTotal    = 0;

float g_strokeMm = 0.0;    // 'm' -- measured stroke, mount to mount
float g_armMm    = 0.0;    // 'a' -- yaw pivot to the actuator's moving mount

bool g_noiseTested = false;
unsigned long g_noiseEdges = 0;

// ---------------------------------------------------------------------------

void armFor(uint8_t which) {
  zeroStroke();
  g_declared   = which;
  g_runActive  = false;
  g_printedTo  = 0;
  g_awaitingConfirm = false;

  Serial.println();
  Serial.print(F(">>> armed for a ")); Serial.print(endName(which));
  Serial.println(F(" stroke -- run it, and leave the supply ON at the end."));
}

void endRun(unsigned long total) {
  g_runActive = false;

  Serial.println();
  Serial.print(F("--- stroke finished: ")); Serial.print(total);
  Serial.print(F(" pulses to the ")); Serial.print(endName(g_declared));
  Serial.println(F(" end"));

  if (total < 3) {
    Serial.println(F("    Too short to be a full stroke -- discarded."));
    armFor(g_declared);
    return;
  }

  g_pendingTotal    = total;
  g_awaitingConfirm = true;

  Serial.println();
  Serial.println(F("    Check the supply's current display NOW."));
  Serial.println(F("    Did the current drop to near zero?    y / n"));
  Serial.println(F("      y = a cam switch cut the motor. Working as intended."));
  Serial.println(F("      n = current still flowing, so the motor is turning and"));
  Serial.println(F("          the SENSOR stopped counting. Different fault, and"));
  Serial.println(F("          this sketch cannot tell them apart without you."));
}

void resolveConfirm(bool wasCamStop) {
  g_awaitingConfirm = false;

  if (!wasCamStop) {
    Serial.println();
    Serial.println(F("    DISCARDED -- sensor dropout, not a cam stop. The pulse"));
    Serial.println(F("    train broke while the motor was still turning. Go to"));
    Serial.println(F("    reed_switch_test and find out why before continuing;"));
    Serial.println(F("    nothing measured here means anything until it is fixed."));
    armFor(g_declared);
    return;
  }

  recordStroke(g_declared, g_pendingTotal);
  Serial.print(F("    recorded ("));
  Serial.print(g_count[g_declared]);
  Serial.print(F(" stored for the "));
  Serial.print(endName(g_declared));
  Serial.println(F(" end)"));

  armFor(g_declared == END_EXTEND ? END_RETRACT : END_EXTEND);
}

// ---------------------------------------------------------------------------
//  Noise floor
// ---------------------------------------------------------------------------

void noiseFloorTest() {
  Serial.println();
  Serial.println(F("Noise floor: 10 seconds, actuator STOPPED. Leave the motor"));
  Serial.println(F("supply switched on -- an idle bridge still radiates, and"));
  Serial.println(F("that is what you want to measure."));

  // Measure a delta rather than zeroing. Running 'f' after a few strokes is a
  // reasonable thing to want to do -- checking whether a warm bridge is noisier
  // than a cold one, say -- and it must not cost you the gap statistics those
  // strokes produced. Anything the test does pick up is restored away
  // afterwards, since it is not stroke data and would skew the report.
  noInterrupts();
  const unsigned long edgesBefore   = g_edges;
  const unsigned long rejectsBefore = g_rejects;
  const unsigned long minBefore     = g_minGapUs;
  const unsigned long maxBefore     = g_maxGapUs;
  interrupts();

  const unsigned long start = millis();
  while (millis() - start < 10000UL) {
    if ((millis() - start) % 1000 < 20) { Serial.print('.'); delay(25); }
  }
  Serial.println();

  noInterrupts();
  g_noiseEdges = g_edges - edgesBefore;
  g_edges    = edgesBefore;
  g_rejects  = rejectsBefore;
  g_minGapUs = minBefore;
  g_maxGapUs = maxBefore;
  interrupts();
  g_noiseTested = true;

  Serial.print(F("  edges in 10 s : ")); Serial.println(g_noiseEdges);
  if (g_noiseEdges == 0) {
    Serial.println(F("  clean -- no pickup with the actuator stopped."));
  } else {
    Serial.println(F("  PICKUP DETECTED. These become counts the controller"));
    Serial.println(F("  believes. They are indistinguishable from real motion,"));
    Serial.println(F("  so position drifts and nothing downstream can detect it."));
    Serial.println(F("  Shield the sensor run, add an RC filter at the pin, keep"));
    Serial.println(F("  it away from the motor leads."));
  }

  armFor(g_declared == END_NONE ? END_EXTEND : g_declared);
}

// ---------------------------------------------------------------------------
//  Report
// ---------------------------------------------------------------------------

void printCheck(const __FlashStringHelper* label, uint8_t verdict) {
  // verdict: 0 fail, 1 pass, 2 untested
  Serial.print(verdict == 1 ? F("  [PASS] ") : verdict == 0 ? F("  [FAIL] ")
                                                            : F("  [ -- ] "));
  Serial.println(label);
}

void printEndLine(uint8_t which) {
  unsigned long lo, hi;
  float mean;

  Serial.print(F("  "));
  Serial.print(endName(which));
  Serial.print(which == END_EXTEND ? F("  end : ") : F(" end : "));

  if (!endStats(which, &lo, &hi, &mean)) {
    Serial.println(F("no strokes"));
    return;
  }

  Serial.print(F("n=")); Serial.print(g_count[which]);
  Serial.print(F("  min=")); Serial.print(lo);
  Serial.print(F("  max=")); Serial.print(hi);
  Serial.print(F("  mean=")); Serial.print(mean, 1);
  Serial.print(F("  spread=")); Serial.println(hi - lo);
}

void printReport() {
  noInterrupts();
  const unsigned long edges   = g_edges;
  const unsigned long rejects = g_rejects;
  const unsigned long minGap  = g_minGapUs;
  const unsigned long maxGap  = g_maxGapUs;
  interrupts();

  const bool haveGaps = (minGap != 0xFFFFFFFFUL && maxGap > 0);

  Serial.println();
  Serial.println(F("========================================================"));
  Serial.println(F("  ACTUATOR SENSOR BENCH REPORT"));
  Serial.println(F("========================================================"));

  // --- sensor -------------------------------------------------------------
  Serial.println(F("  SENSOR"));
  Serial.print(F("    debounce    : ")); Serial.print(g_debounceUs);
  Serial.println(F(" us"));
  Serial.print(F("    edges       : ")); Serial.println(edges);
  Serial.print(F("    rejected    : ")); Serial.print(rejects);
  if (edges > 0) {
    Serial.print(F("  ("));
    Serial.print((rejects * 100.0) / edges, 1);
    Serial.print(F("%)"));
  }
  Serial.println();

  if (haveGaps) {
    Serial.print(F("    fastest gap : ")); Serial.print(minGap / 1000.0, 2);
    Serial.print(F(" ms  -> ")); Serial.print(1000000.0 / minGap, 2);
    Serial.println(F(" Hz"));
    Serial.print(F("    slowest gap : ")); Serial.print(maxGap / 1000.0, 2);
    Serial.print(F(" ms  -> ")); Serial.print(1000000.0 / maxGap, 2);
    Serial.println(F(" Hz"));
  }

  if (g_noiseTested) {
    Serial.print(F("    noise floor : ")); Serial.print(g_noiseEdges);
    Serial.println(F(" edges in 10 s, stationary"));
  } else {
    Serial.println(F("    noise floor : not tested -- run 'f'"));
  }

  // --- cam switches -------------------------------------------------------
  Serial.println();
  Serial.println(F("  END STOPS"));
  printEndLine(END_EXTEND);
  printEndLine(END_RETRACT);

  float meanExt = 0, meanRet = 0;
  const bool haveExt = endStats(END_EXTEND, nullptr, nullptr, &meanExt);
  const bool haveRet = endStats(END_RETRACT, nullptr, nullptr, &meanRet);
  float dirDiff = 0.0;

  if (haveExt && haveRet) {
    dirDiff = meanExt > meanRet ? meanExt - meanRet : meanRet - meanExt;
    Serial.print(F("  cross-check  : extend "));
    Serial.print(meanExt, 1);
    Serial.print(F(" vs retract "));
    Serial.print(meanRet, 1);
    Serial.print(F("  diff "));
    Serial.println(dirDiff, 1);
  }

  // --- scale --------------------------------------------------------------
  float meanStroke = 0.0;
  if (haveExt && haveRet)  meanStroke = (meanExt + meanRet) / 2.0;
  else if (haveExt)        meanStroke = meanExt;
  else if (haveRet)        meanStroke = meanRet;

  float mmPerCount = 0.0;
  float degPerCount = 0.0;

  if (g_strokeMm > 0.0 && meanStroke > 0.0) {
    mmPerCount = g_strokeMm / meanStroke;
    Serial.println();
    Serial.println(F("  SCALE"));
    Serial.print(F("    stroke      : ")); Serial.print(g_strokeMm, 1);
    Serial.println(F(" mm (measured)"));
    Serial.print(F("    mm/count    : ")); Serial.println(mmPerCount, 4);

    if (g_armMm > 0.0) {
      // Best case only. dTheta/dL = 1 / (arm * sin(angle between the actuator
      // and the arm)), so perpendicular gives the smallest step -- the finest
      // resolution the linkage will ever achieve. Everywhere else it is worse.
      degPerCount = (mmPerCount / g_armMm) * 57.29578;
      Serial.print(F("    arm         : ")); Serial.print(g_armMm, 1);
      Serial.println(F(" mm (pivot to moving mount)"));
      Serial.print(F("    deg/count   : ")); Serial.print(degPerCount, 4);
      Serial.println(F("   BEST CASE"));
      Serial.println(F("    That figure assumes the actuator is perpendicular to"));
      Serial.println(F("    the arm. Anywhere else in the arc it is coarser."));
      Serial.println(F("    host/geometry.py computes the real number for the"));
      Serial.println(F("    actual geometry."));
      Serial.println(F("    This is the floor on yaw pointing accuracy. If it is"));
      Serial.println(F("    too coarse, the fix is a longer arm, not software."));
    } else {
      Serial.println(F("    No arm length entered -- type  a <mm>  (yaw pivot to"));
      Serial.println(F("    the actuator's moving mount) for degrees per count."));
    }
  }

  // --- constants ----------------------------------------------------------
  if (haveGaps) {
    unsigned long suggDebounce = minGap / 4;
    if (suggDebounce > 3000) suggDebounce = 3000;
    unsigned long suggStall = (maxGap / 1000UL) * 3UL;
    if (suggStall < 250) suggStall = 250;

    Serial.println();
    Serial.println(F("  -- copy into actuator_system/ActuatorConfig.h --"));
    Serial.print(F("    REED_DEBOUNCE_US   = ")); Serial.println(suggDebounce);
    Serial.print(F("    STALL_TIMEOUT_MS   = ")); Serial.println(suggStall);

    if (meanStroke > 0.0) {
      Serial.print(F("    SIM_TRAVEL_COUNTS  = "));
      Serial.println((unsigned long)(meanStroke + 0.5));
      Serial.print(F("    SIM_PULSES_PER_SEC = "));
      Serial.println((unsigned int)(1000000.0 / minGap + 0.5));
      Serial.println(F("    (the SIM_ values only make the simulator behave like"));
      Serial.println(F("     the real unit -- worth setting, never load-bearing)"));
    }
  }

  // --- the gate -----------------------------------------------------------
  Serial.println();
  Serial.println(F("  READY TO CLOSE THE LOOP?"));

  const uint8_t cNoise = !g_noiseTested ? 2 : (g_noiseEdges == 0 ? 1 : 0);
  printCheck(F("no electrical pickup when stationary"), cNoise);

  const uint8_t cReject = edges < 20 ? 2 : (rejects * 5 < edges ? 1 : 0);
  printCheck(F("bounce is a small share of all edges"), cReject);

  const uint8_t cMargin = !haveGaps ? 2 : (minGap > g_debounceUs * 3 ? 1 : 0);
  printCheck(F("fastest real gap is well clear of the debounce"), cMargin);

  const uint8_t cExtN = g_count[END_EXTEND] >= 3 ? 1 : 2;
  const uint8_t cRetN = g_count[END_RETRACT] >= 3 ? 1 : 2;
  printCheck(F("at least 3 strokes recorded, each end"),
             (cExtN == 1 && cRetN == 1) ? 1 : 2);

  const uint8_t cExt = g_count[END_EXTEND] < 3 ? 2 : (spreadOf(END_EXTEND) <= 2 ? 1 : 0);
  printCheck(F("extend stop repeats within 2 counts"), cExt);

  const uint8_t cRet = g_count[END_RETRACT] < 3 ? 2 : (spreadOf(END_RETRACT) <= 2 ? 1 : 0);
  printCheck(F("retract stop repeats within 2 counts"), cRet);

  const uint8_t cDir = (haveExt && haveRet) ? (dirDiff <= 2.0 ? 1 : 0) : 2;
  printCheck(F("both directions measure the same travel"), cDir);

  const uint8_t cScale = mmPerCount > 0.0 ? 1 : 2;
  printCheck(F("millimetres per count known"), cScale);

  const uint8_t checks[] = {cNoise, cReject, cMargin, cExt, cRet, cDir, cScale};
  uint8_t fails = 0, untested = 0;
  for (uint8_t i = 0; i < sizeof(checks); i++) {
    if (checks[i] == 0) fails++;
    else if (checks[i] == 2) untested++;
  }

  Serial.println();
  if (fails > 0) {
    Serial.print(F("  NOT READY -- ")); Serial.print(fails);
    Serial.println(F(" check(s) failed."));
    Serial.println(F("  Closing a position loop on a sensor that failed any of"));
    Serial.println(F("  the above buys you a controller that is confidently"));
    Serial.println(F("  wrong. Use reed_switch_test and cam_switch_test to find"));
    Serial.println(F("  out why before moving on."));
  } else if (untested > 0) {
    Serial.print(F("  INCOMPLETE -- ")); Serial.print(untested);
    Serial.println(F(" check(s) not yet measured."));
  } else {
    Serial.println(F("  READY. Copy the constants above into ActuatorConfig.h,"));
    Serial.println(F("  set SIMULATE_ACTUATOR to 0, pull the pin 6 jumper, and"));
    Serial.println(F("  move to actuator_system."));
  }
  Serial.println(F("========================================================"));
  Serial.println();
}

// ---------------------------------------------------------------------------

void printHelp() {
  Serial.println(F("--- commands -------------------------------------------"));
  Serial.println(F("  ?        this help"));
  Serial.println(F("  f        noise floor test -- actuator must be STOPPED"));
  Serial.println(F("  e / r    arm for a stroke ending at EXTEND / RETRACT"));
  Serial.println(F("  y / n    answer the supply-current question"));
  Serial.println(F("  m <mm>   measured stroke length, mount to mount"));
  Serial.println(F("  a <mm>   yaw pivot to the actuator's moving mount"));
  Serial.println(F("  d <us>   set the debounce window"));
  Serial.println(F("  s        the report, the config block, and the gate"));
  Serial.println(F("  z        discard everything and start over"));
  Serial.println(F("--------------------------------------------------------"));
}

void handleCommand(char* line) {
  if (line[0] == '\0') return;

  const char cmd = tolower(line[0]);
  const char* arg = line + 1;
  while (*arg == ' ' || *arg == '\t') arg++;
  const bool hasArg = (*arg != '\0');

  if (g_awaitingConfirm) {
    if (cmd == 'y') { resolveConfirm(true);  return; }
    if (cmd == 'n') { resolveConfirm(false); return; }
    Serial.println(F("    waiting on the supply-current answer: y or n"));
    return;
  }

  switch (cmd) {
    case '?': printHelp(); break;
    case 's': printReport(); break;
    case 'f': noiseFloorTest(); break;
    case 'e': armFor(END_EXTEND); break;
    case 'r': armFor(END_RETRACT); break;

    case 'm':
      if (!hasArg) {
        Serial.print(F("stroke = ")); Serial.print(g_strokeMm, 1);
        Serial.println(F(" mm"));
        break;
      }
      g_strokeMm = atof(arg);
      Serial.print(F("stroke = ")); Serial.print(g_strokeMm, 1);
      Serial.println(F(" mm"));
      break;

    case 'a':
      if (!hasArg) {
        Serial.print(F("arm = ")); Serial.print(g_armMm, 1);
        Serial.println(F(" mm"));
        break;
      }
      g_armMm = atof(arg);
      Serial.print(F("arm = ")); Serial.print(g_armMm, 1);
      Serial.println(F(" mm"));
      break;

    case 'd':
      if (!hasArg) {
        Serial.print(F("debounce = ")); Serial.print(g_debounceUs);
        Serial.println(F(" us"));
        break;
      }
      g_debounceUs = (unsigned long)constrain(atol(arg), 0L, 100000L);
      Serial.print(F("debounce = ")); Serial.print(g_debounceUs);
      Serial.println(F(" us  -- press z and re-run the strokes"));
      break;

    case 'z':
      g_count[0] = g_count[1] = 0;
      g_noiseTested = false;
      zeroSensorStats();
      Serial.println(F("everything discarded"));
      armFor(END_EXTEND);
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
  attachInterrupt(digitalPinToInterrupt(REED_PIN), onEdge, FALLING);

  Serial.println();
  Serial.println(F("=== Actuator Sensor Bench Test (combined) ==="));
  Serial.println(F("Reed quality and end-stop repeatability in one session,"));
  Serial.println(F("plus the config block and a go / no-go gate."));
  Serial.println();
  Serial.println(F("The motor runs from the bench supply -- this sketch does not"));
  Serial.println(F("drive it. Leave the supply ON when the actuator reaches an"));
  Serial.println(F("end so you can watch the current drop."));
  Serial.println();
  Serial.println(F("Order: f, then strokes both ways, then m and a, then s."));
  Serial.println();
  printHelp();

  armFor(END_EXTEND);
}

void loop() {
  pollSerial();

  if (g_awaitingConfirm) return;

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
