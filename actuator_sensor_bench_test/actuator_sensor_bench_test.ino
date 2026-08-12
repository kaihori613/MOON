/*
  Actuator Sensor Bench Test
  ---------------------------
  Purpose: Verify the HTS TVRO linear actuator's internal position sensor
  (magnet + reed switch) before wiring it into the closed-loop control sketch.

  WHAT THIS DOES:
   - Counts pulses from the reed switch as the actuator moves (driven
     manually from your bench DC supply, NOT by this Arduino yet).
   - Prints each pulse with a running total and elapsed time to the Serial
     Monitor, so you can watch the count increase as the actuator travels.
   - Reports pulse rate (Hz) so you can sanity-check consistency between runs.
   - Includes a reset button to zero the counter between test runs.

  WHAT THIS DOES NOT DO YET:
   - Does not drive the motor. Keep using the bench DC supply for that.
   - Does not electronically detect the cam-based limit switches. Since
     those cut motor current internally at the travel extremes (not a
     separate signal wire to Arduino), the way to confirm them on the
     bench is: run the actuator to full extension, then full retraction,
     from the DC supply, and note the pulse count where the motor current
     visibly/audibly cuts off. Do this for a couple of runs and compare —
     consistent counts at both ends is a good sign the sensor + limit
     switches agree with each other.

  WIRING (bench test):
    Reed switch signal -> Arduino pin REED_PIN (below)
    Reed switch other leg -> Arduino GND
    Reset button -> Arduino pin RESET_PIN, other leg -> GND

  NOTE ON REED SWITCH WIRING:
    Most reed switches are simple normally-open contacts with no dedicated
    supply pin. Wire one leg to the Arduino pin and the other to GND, and
    this sketch uses INPUT_PULLUP so the pin reads HIGH when open and LOW
    when the magnet closes the switch. If your actuator's sensor instead
    outputs an active signal (some are open-collector with a pull-up
    already on the board), check with a meter first -- if it's not a
    simple dry contact, let me know and I'll adjust this.

  === CONFIG -- adjust to match your wiring ===
*/

const uint8_t REED_PIN  = 2;   // must be interrupt-capable (pin 2 or 3 on Uno/Nano)
const uint8_t RESET_PIN = 4;

const unsigned long DEBOUNCE_US = 3000;  // minimum microseconds between valid pulses
                                          // (reed switches can bounce mechanically;
                                          // raise this if you see double-counted pulses,
                                          // lower it if fast pulses seem to get dropped)

volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseMicros = 0;
volatile unsigned long lastPulseInterval = 0;

unsigned long lastPrintedCount = 0;
// Timing is measured from the FIRST PULSE of a run, not from the reset. You
// reset the counter and then reach over and switch on the supply, and that
// dead time is not part of the stroke -- counting it would understate the
// pulse rate by however long you took.
unsigned long firstPulseMillis = 0;

// Run statistics. A "run" is one uninterrupted stroke; it ends automatically
// once pulses stop arriving for RUN_IDLE_MS, which is what happens when the
// cam limit switch cuts motor current at either extreme.
const unsigned long RUN_IDLE_MS = 1500;

unsigned long minGapUs      = 0xFFFFFFFFUL;   // fastest = highest pulse rate
unsigned long maxGapUs      = 0;              // slowest = lowest pulse rate
unsigned long lastPulseSeen = 0;
bool          runActive     = false;

// Totals from recent full strokes. The cam limit switches are not wired to
// the Arduino -- they cut motor current inside the actuator -- so the only
// evidence we get that they fired is that pulses stopped. What tells us they
// fired in the SAME PLACE every time is these totals agreeing.
const uint8_t RUN_HISTORY = 4;
unsigned long runTotals[RUN_HISTORY];
uint8_t       runsStored = 0;

// Zero everything and arm for the next stroke. Silent, because this runs
// automatically at the end of every run and a message each time would bury
// the summary you actually want to read.
void zeroCounters() {
  noInterrupts();
  pulseCount = 0;
  interrupts();
  lastPrintedCount = 0;
  minGapUs         = 0xFFFFFFFFUL;
  maxGapUs         = 0;
  runActive        = false;
  firstPulseMillis = 0;            // re-armed by the next pulse
}

void resetRun(const __FlashStringHelper* why) {
  zeroCounters();
  Serial.print(F(">>> counter reset -- "));
  Serial.println(why);
}

void printRunSummary(unsigned long count) {
  const float elapsedSec = (lastPulseSeen - firstPulseMillis) / 1000.0;

  Serial.println();
  Serial.println(F("--- run summary ----------------------------------------"));
  Serial.print(F("  pulses        : ")); Serial.println(count);
  Serial.print(F("  duration      : ")); Serial.print(elapsedSec, 2);
  Serial.println(F(" s"));

  // N pulses span N-1 intervals, so the rate is (count-1)/duration. Using
  // count/duration would overstate it, most noticeably on short runs.
  if (elapsedSec > 0 && count > 1) {
    Serial.print(F("  average rate  : "));
    Serial.print((count - 1) / elapsedSec, 2);
    Serial.println(F(" Hz"));
  }

  if (count < 3) {
    Serial.println(F("  (too few pulses to characterise -- run a full stroke)"));
    Serial.println(F("--------------------------------------------------------"));
    return;
  }

  const float fastHz = 1000000.0 / minGapUs;
  const float slowHz = 1000000.0 / maxGapUs;

  Serial.print(F("  fastest gap   : ")); Serial.print(minGapUs / 1000.0, 1);
  Serial.print(F(" ms  -> ")); Serial.print(fastHz, 2); Serial.println(F(" Hz"));
  Serial.print(F("  slowest gap   : ")); Serial.print(maxGapUs / 1000.0, 1);
  Serial.print(F(" ms  -> ")); Serial.print(slowHz, 2); Serial.println(F(" Hz"));

  // Turn the measurement straight into the two constants that depend on it.
  unsigned long suggDebounce = minGapUs / 4;
  if (suggDebounce > 3000) suggDebounce = 3000;      // no point going higher
  unsigned long suggStall = (maxGapUs / 1000UL) * 3UL;
  if (suggStall < 250) suggStall = 250;

  Serial.println(F("  -- copy into ActuatorConfig.h --"));
  Serial.print(F("  REED_DEBOUNCE_US  = ")); Serial.println(suggDebounce);
  Serial.print(F("  STALL_TIMEOUT_MS  = ")); Serial.println(suggStall);
  // --- cam limit switch repeatability ------------------------------------
  if (runsStored == RUN_HISTORY) {
    for (uint8_t i = 0; i < RUN_HISTORY - 1; i++) runTotals[i] = runTotals[i + 1];
    runsStored--;
  }
  runTotals[runsStored++] = count;

  if (runsStored > 1) {
    unsigned long lo = runTotals[0], hi = runTotals[0];
    Serial.print(F("  recent strokes: "));
    for (uint8_t i = 0; i < runsStored; i++) {
      if (i) Serial.print(F(", "));
      Serial.print(runTotals[i]);
      if (runTotals[i] < lo) lo = runTotals[i];
      if (runTotals[i] > hi) hi = runTotals[i];
    }
    Serial.println();

    const unsigned long spread = hi - lo;
    Serial.print(F("  spread        : ")); Serial.print(spread);
    Serial.println(spread <= 2 ? F(" counts -- cam switches are repeatable")
                               : F(" counts -- TOO WIDE, see below"));
  }

  Serial.println(F("--------------------------------------------------------"));
  Serial.println(F("A summary appearing while the supply is still ON means a"));
  Serial.println(F("cam limit switch cut the motor -- that is the switch"));
  Serial.println(F("working. Check the supply's current display: it should"));
  Serial.println(F("drop to near zero at the same moment. If current is still"));
  Serial.println(F("flowing, the motor is running and the SENSOR stopped"));
  Serial.println(F("counting instead -- a completely different fault, and one"));
  Serial.println(F("this sketch cannot tell apart on its own."));
  Serial.println();
}

void handleReedPulse() {
  unsigned long now = micros();
  unsigned long sinceLast = now - lastPulseMicros;
  if (sinceLast < DEBOUNCE_US) return;  // ignore switch bounce

  lastPulseInterval = sinceLast;
  lastPulseMicros = now;
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(REED_PIN), handleReedPulse, FALLING);

  firstPulseMillis = 0;          // set by the first pulse, not by boot

  Serial.println(F("=== Actuator Reed Switch Bench Test ==="));
  Serial.println(F("Counting is automatic. Switch the supply on and the count"));
  Serial.println(F("starts at 1; switch it off (or let a cam switch stop the"));
  Serial.println(F("motor) and after 1.5s of silence the run summary prints"));
  Serial.println(F("and the counter zeroes itself for the next stroke."));
  Serial.println(F("Nothing to press between runs. Send any character to"));
  Serial.println(F("abandon a run early."));
  Serial.println();
}

void loop() {
  // Reset button (active low)
  if (digitalRead(RESET_PIN) == LOW) {
    resetRun(F("button"));
    delay(300);  // simple debounce for the button press itself
  }

  // Anything typed into the Serial Monitor also resets, so the button is
  // optional -- useful if you have not wired one up yet.
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    resetRun(F("serial"));
  }

  noInterrupts();
  unsigned long count = pulseCount;
  unsigned long interval = lastPulseInterval;
  interrupts();

  if (count != lastPrintedCount) {
    if (count == 1) firstPulseMillis = millis();   // the stroke starts here
    float elapsedSec = (millis() - firstPulseMillis) / 1000.0;
    float hz = interval > 0 ? 1000000.0 / interval : 0;

    // Skip the first pulse of a run: its "gap" is measured from whenever the
    // last run ended, so it is meaningless.
    if (count > 1) {
      if (interval < minGapUs) minGapUs = interval;
      if (interval > maxGapUs) maxGapUs = interval;
    }

    lastPulseSeen = millis();
    runActive     = true;

    Serial.print(F("Pulse #"));
    Serial.print(count);
    Serial.print(F("  |  elapsed: "));
    Serial.print(elapsedSec, 2);
    Serial.print(F("s  |  rate: "));
    Serial.print(hz, 2);
    Serial.println(F(" Hz"));

    lastPrintedCount = count;
  }

  // Pulses stopped -- either the actuator hit a cam limit switch or you cut
  // the supply. Either way the stroke is over, so report it.
  if (runActive && millis() - lastPulseSeen > RUN_IDLE_MS) {
    runActive = false;
    printRunSummary(count);
    zeroCounters();     // armed: the next pulse begins a fresh run at 1
  }
}
