/*
  Reed Switch Test  --  is the sensor trustworthy, and how far is one pulse?
  --------------------------------------------------------------------------
  Two questions, in that order:

    1. Does the reed produce exactly one clean pulse per magnet pass?
    2. How many millimetres of rod is one pulse worth?

  The first has to be settled before the second is worth asking, because a
  sensor that double-counts gives a mm-per-pulse figure that is exactly half
  the truth and looks perfectly reasonable. Answer 1 with the histogram and
  the noise floor, then answer 2 with a timed run.

  RUNS BY TIME, AND RUNS BY COUNT
    'e 10000' drives extend for ten seconds, counts the pulses that arrive
    while it does, and stops. Then you measure the rod with a rule or calipers
    -- the distance it MOVED, not its overall length -- and type  m <mm>. That
    is the whole calibration: distance over pulses is mm per pulse, and it is a
    property of the mechanism that does not change with duty or direction.
    Distance over time is mm per second, which does change with both, and is
    only ever true for the duty the run used.

    This is the number the repo has been missing. One reed count is the floor
    on pointing resolution, and until mm-per-count is known there is no way to
    say whether that floor is finer or coarser than the link needs.

    '+50' and '-50' drive fifty COUNTS instead, and stop on the sensor rather
    than on the clock. Ten seconds out and ten seconds back does not return the
    rod to where it started -- the load is not symmetric, so the two directions
    do not travel at the same rate, and that is exactly why nothing here is
    positioned by stopwatch. Fifty counts out and fifty counts back should
    return it, and what it misses by is backlash on its own, with the speed
    difference taken out.

    A count run also reports its OVERSHOOT, which is the most useful number on
    this bench after mm-per-count. The motor is cut when the target arrives and
    the carriage keeps going, so the overshoot is coast measured in the only
    unit the controller can act on. It is the floor on where any move can land:
    Config.h asks for a deadband of one count, and if the overshoot at a given
    duty is larger than that, a move at that duty can never settle inside it.
    Dropping the duty until the overshoot fits is how SPEED_TRIM gets found.

  HOW THIS DIFFERS FROM THE OTHER SKETCHES
    The debounce filter here does NOT throw pulses away silently. The interrupt
    records every single falling edge, and the rejection happens afterwards in
    loop(), so the sketch can tell you what it *would* have discarded. That is
    the whole point: a sensor that needs heavy filtering to look clean is a
    sensor about to lose counts at speed, and a sketch that hides its rejects
    will never show you that.

  WHAT YOU ARE LOOKING FOR
    The gap histogram should show ONE cluster, at the travel rate. Two clusters
    means double-counting: a second group of very short gaps is the contact
    bouncing, or a magnet being seen twice on one pass.

  WIRING
    Reed switch one leg -> D2, other leg -> GND. INPUT_PULLUP is enabled in
    software, so the pin reads HIGH when open, LOW when the magnet closes the
    contact.

    L298N:  ENA -> D9, IN1 -> D6, IN2 -> D5.  ENA jumper OFF or the speed does
            nothing. 5V_EN jumper OFF at 24 V or the onboard regulator cooks.
    HW-039: RPWM -> D9, LPWM -> D10, R_EN/L_EN -> D8 or +5V.

    GND of the Arduino to the motor supply negative, and keep the reed's return
    off that same wire if you can -- motor return current down a shared ground
    is where pickup comes from.

  THE DRIVER IS OFF UNTIL YOU ASK FOR A RUN
    Everything except a timed run holds the bridge hard off, and setup() claims
    those pins and drives them low before anything that can block. Leaving them
    unconfigured is not the same as leaving them alone: after a reset they are
    high-impedance inputs, and an L298N's ENA floating high with IN1 and IN2 at
    different levels will run the motor. During the noise floor test, whose
    entire premise is that the carriage is stationary, that is the worst
    possible failure -- it corrupts the measurement and drives the actuator
    unattended.

    Powering the bridge is the point rather than a risk: the original clean
    result on this sensor was measured with no bridge in the circuit, and that
    is the one configuration where a clean result proves nothing.

  THE THING TO BE CAREFUL ABOUT
    Ten seconds of travel may be more stroke than the actuator has left. What
    normally stops it is a cam limit switch cutting current inside the actuator
    -- and when that happens the pulses stop while the motor is still commanded
    on, which this sketch detects and reports as an early end. That is a normal
    ending, not a fault, but the run is then shorter than you asked for. Read
    the time it actually ran, not the time you asked for, and if it ended early
    reverse a few seconds and run again from somewhere with room.

    Current limit on the supply, 3-5 A, and watch the ammeter.

  PROCEDURE
    1. Driver POWERED, motor connected, carriage stationary. Type  n  for the
       noise-floor test. Anything above zero is electrical pickup, not motion,
       and it will corrupt position forever once the bridge starts switching.
    2. Type  v  to see individual pulses, then  e 2000  and watch them arrive.
       Slow travel is where double-counting is easiest to see; 'w 90' first if
       you want it slower still.
    3. Type  s  for the sensor report. If rejects appear, use  d <us>  to try a
       different debounce without reflashing, then  z  and re-run. Copy the
       value you settle on into actuator_v1/Config.h as REED_DEBOUNCE_US.
    4. Only once that is clean: mark the rod, run  e 10000  or  r 10000  from a
       position with room to move, measure the travel, and type  m <mm>.
       Repeat both directions. mm per pulse should agree even though the
       distances do not.
    5. Coast and backlash, which want counts rather than time. From a mark,
       '+50' then '-50'. Read the overshoot on each report -- that is coast --
       and measure how far short of the mark it came back, which is backlash.
       Then 'w 150', '+50', '-50' again: both numbers should shrink.
*/

const uint8_t REED_PIN = 2;      // must be interrupt-capable (D2/D3 on Uno)

// --- The motor driver -------------------------------------------------------
//  Same selection, same pins and same ordering as actuator_v1/Config.h. The
//  bridge is off except during a timed run, and claimed low in setup() before
//  anything that can block, rather than left floating for it to decide.

#define DRV_L298N  1
#define DRV_HW039  2

#define MOTOR_DRIVER  DRV_L298N      // <-- match your hardware

#if MOTOR_DRIVER == DRV_L298N
  const uint8_t PIN_ENA = 9;
  const uint8_t PIN_IN1 = 6;
  const uint8_t PIN_IN2 = 5;
#elif MOTOR_DRIVER == DRV_HW039
  const uint8_t PIN_RPWM = 9;
  const uint8_t PIN_LPWM = 10;
  const uint8_t PIN_EN   = 8;
#else
  #error "Set MOTOR_DRIVER to DRV_L298N or DRV_HW039"
#endif

// With the L298N, ENA low disables the output stage outright; IN1 and IN2
// equal is coast, so even a failed ENA cannot produce rotation. With the
// HW-039 the two PWM inputs are the whole of the off switch, which is why
// both go low by digitalWrite rather than analogWrite(0).
void motorOff() {
#if MOTOR_DRIVER == DRV_L298N
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
#else
  digitalWrite(PIN_RPWM, LOW);
  digitalWrite(PIN_LPWM, LOW);
  digitalWrite(PIN_EN,   LOW);
#endif
}

void motorPins() {
#if MOTOR_DRIVER == DRV_L298N
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
#else
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
#endif
  motorOff();
}

// Direction is set while the output stage is off, then speed. On the HW-039
// the idle side goes hard off before the driven side goes anywhere near on:
// with the enables tied high those two pins are the only thing standing
// between the supply and a shoot-through.
void motorApply(char dir, uint8_t speed) {
#if MOTOR_DRIVER == DRV_L298N
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_IN1, dir == 'e' ? HIGH : LOW);
  digitalWrite(PIN_IN2, dir == 'e' ? LOW  : HIGH);
  analogWrite(PIN_ENA, speed);
#else
  digitalWrite(dir == 'e' ? PIN_LPWM : PIN_RPWM, LOW);
  digitalWrite(PIN_EN, HIGH);
  analogWrite(dir == 'e' ? PIN_RPWM : PIN_LPWM, speed);
#endif
}

// --- Timed run tuning -------------------------------------------------------

const unsigned long RUN_MS_DEFAULT = 10000;   // bare 'e' / 'r'
const unsigned long RUN_MS_MIN     = 200;
const unsigned long RUN_MS_MAX     = 30000;   // MAX_RUN_MS in Config.h

// A count run still gets the time backstop above: if the target is never
// reached, something has to stop the motor, and out here nothing else will.
const long RUN_COUNTS_DEFAULT = 1;            // bare '+' / '-', as in Config.h
const long RUN_COUNTS_MAX     = 2000;

const uint8_t SPEED_DEFAULT = 180;            // SPEED_RUN in Config.h
const uint8_t SPEED_FLOOR   = 60;             // below this it buzzes, not moves

// Motor commanded on but no pulse for this long: the actuator has either hit
// an internal cam limit switch or it is jammed, and nothing out here can tell
// those apart. Either way the run is over. Matches Config.h.
const uint16_t RUN_STALL_MS = 700;

// Grace from motor-on to the first pulse, for breakaway.
const uint16_t RUN_GRACE_MS = 900;

// The carriage does not stop when the current does. Pulses that arrive while
// it coasts are real travel and are part of the distance about to be measured,
// so keep counting through this window before reporting anything.
const uint16_t RUN_COAST_MS = 400;

// Magnet passes per revolution of the sensed shaft. One magnet on the motor
// shaft is the usual arrangement, in which case a pulse is a cycle and this is
// 1. If yours carries more, set the real number and the report will divide.
const uint8_t PULSES_PER_CYCLE = 1;

const uint16_t RUN_TICK_MS = 1000;   // how often a run prints its progress

// Live-adjustable with 'd', so you can find the right value on the bench
// instead of reflashing between guesses.
unsigned long g_debounceUs = 3000;

// A gap longer than this spans a pause between two moves, not travel. Real
// travel at the slowest usable duty is orders of magnitude faster; anything
// this slow is the bench standing still.
const unsigned long IDLE_GAP_US = 2000000UL;

bool g_verbose = true;           // print every pulse as it arrives ('v')

// ---------------------------------------------------------------------------
//  Edge capture
// ---------------------------------------------------------------------------
//  The ISR does the least work that is still honest: timestamp the edge, work
//  out the gap since the previous one, push it into a ring buffer. All
//  judgement happens in loop().

const uint8_t GAP_BUF = 64;
volatile unsigned long g_gap[GAP_BUF];
volatile uint8_t  g_head = 0;
volatile uint8_t  g_tail = 0;
volatile unsigned long g_lastEdgeUs = 0;
volatile unsigned long g_edgeTotal  = 0;
volatile unsigned long g_overflows  = 0;   // ring buffer lapped: loop() too slow
volatile bool     g_haveFirstEdge = false;

void onEdge() {
  const unsigned long now = micros();

  // The very first edge has no previous edge to measure against, so its "gap"
  // is however long the board has been powered up. Meaningless -- drop it.
  if (!g_haveFirstEdge) {
    g_haveFirstEdge = true;
    g_lastEdgeUs = now;
    g_edgeTotal++;
    return;
  }

  const unsigned long gap = now - g_lastEdgeUs;
  g_lastEdgeUs = now;
  g_edgeTotal++;

  const uint8_t next = (uint8_t)((g_head + 1) % GAP_BUF);
  if (next == g_tail) {          // full: drop this one, but say so later
    g_overflows++;
    return;
  }
  g_gap[g_head] = gap;
  g_head = next;
}

// ---------------------------------------------------------------------------
//  Gap histogram
// ---------------------------------------------------------------------------
//  Log-spaced buckets. A healthy reed on a moving actuator puts everything in
//  one or two adjacent buckets. Bounce shows up as a separate cluster down at
//  the sub-millisecond end, which is exactly what the debounce is there to cut.

const uint8_t N_BUCKETS = 12;
const unsigned long BUCKET_EDGE_US[N_BUCKETS] = {
  200UL, 500UL, 1000UL, 2000UL, 5000UL, 10000UL,
  20000UL, 50000UL, 100000UL, 200000UL, 500000UL, 0xFFFFFFFFUL
};

unsigned long g_bucket[N_BUCKETS];

unsigned long g_accepted = 0;
unsigned long g_rejected = 0;
unsigned long g_minAcceptUs = 0xFFFFFFFFUL;
unsigned long g_maxAcceptUs = 0;

uint8_t bucketOf(unsigned long gapUs) {
  for (uint8_t i = 0; i < N_BUCKETS; i++) {
    if (gapUs < BUCKET_EDGE_US[i]) return i;
  }
  return N_BUCKETS - 1;
}

void zeroStats() {
  noInterrupts();
  g_head = g_tail = 0;
  g_edgeTotal = 0;
  g_overflows = 0;
  g_haveFirstEdge = false;
  interrupts();

  for (uint8_t i = 0; i < N_BUCKETS; i++) g_bucket[i] = 0;
  g_accepted = 0;
  g_rejected = 0;
  g_minAcceptUs = 0xFFFFFFFFUL;
  g_maxAcceptUs = 0;
}

// ---------------------------------------------------------------------------

void printBucketLabel(uint8_t i) {
  const unsigned long edge = BUCKET_EDGE_US[i];
  if (i == N_BUCKETS - 1) {
    Serial.print(F("  >= 500 ms"));
    return;
  }
  Serial.print(F("  <  "));
  if (edge < 1000UL) {
    Serial.print(edge / 1000.0, 1);
  } else {
    Serial.print(edge / 1000UL);
    Serial.print(F(".0"));
  }
  Serial.print(F(" ms"));
}

void printHistogram() {
  unsigned long peak = 0;
  for (uint8_t i = 0; i < N_BUCKETS; i++) {
    if (g_bucket[i] > peak) peak = g_bucket[i];
  }
  if (peak == 0) {
    Serial.println(F("  (no gaps recorded yet)"));
    return;
  }

  for (uint8_t i = 0; i < N_BUCKETS; i++) {
    if (g_bucket[i] == 0) continue;

    printBucketLabel(i);

    // Mark the buckets the current debounce is discarding, so the effect of
    // changing it with 'd' is visible at a glance.
    Serial.print(BUCKET_EDGE_US[i] <= g_debounceUs ? F(" x |") : F("   |"));

    const uint8_t bar = (uint8_t)((g_bucket[i] * 40UL) / peak);
    for (uint8_t b = 0; b < bar; b++) Serial.print('#');
    Serial.print(F("  "));
    Serial.println(g_bucket[i]);
  }
  Serial.println(F("                x = discarded by the current debounce"));
}

// The cheap test for double-counting: find the bucket where most pulses landed
// (that is the travel rate), then look for a second population well below it.
// Genuine travel produces one cluster. A contact that bounces, or a magnet
// seen twice per pass, produces two.
bool looksBimodal(uint8_t* modeOut, uint8_t* strayOut) {
  uint8_t mode = 0;
  unsigned long peak = 0;
  for (uint8_t i = 0; i < N_BUCKETS; i++) {
    if (g_bucket[i] > peak) { peak = g_bucket[i]; mode = i; }
  }
  if (peak < 5 || mode < 3) return false;

  // "Meaningful" means more than a couple of stray edges: 2% of the peak, or
  // 2 counts, whichever is larger.
  unsigned long threshold = peak / 50;
  if (threshold < 2) threshold = 2;

  for (uint8_t i = 0; i + 3 <= mode; i++) {
    if (g_bucket[i] >= threshold) {
      if (modeOut)  *modeOut = mode;
      if (strayOut) *strayOut = i;
      return true;
    }
  }
  return false;
}

void printReport() {
  noInterrupts();
  const unsigned long edges = g_edgeTotal;
  const unsigned long overflow = g_overflows;
  interrupts();

  Serial.println();
  Serial.println(F("--- reed switch report ---------------------------------"));
  Serial.print(F("  debounce      : ")); Serial.print(g_debounceUs);
  Serial.println(F(" us"));
  Serial.print(F("  edges seen    : ")); Serial.println(edges);
  Serial.print(F("  accepted      : ")); Serial.println(g_accepted);
  Serial.print(F("  rejected      : ")); Serial.print(g_rejected);

  if (g_accepted > 0) {
    Serial.print(F("  ("));
    Serial.print((g_rejected * 100.0) / (g_accepted + g_rejected), 1);
    Serial.print(F("%)"));
  }
  Serial.println();

  if (overflow > 0) {
    Serial.print(F("  ! buffer overflows: ")); Serial.println(overflow);
    Serial.println(F("    Edges arrived faster than loop() drained them. Turn"));
    Serial.println(F("    off per-pulse printing with 'v' and repeat -- Serial"));
    Serial.println(F("    output is almost certainly what slowed it down."));
  }

  if (g_accepted < 3) {
    Serial.println(F("  (too few pulses -- move the actuator and try again)"));
    Serial.println(F("--------------------------------------------------------"));
    return;
  }

  Serial.print(F("  fastest gap   : ")); Serial.print(g_minAcceptUs / 1000.0, 2);
  Serial.print(F(" ms  -> ")); Serial.print(1000000.0 / g_minAcceptUs, 2);
  Serial.println(F(" Hz"));
  Serial.print(F("  slowest gap   : ")); Serial.print(g_maxAcceptUs / 1000.0, 2);
  Serial.print(F(" ms  -> ")); Serial.print(1000000.0 / g_maxAcceptUs, 2);
  Serial.println(F(" Hz"));

  Serial.println();
  Serial.println(F("  gap distribution:"));
  printHistogram();

  // --- verdict ------------------------------------------------------------
  Serial.println();
  uint8_t mode = 0, stray = 0;

  if (looksBimodal(&mode, &stray)) {
    Serial.println(F("  VERDICT: TWO CLUSTERS -- suspect double counting."));
    Serial.println(F("  Most gaps are at the travel rate, but a second group is"));
    Serial.println(F("  far too short to be real motion. Either the contact is"));
    Serial.println(F("  bouncing or one magnet pass is being seen twice."));
    Serial.println(F("  Raise the debounce with 'd' until the short cluster is"));
    Serial.println(F("  marked x, then confirm the accepted count still matches"));
    Serial.println(F("  the physical travel."));
  } else if (g_rejected == 0) {
    Serial.println(F("  VERDICT: clean. No edge needed rejecting."));
    Serial.println(F("  The debounce could probably come down, which buys margin"));
    Serial.println(F("  at full speed. Only do that if the fastest gap above is"));
    Serial.println(F("  uncomfortably close to the debounce value."));
  } else if (g_rejected * 5 < g_accepted) {
    Serial.println(F("  VERDICT: usable. A few edges rejected, which is normal"));
    Serial.println(F("  mechanical bounce being cleaned up as intended."));
  } else {
    Serial.println(F("  VERDICT: NOISY -- rejecting a large share of all edges."));
    Serial.println(F("  The filter is holding this together. It will not hold at"));
    Serial.println(F("  full speed, when real pulses arrive close enough to the"));
    Serial.println(F("  debounce window to be thrown away with the noise."));
    Serial.println(F("  Wants an RC filter and a stronger pull-up, not a bigger"));
    Serial.println(F("  debounce."));
  }

  // The constraint that actually matters: the filter must be comfortably
  // shorter than the shortest real gap, or it starts eating real pulses.
  if (g_minAcceptUs < g_debounceUs * 3) {
    Serial.println();
    Serial.println(F("  ! The fastest real gap is within 3x of the debounce."));
    Serial.println(F("    There is very little margin here -- at full speed this"));
    Serial.println(F("    filter will start discarding real pulses, and lost"));
    Serial.println(F("    counts never come back."));
  }

  Serial.println(F("  -- copy into actuator_v1/Config.h --"));
  Serial.print(F("  REED_DEBOUNCE_US  = ")); Serial.println(g_debounceUs);
  Serial.println(F("--------------------------------------------------------"));
  Serial.println();
}

// ---------------------------------------------------------------------------
//  Runs
// ---------------------------------------------------------------------------
//  Two ways to ask for travel, and the difference between them is the whole
//  lesson of this bench:
//
//    'e 10000'  ten seconds of extend        -- time is the target
//    '+50'      fifty pulses of extend       -- distance is the target
//
//  Ten seconds out and ten seconds back does NOT return the rod to where it
//  started, because the two directions do not travel at the same rate under an
//  asymmetric load. Fifty counts out and fifty counts back should -- and what
//  it misses by is backlash, measured directly, with the speed asymmetry taken
//  out of the picture. That is the pair of runs worth doing.
//
//  A count run also measures its own overshoot, which is coast expressed in the
//  only unit the controller can act on. Config.h wants that number twice over:
//  as COAST_SETTLE_MS, and as the duty at which overshoot finally fits inside
//  DEADBAND_COUNTS -- which is what SPEED_TRIM is for.
//
//  Either way, anything that could cost a pulse costs the measurement, which is
//  why per-pulse printing is suppressed for the duration and buffer overflows
//  are reported as grounds to throw the run away.

uint8_t g_speed = SPEED_DEFAULT;

bool          g_runActive   = false;   // includes the coast tail
bool          g_runCoasting = false;   // motor already cut, still counting
bool          g_runStalled  = false;   // pulses stopped while still commanded
bool          g_runByCounts = false;   // target is pulses, not milliseconds
char          g_runDir      = 'e';
unsigned long g_runWindowMs = 0;       // the backstop, and the target if timed
unsigned long g_runTarget   = 0;       // pulses wanted, if g_runByCounts
unsigned long g_runStartMs  = 0;
unsigned long g_runStopMs   = 0;       // when the motor was cut
unsigned long g_runNextTick = 0;

unsigned long g_runPulses      = 0;
unsigned long g_runCoastPulses = 0;
unsigned long g_runRejects     = 0;
unsigned long g_runOverflowAt  = 0;    // overflow count when the run started
unsigned long g_lastAcceptMs   = 0;

const __FlashStringHelper* g_runWhy = nullptr;

// The last completed run, kept so 'm' has something to divide by.
bool          g_haveRun     = false;
unsigned long g_lastPulses  = 0;
unsigned long g_lastRanMs   = 0;
uint8_t       g_lastSpeed   = 0;
char          g_lastDir     = 'e';
bool          g_lastEndedEarly = false;

void beginRun(char dir, bool byCounts, unsigned long amount) {
  if (g_runActive) {
    Serial.println(F("  already running -- 'x' stops it"));
    return;
  }
  if (g_speed < SPEED_FLOOR) {
    Serial.println(F("  duty is below the floor -- 'w <60-255>' first"));
    return;
  }

  g_runDir      = dir;
  g_runByCounts = byCounts;
  g_runTarget   = byCounts ? amount : 0;
  // A count run keeps the full time backstop: reaching the target is what
  // normally ends it, and if that never happens something still has to.
  g_runWindowMs = byCounts ? RUN_MS_MAX
                           : constrain(amount, RUN_MS_MIN, RUN_MS_MAX);
  g_runPulses      = 0;
  g_runCoastPulses = 0;
  g_runRejects     = 0;
  g_runCoasting = false;
  g_runStalled  = false;
  g_runWhy      = nullptr;

  noInterrupts();
  g_runOverflowAt = g_overflows;
  interrupts();

  Serial.println();
  Serial.print(F("  RUN  "));
  Serial.print(dir == 'e' ? F("EXTEND") : F("RETRACT"));
  Serial.print(F("  duty ")); Serial.print(g_speed);
  Serial.print(F("  for "));
  if (byCounts) {
    Serial.print(g_runTarget); Serial.print(F(" counts"));
  } else {
    Serial.print(g_runWindowMs / 1000.0, 2); Serial.print(F(" s"));
  }
  Serial.println(F("   ('x' stops)"));
  if (g_verbose) {
    Serial.println(F("  (per-pulse printing is held off while it runs -- the"));
    Serial.println(F("   serial writes are what would lose the count)"));
  }

  g_runActive  = true;
  g_runStartMs = millis();
  g_runNextTick = g_runStartMs + RUN_TICK_MS;
  motorApply(dir, g_speed);
}

// Cut the motor but stay in the run: the carriage is still moving and those
// pulses belong to the distance being measured.
void endDrive(const __FlashStringHelper* why) {
  motorOff();
  g_runStopMs   = millis();
  g_runCoasting = true;
  g_runWhy      = why;

  Serial.print(F("  motor off ("));
  Serial.print(why);
  Serial.println(F(") -- counting the coast"));
}

void printRunReport() {
  const unsigned long ranMs = g_runStopMs - g_runStartMs;

  Serial.println();
  Serial.println(F("--- run ------------------------------------------------"));
  Serial.print(F("  direction     : "));
  Serial.print(g_runDir == 'e' ? F("EXTEND") : F("RETRACT"));
  Serial.print(F("        duty ")); Serial.println(g_speed);

  Serial.print(F("  asked for     : "));
  if (g_runByCounts) {
    Serial.print(g_runTarget); Serial.println(F(" counts"));
  } else {
    Serial.print(g_runWindowMs / 1000.0, 2); Serial.println(F(" s"));
  }

  Serial.print(F("  ran           : ")); Serial.print(ranMs / 1000.0, 2);
  Serial.print(F(" s   (")); Serial.print(g_runWhy); Serial.println(F(")"));

  Serial.print(F("  pulses        : ")); Serial.print(g_runPulses);
  if (g_runCoastPulses > 0) {
    Serial.print(F("   (")); Serial.print(g_runCoastPulses);
    Serial.print(F(" of them after the motor was cut)"));
  }
  Serial.println();

  // The carriage does not stop when the current does, and how far past the
  // cut it goes is the number every landing in actuator_v1 depends on. In
  // milliseconds it is COAST_SETTLE_MS; in counts it is the error the deadband
  // has to absorb. Neither has ever been measured on this hardware.
  Serial.print(F("  coast         : "));
  if (g_runCoastPulses > 0) {
    Serial.print(g_runCoastPulses); Serial.print(F(" counts in "));
    Serial.print(g_lastAcceptMs - g_runStopMs); Serial.println(F(" ms"));
  } else {
    Serial.print(F("no counts after the cut -- under one count, and under "));
    Serial.print(RUN_COAST_MS); Serial.println(F(" ms"));
  }

  if (PULSES_PER_CYCLE > 1) {
    Serial.print(F("  cycles        : "));
    Serial.print(g_runPulses / (float)PULSES_PER_CYCLE, 2);
    Serial.print(F("   (at ")); Serial.print(PULSES_PER_CYCLE);
    Serial.println(F(" magnets per revolution)"));
  }

  Serial.print(F("  rejected      : ")); Serial.println(g_runRejects);

  if (g_runPulses >= 2 && ranMs > 0) {
    Serial.print(F("  rate          : "));
    Serial.print((g_runPulses * 1000.0) / ranMs, 2);
    Serial.print(F(" pulses/s   mean gap "));
    Serial.print(ranMs / (float)g_runPulses, 1);
    Serial.println(F(" ms"));
  }

  noInterrupts();
  const unsigned long overflowed = g_overflows - g_runOverflowAt;
  interrupts();
  if (overflowed > 0) {
    Serial.print(F("  ! edges lost to buffer overflow: "));
    Serial.println(overflowed);
    Serial.println(F("    The count above is short by at least that much and"));
    Serial.println(F("    must not be calibrated against. Turn per-pulse"));
    Serial.println(F("    printing off with 'v' and run it again."));
  }

  if (g_runStalled) {
    Serial.println();
    Serial.println(F("  Ended early: the pulses stopped while the motor was"));
    Serial.println(F("  still commanded on. Almost certainly an internal cam"));
    Serial.println(F("  cutting at the end of travel, which is a normal ending"));
    Serial.println(F("  -- but the rod stopped moving before the window did."));
    Serial.println(F("  The count and the distance still agree with each other,"));
    Serial.println(F("  so this run is fine to measure. The mm/s figure below"));
    Serial.println(F("  will be right and the time will not mean what you asked."));
  }

  if (g_runPulses == 0) {
    Serial.println();
    Serial.println(F("  No pulses at all. If the rod moved, the sensor or its"));
    Serial.println(F("  wiring is the problem, not the bridge -- 'p' reads the"));
    Serial.println(F("  pin level directly. If it did not move, raise the duty"));
    Serial.println(F("  with 'w' and check the supply is not current limiting."));
    Serial.println(F("--------------------------------------------------------"));
    return;
  }

  // What a count run is really for. Landing exactly on the target is not
  // possible -- the decision to cut comes after the count that triggered it,
  // and the carriage coasts after that -- so the interesting number is how far
  // past it went, and whether that fits inside the deadband v1 will use.
  if (g_runByCounts && !g_runStalled && g_runPulses >= g_runTarget) {
    Serial.println();
    if (g_runPulses > g_runTarget) {
      Serial.print(F("  OVERSHOT by ")); Serial.print(g_runPulses - g_runTarget);
      Serial.println(F(" counts."));
    } else {
      Serial.println(F("  Landed on the target."));
    }
    Serial.println(F("  That overshoot is coast, and it is the resolution floor"));
    Serial.println(F("  at this duty -- no controller can land finer than the"));
    Serial.println(F("  distance the carriage travels after the current is cut."));
    Serial.println(F("  Config.h sets DEADBAND_COUNTS to 1, so if the overshoot"));
    Serial.println(F("  is bigger than that, a move at this duty can never"));
    Serial.println(F("  settle inside the deadband. Drop the duty with 'w' and"));
    Serial.println(F("  repeat until it fits: that duty is SPEED_TRIM."));
  }

  Serial.println();
  if (g_runByCounts) {
    Serial.println(F("  Now run the same count the other way. Where the rod"));
    Serial.println(F("  lands short of where it started is backlash -- with the"));
    Serial.println(F("  direction speed difference taken out, which is what a"));
    Serial.println(F("  timed run cannot do."));
  }
  Serial.println(F("  Measure how far the rod MOVED -- not its total length --"));
  Serial.println(F("  and type   m <mm>"));
  Serial.println(F("--------------------------------------------------------"));
}

void finishRun() {
  g_runActive   = false;
  g_runCoasting = false;

  printRunReport();

  if (g_runPulses > 0) {
    g_haveRun    = true;
    g_lastPulses = g_runPulses;
    g_lastRanMs  = g_runStopMs - g_runStartMs;
    g_lastSpeed  = g_speed;
    g_lastDir    = g_runDir;
    g_lastEndedEarly = g_runStalled;
  }
}

void serviceRun() {
  if (!g_runActive) return;

  const unsigned long now = millis();

  if (g_runCoasting) {
    if (now - g_runStopMs >= RUN_COAST_MS) finishRun();
    return;
  }

  // Checked after the ring buffer has been drained, so the count is as current
  // as it can be. One loop pass of latency is nothing next to the coast that
  // follows, which is the error that actually decides where the rod lands.
  if (g_runByCounts && g_runPulses >= g_runTarget) {
    endDrive(F("count reached"));
    return;
  }

  if (now - g_runStartMs >= g_runWindowMs) {
    endDrive(g_runByCounts ? F("backstop -- the count was never reached")
                           : F("window elapsed"));
    return;
  }

  // Pulses stopping while the motor is still on is the end of travel or a jam,
  // and out here those look identical. Both want the current off now.
  const unsigned long since = now - (g_runPulses ? g_lastAcceptMs : g_runStartMs);
  if (since >= (g_runPulses ? RUN_STALL_MS : RUN_GRACE_MS)) {
    g_runStalled = true;
    endDrive(g_runPulses ? F("pulses stopped -- end of travel, or jammed")
                         : F("never broke away -- no pulse at all"));
    return;
  }

  if (now >= g_runNextTick) {
    g_runNextTick += RUN_TICK_MS;
    Serial.print(F("    "));
    Serial.print((now - g_runStartMs) / 1000.0, 1);
    Serial.print(F(" s   "));
    Serial.print(g_runPulses);
    if (g_runByCounts) {
      Serial.print(F(" / ")); Serial.print(g_runTarget);
      Serial.println(F(" counts"));
    } else {
      Serial.println(F(" pulses"));
    }
  }
}

// The calibration itself. Distance over pulses is a property of the mechanism;
// distance over time is a property of this duty and nothing else.
void measuredDistance(float mm) {
  if (!g_haveRun) {
    Serial.println(F("  no run to measure -- 'e' or 'r' first"));
    return;
  }
  if (mm <= 0.0f) {
    Serial.println(F("  give the distance travelled in mm, as a positive"));
    Serial.println(F("  number -- direction is already known from the run"));
    return;
  }

  const float mmPerPulse = mm / g_lastPulses;

  Serial.println();
  Serial.println(F("--- distance calibration -------------------------------"));
  Serial.print(F("  measured      : ")); Serial.print(mm, 2);
  Serial.print(F(" mm over ")); Serial.print(g_lastPulses);
  Serial.print(F(" pulses "));
  Serial.println(g_lastDir == 'e' ? F("extending") : F("retracting"));

  Serial.print(F("  mm per pulse  : ")); Serial.println(mmPerPulse, 4);
  Serial.print(F("  pulses per mm : ")); Serial.println(1.0f / mmPerPulse, 3);
  if (PULSES_PER_CYCLE > 1) {
    Serial.print(F("  mm per cycle  : "));
    Serial.println(mmPerPulse * PULSES_PER_CYCLE, 4);
  }

  Serial.print(F("  travel rate   : "));
  Serial.print((mm * 1000.0f) / g_lastRanMs, 2);
  Serial.print(F(" mm/s at duty ")); Serial.println(g_lastSpeed);

  Serial.println();
  Serial.println(F("  mm/pulse is the mechanism and holds at any duty. mm/s is"));
  Serial.println(F("  this duty only, and both ends of the stroke will be"));
  Serial.println(F("  slower than the middle under load."));

  // One pulse is the smallest move the controller can distinguish, so it is
  // also the floor on pointing -- which is the whole reason this number was
  // wanted. Quantisation is +/-1 pulse, so a short run inflates the error.
  Serial.print(F("  resolution    : one count = "));
  Serial.print(mmPerPulse, 4);
  Serial.println(F(" mm of rod"));
  Serial.print(F("  this run is +/-1 pulse, so +/-"));
  Serial.print((100.0f / g_lastPulses), 1);
  Serial.println(F("% on the figure above."));
  if (g_lastPulses < 50) {
    Serial.println(F("  Run longer for a tighter number -- the error is one"));
    Serial.println(F("  pulse however far it went."));
  }
  if (g_lastEndedEarly) {
    Serial.println(F("  That run ended early against a stop. Fine for mm/pulse,"));
    Serial.println(F("  but the rod was slowing into the cam, so mm/s is low."));
  }
  Serial.println(F("--------------------------------------------------------"));
  Serial.println();
}

// ---------------------------------------------------------------------------
//  Noise floor
// ---------------------------------------------------------------------------
//  With the actuator stationary, every edge is electrical. This is the single
//  most useful number this sketch produces, because noise counts are
//  indistinguishable from real ones downstream -- they corrupt position
//  permanently and nothing later can detect or undo it.

const unsigned long NOISE_TEST_MS = 10000;

void noiseFloorTest() {
  Serial.println();
  Serial.println(F("Noise floor test: 10 seconds. The actuator must be STOPPED."));
  Serial.println(F("Leave the motor supply switched on if you can -- a bridge"));
  Serial.println(F("sitting idle still radiates, and that is what you want to"));
  Serial.println(F("measure."));

  zeroStats();
  const unsigned long start = millis();

  while (millis() - start < NOISE_TEST_MS) {
    if ((millis() - start) % 1000 < 20) {
      Serial.print('.');
      delay(25);
    }
  }
  Serial.println();

  noInterrupts();
  const unsigned long edges = g_edgeTotal;
  interrupts();

  Serial.print(F("  edges in 10 s : ")); Serial.println(edges);
  if (edges == 0) {
    Serial.println(F("  VERDICT: clean. No pickup with the actuator stopped."));
  } else {
    Serial.println(F("  VERDICT: PICKUP DETECTED."));
    Serial.println(F("  These are counts the controller will believe. They are"));
    Serial.println(F("  indistinguishable from real motion, so position will"));
    Serial.println(F("  drift and nothing downstream can detect it. Fix this"));
    Serial.println(F("  before closing the loop: shield the sensor run, add an"));
    Serial.println(F("  RC filter at the pin, keep it away from the motor leads."));
  }
  Serial.println();
  zeroStats();
}

// ---------------------------------------------------------------------------

void printHelp() {
  Serial.println(F("--- commands -------------------------------------------"));
  Serial.println(F("  ?        this help"));
  Serial.println(F("  e [ms]   run EXTEND  for ms and count pulses (10000)"));
  Serial.println(F("  r [ms]   run RETRACT for ms and count pulses (10000)"));
  Serial.println(F("  +[n]     extend  n counts and stop  (1)"));
  Serial.println(F("  -[n]     retract n counts and stop  (1)"));
  Serial.println(F("  x        stop the run now"));
  Serial.println(F("  w <n>    PWM duty, 60-255"));
  Serial.println(F("  m <mm>   distance the rod moved on the last run"));
  Serial.println(F("  s        sensor report"));
  Serial.println(F("  z        zero all statistics"));
  Serial.println(F("  d <us>   set the debounce window (no reflash needed)"));
  Serial.println(F("  n        noise floor test -- actuator must be STOPPED"));
  Serial.println(F("  v        toggle per-pulse printing"));
  Serial.println(F("  p        read the pin level right now"));
  Serial.println(F("--------------------------------------------------------"));
}

void handleCommand(char* line) {
  if (line[0] == '\0') return;

  const char cmd = tolower(line[0]);
  const char* arg = line + 1;
  while (*arg == ' ' || *arg == '\t') arg++;
  const bool hasArg = (*arg != '\0');
  const long argVal = hasArg ? strtol(arg, nullptr, 10) : 0;

  // Anything that would move the goalposts under a run in progress: the count
  // and the distance have to describe the same stretch of travel or the
  // calibration is meaningless.
  if (g_runActive && (cmd == 'z' || cmd == 'd' || cmd == 'n')) {
    Serial.println(F("  not while a run is going -- 'x' stops it"));
    return;
  }

  switch (cmd) {
    case '?': printHelp(); break;
    case 's': printReport(); break;

    case 'e':
    case 'r':
      beginRun(cmd, false, hasArg ? (unsigned long)argVal : RUN_MS_DEFAULT);
      break;

    // Same fingering as actuator_v1: '+' and '-' are counts, 'e' and 'r' are
    // open-loop time. Learning them here means learning them once.
    case '+':
    case '-':
      beginRun(cmd == '+' ? 'e' : 'r', true,
               (unsigned long)constrain(hasArg ? argVal : RUN_COUNTS_DEFAULT,
                                        1L, RUN_COUNTS_MAX));
      break;

    case 'x':
      if (g_runActive && !g_runCoasting) {
        endDrive(F("stopped by hand"));
      } else if (g_runActive) {
        Serial.println(F("  motor is already off -- letting the coast finish"));
      } else {
        motorOff();               // belt and braces, costs nothing
        Serial.println(F("  already stopped"));
      }
      break;

    case 'w':
      if (!hasArg) {
        Serial.print(F("duty = ")); Serial.println(g_speed);
        break;
      }
      g_speed = (uint8_t)constrain(argVal, 0L, 255L);
      Serial.print(F("duty = ")); Serial.println(g_speed);
      if (g_speed < SPEED_FLOOR) {
        Serial.println(F("  below the floor -- it will buzz without moving"));
      }
      if (g_runActive && !g_runCoasting) {
        Serial.println(F("  applies to the NEXT run, not this one -- changing"));
        Serial.println(F("  speed mid-run would make mm/s mean nothing"));
      }
      break;

    case 'm': measuredDistance(hasArg ? atof(arg) : 0.0f); break;

    case 'z':
      zeroStats();
      Serial.println(F("statistics zeroed"));
      break;

    case 'd':
      if (!hasArg) {
        Serial.print(F("debounce = ")); Serial.print(g_debounceUs);
        Serial.println(F(" us"));
        break;
      }
      g_debounceUs = (unsigned long)constrain(argVal, 0L, 100000L);
      Serial.print(F("debounce = ")); Serial.print(g_debounceUs);
      Serial.println(F(" us"));
      // The accepted/rejected tallies were counted as pulses arrived and are
      // not retroactively re-judged. The histogram is raw gaps, though, so its
      // x markers do move -- which previews what this value would have cut.
      Serial.println(F("The histogram's x markers now show what this value would"));
      Serial.println(F("have discarded. The accepted/rejected tallies still"));
      Serial.println(F("reflect the old one -- press 'z' and re-run the stroke"));
      Serial.println(F("to measure it properly."));
      break;

    case 'n': noiseFloorTest(); break;

    case 'v':
      g_verbose = !g_verbose;
      Serial.print(F("per-pulse printing "));
      Serial.println(g_verbose ? F("on") : F("off"));
      break;

    case 'p':
      Serial.print(F("D"));
      Serial.print(REED_PIN);
      Serial.print(F(" reads "));
      Serial.println(digitalRead(REED_PIN) == LOW
                     ? F("LOW  (magnet present / contact closed)")
                     : F("HIGH (contact open)"));
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
  // Before the serial handshake, which can block: an unconfigured driver input
  // is an input, and an L298N gets to decide for itself what that means.
  motorPins();

  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(REED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REED_PIN), onEdge, FALLING);

  zeroStats();

  Serial.println();
  Serial.println(F("=== Reed Switch Test -- sensor, and mm per pulse ==="));
  Serial.print(F("Driver: "));
#if MOTOR_DRIVER == DRV_L298N
  Serial.println(F("L298N, ENA D9, IN1 D6, IN2 D5 -- all low until a run"));
#else
  Serial.println(F("HW-039, RPWM D9, LPWM D10, EN D8 -- all low until a run"));
#endif
  Serial.print(F("Duty ")); Serial.print(g_speed);
  Serial.print(F(", debounce ")); Serial.print(g_debounceUs);
  Serial.println(F(" us"));
  Serial.println();
  Serial.println(F("POWER THE DRIVER before the noise floor test. Measuring it"));
  Serial.println(F("with the bridge out of circuit is the one configuration"));
  Serial.println(F("where a clean result proves nothing."));
  Serial.println();
  Serial.println(F("Order: 'n' with the carriage still, then a short 'e 2000'"));
  Serial.println(F("to watch pulses arrive, then 's' for the sensor verdict."));
  Serial.println(F("Only once that is clean is the calibration worth doing:"));
  Serial.println(F("mark the rod, 'e 10000', measure the travel, 'm <mm>'."));
  Serial.println();
  Serial.println(F("'+50' and '-50' stop on the sensor instead of the clock."));
  Serial.println(F("Out and back by COUNT should return the rod to its mark;"));
  Serial.println(F("out and back by TIME will not, and the gap between those"));
  Serial.println(F("two facts is the reason this system counts pulses."));
  Serial.println();
  Serial.println(F("Current limit the supply to 3-5 A. Ten seconds may be more"));
  Serial.println(F("stroke than is left -- start from somewhere with room."));
  Serial.println();
  printHelp();
}

void loop() {
  pollSerial();

  // Drain the ring buffer and judge each gap. Doing this here rather than in
  // the ISR is what lets the report account for rejected edges.
  while (true) {
    unsigned long gap;

    noInterrupts();
    if (g_tail == g_head) { interrupts(); break; }
    gap = g_gap[g_tail];
    g_tail = (uint8_t)((g_tail + 1) % GAP_BUF);
    interrupts();

    // The first pulse of a move is separated from the last pulse of the
    // previous one by however long the bench sat idle. The magnet did pass, so
    // it counts -- but letting that gap into the histogram or the slowest-gap
    // figure makes both describe the pause instead of the travel.
    const bool travelGap = (gap < IDLE_GAP_US);

    if (travelGap) g_bucket[bucketOf(gap)]++;

    if (gap < g_debounceUs) {
      g_rejected++;
      if (g_runActive) g_runRejects++;
      if (g_verbose && !g_runActive) {
        Serial.print(F("  reject   gap "));
        Serial.print(gap);
        Serial.println(F(" us  (under the debounce -- bounce or noise)"));
      }
      continue;
    }

    g_accepted++;
    if (travelGap) {
      if (gap < g_minAcceptUs) g_minAcceptUs = gap;
      if (gap > g_maxAcceptUs) g_maxAcceptUs = gap;
    }

    if (g_runActive) {
      g_runPulses++;
      if (g_runCoasting) g_runCoastPulses++;
      g_lastAcceptMs = millis();
    }

    // Serial writes during a run are what would overflow the ring and cost the
    // count the whole measurement rests on, so the run prints once a second
    // and nothing else.
    if (g_verbose && !g_runActive) {
      Serial.print(F("Pulse #"));
      Serial.print(g_accepted);
      Serial.print(F("  gap "));
      Serial.print(gap / 1000.0, 2);
      Serial.print(F(" ms  rate "));
      Serial.print(1000000.0 / gap, 2);
      Serial.println(F(" Hz"));
    }
  }

  serviceRun();
}
