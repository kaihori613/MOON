/*
  Reed Switch Test  --  is the position sensor trustworthy?
  ---------------------------------------------------------
  One question only: does the reed switch produce exactly one clean pulse per
  magnet pass? Nothing here cares about strokes, ends of travel, or how far the
  actuator went. Run this first. If the sensor lies, every number produced by
  every other sketch in this repo is built on sand -- position is integrated
  from these pulses and never corrected.

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

  THE DRIVER IS HELD OFF, NOT IGNORED
    This sketch still does not drive the motor. But leaving the driver's input
    pins unconfigured is not the same as leaving them alone: after a reset they
    are high-impedance inputs, and an L298N's ENA floating high with IN1 and
    IN2 at different levels will run the motor. During a test whose entire
    premise is that the carriage is stationary, that is the worst possible
    failure -- it corrupts the measurement and drives the actuator unattended.

    So setup() claims those pins and drives them low before anything else.
    The bridge can then be powered, which is the whole point: the original
    clean result on this sensor was measured with no bridge in the circuit,
    and that is the one configuration where a clean result proves nothing.

  PROCEDURE
    1. Driver POWERED, motor connected, carriage stationary. Type  n  for the
       noise-floor test. Anything above zero is electrical pickup, not motion,
       and it will corrupt position forever once the bridge starts switching.
    2. Drive the actuator SLOWLY from the bench supply. Watch the pulses print.
       Slow travel is where double-counting is easiest to see.
    3. Drive it at full speed. Type  s  for the report.
    4. If rejects appear, use  d <us>  to try a different debounce without
       reflashing, then re-run. Copy the value you settle on into
       actuator_v1/Config.h as REED_DEBOUNCE_US.
*/

const uint8_t REED_PIN = 2;      // must be interrupt-capable (D2/D3 on Uno)

// --- The motor driver, which this sketch holds OFF -------------------------
//  Same selection as actuator_v1/Config.h. Nothing here ever drives the motor;
//  these pins exist only so they can be claimed and pulled low, instead of
//  being left floating where the bridge decides for itself what to do.

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

// Called first thing in setup(), before anything that can block. With the
// L298N, ENA low disables the output stage outright; IN1 and IN2 equal is
// coast, so even a failed ENA cannot produce rotation.
void holdDriverOff() {
#if MOTOR_DRIVER == DRV_L298N
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
#else
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
  digitalWrite(PIN_RPWM, LOW);
  digitalWrite(PIN_LPWM, LOW);
  digitalWrite(PIN_EN,   LOW);
#endif
}

// Live-adjustable with 'd', so you can find the right value on the bench
// instead of reflashing between guesses.
unsigned long g_debounceUs = 3000;

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
  Serial.println(F("  s        report"));
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

  switch (cmd) {
    case '?': printHelp(); break;
    case 's': printReport(); break;

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
  holdDriverOff();

  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(REED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REED_PIN), onEdge, FALLING);

  zeroStats();

  Serial.println();
  Serial.println(F("=== Reed Switch Test ==="));
  Serial.print(F("Driver held OFF: "));
#if MOTOR_DRIVER == DRV_L298N
  Serial.println(F("L298N, ENA D9 low, IN1 D6 and IN2 D5 low"));
#else
  Serial.println(F("HW-039, RPWM D9, LPWM D10, EN D8 all low"));
#endif
  Serial.println(F("Sensor only -- this sketch never turns the motor. Move the"));
  Serial.println(F("carriage by hand, or from the bench supply direct."));
  Serial.println();
  Serial.println(F("POWER THE DRIVER for the noise floor test. Measuring it"));
  Serial.println(F("with the bridge out of circuit is the one configuration"));
  Serial.println(F("where a clean result proves nothing."));
  Serial.println();
  Serial.println(F("Start with 'n' (noise floor, carriage still), then move the"));
  Serial.println(F("carriage slowly and watch the pulses, then at full"));
  Serial.println(F("speed, then 's' for the report."));
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

    g_bucket[bucketOf(gap)]++;

    if (gap < g_debounceUs) {
      g_rejected++;
      if (g_verbose) {
        Serial.print(F("  reject   gap "));
        Serial.print(gap);
        Serial.println(F(" us  (under the debounce -- bounce or noise)"));
      }
      continue;
    }

    g_accepted++;
    if (gap < g_minAcceptUs) g_minAcceptUs = gap;
    if (gap > g_maxAcceptUs) g_maxAcceptUs = gap;

    if (g_verbose) {
      Serial.print(F("Pulse #"));
      Serial.print(g_accepted);
      Serial.print(F("  gap "));
      Serial.print(gap / 1000.0, 2);
      Serial.print(F(" ms  rate "));
      Serial.print(1000000.0 / gap, 2);
      Serial.println(F(" Hz"));
    }
  }
}
