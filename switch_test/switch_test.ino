// ===========================================================================
//  switch_test  --  do the buttons, the limit switches and the home switch
//                   read the way actuator_v2 expects?
// ===========================================================================
//
//  115200 baud, nothing to type. Two stages:
//
//    1. REST CHECK. Don't touch anything. Every input must read up / clear
//       on its own. Any that don't are listed with the likely cause, and the
//       list is re-checked continuously, so you can fix a wire and watch it
//       drop off. Once all six have read right for a full second it prints
//       READY and beeps.
//    2. PRESS TEST. One line and a beep each time any input changes, so you
//       can trip a limit switch out at the pivot and hear it from the bench.
//       Extend and retract instead sound for as long as they are held --
//       extend higher, retract lower. The home switch gives one 2 s beep.
//       Presses are counted only from READY on.
//
//  Send any character to reprint everything and run the rest check again.
//
//  No motor code. The L298N pins are all held LOW, so the bridge stays off
//  whether or not the 25 V is on -- including dead channel A's.
//
//  Pins are actuator_v2's (WIRING.md):
//    D3 extend, D4 retract, D7 kill   push buttons, pin -> button -> GND
//    A0 limit 1 (+15), A1 limit 2 (-15)   microswitches, COM-NC to GND
//    A2 home                          microswitch, COM-NO to GND

// Match actuator_v2/Config.h. 1 = limits wired COM-NC, 0 = wired COM-NO.
#define LIMITS_NORMALLY_CLOSED 1

const uint8_t PIN_BUZZER = 11;
const uint8_t BRIDGE_PINS[] = {9, 6, 5, 10, 8, 12};   // ENA IN1 IN2 ENB IN3 IN4

const uint8_t N = 6;
const uint8_t PINS[N]  = {3, 4, 7, A0, A1, A2};
const bool    IS_NC[N] = {false, false, false,
                          LIMITS_NORMALLY_CLOSED, LIMITS_NORMALLY_CLOSED, false};
const bool    IS_LIMIT[N] = {false, false, false, true, true, true};

const uint8_t  DEBOUNCE_MS = 20;
const uint16_t REST_QUIET_MS = 1000;  // how long everything must read right

// The home switch gets one long beep when it is pressed, so it can be told
// apart from everything else by ear. Nothing interrupts it once it starts.
const uint8_t  HOME = 5;              // index of A2 in PINS
const uint16_t HOME_BEEP_MS = 2000;
static unsigned long g_longBeepUntil = 0;

// Extend and retract sound continuously while held, at different pitches so
// they can be told apart by ear. Any timed beep (home, the short ones, READY)
// takes over while it plays, and the held tone comes back after it.
const uint8_t  EXTEND = 0, RETRACT = 1;   // indexes in PINS
const uint16_t EXTEND_HZ  = 2000;
const uint16_t RETRACT_HZ = 1300;
static unsigned long g_shortUntil = 0;    // end of the current timed beep
static uint16_t      g_holdHz = 0;        // held tone now playing, 0 = none

static bool          g_stable[N];     // debounced raw level, true = HIGH
static bool          g_raw[N];
static unsigned long g_changed[N];
static uint16_t      g_count[N];      // times it has gone active since READY

static bool          g_ready = false;
static uint8_t       g_wrongMask = 0xFF;   // 0xFF = not reported yet
static unsigned long g_lastChange = 0;

void printName(uint8_t i) {
  switch (i) {
    case 0: Serial.print(F("D3 extend button ")); break;
    case 1: Serial.print(F("D4 retract button")); break;
    case 2: Serial.print(F("D7 kill button   ")); break;
    case 3: Serial.print(F("A0 limit 1 (+15) ")); break;
    case 4: Serial.print(F("A1 limit 2 (-15) ")); break;
    case 5: Serial.print(F("A2 home switch   ")); break;
  }
}

// Active = pressed, or tripped. NO contacts pull LOW when active; an NC
// contact opens, and the pull-up takes the pin HIGH.
bool isActive(uint8_t i) {
  return IS_NC[i] ? g_stable[i] : !g_stable[i];
}

void printInput(uint8_t i) {
  printName(i);
  Serial.print(F("  raw "));
  Serial.print(g_stable[i] ? F("HIGH") : F("LOW "));
  Serial.print(F("  -> "));
  if (IS_LIMIT[i]) Serial.print(isActive(i) ? F("TRIPPED") : F("clear  "));
  else             Serial.print(isActive(i) ? F("PRESSED") : F("up     "));
  Serial.print(F("  x"));
  Serial.println(g_count[i]);
}

void printAll() {
  Serial.println();
  for (uint8_t i = 0; i < N; i++) printInput(i);
  Serial.println();
}

// Why input i reads active with nobody touching it.
void printRestCause(uint8_t i) {
  printName(i);
  if (IS_LIMIT[i] && IS_NC[i]) {
    Serial.println(F(" reads TRIPPED at rest. Most likely:"));
    Serial.println(F("     - it is wired COM-NO: set LIMITS_NORMALLY_CLOSED 0"));
    Serial.println(F("     - its wire or GND return is off (NC reads that as tripped)"));
    Serial.println(F("     - or it really is pressed"));
  } else if (!IS_LIMIT[i]) {
    Serial.println(F(" reads PRESSED at rest. Most likely:"));
    Serial.println(F("     - the button goes to 5V with a resistor to GND (pull-down)."));
    Serial.println(F("       Wire it pin -> button -> GND and take that resistor out"));
    Serial.println(F("     - or it is held down, stuck, or shorted to GND"));
  } else {
    Serial.println(F(" reads PRESSED at rest. Most likely:"));
    Serial.println(F("     - wired to the NC contact instead of NO"));
    Serial.println(F("     - or it is held down, stuck, or shorted to GND"));
  }
}

// Stage 1. Reports only when the set of wrong inputs changes, so fixing one
// wire prints one update rather than a scrolling wall.
void restCheck(unsigned long now) {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < N; i++) {
    if (isActive(i)) mask |= (uint8_t)(1 << i);
  }

  if (mask != g_wrongMask) {
    g_wrongMask = mask;
    if (mask) {
      Serial.println(F("NOT READY -- these read wrong with nothing touched:"));
      for (uint8_t i = 0; i < N; i++) {
        if (mask & (1 << i)) printRestCause(i);
      }
      Serial.println(F("Checking again continuously..."));
      Serial.println();
    } else {
      Serial.println(F("All six read right at rest -- hold still a moment..."));
    }
  }

  if (mask == 0 && now - g_lastChange >= REST_QUIET_MS) {
    g_ready = true;
    for (uint8_t i = 0; i < N; i++) g_count[i] = 0;
    Serial.println(F("READY -- every input reads right at rest."));
    Serial.println(F("Now press each button and trip each switch, one at a time."));
    Serial.println();
    if ((long)(now - g_longBeepUntil) >= 0) {
      tone(PIN_BUZZER, 1500, 300);
      g_shortUntil = now + 300;
      g_holdHz = 0;
    }
  }
}

// Keeps the extend / retract tone going while one is held, and silences it
// on release. Only after READY: before that a miswired button can read as
// held with nobody touching it, and would drone the whole time.
void holdToneTick(unsigned long now) {
  if ((long)(now - g_longBeepUntil) < 0 || (long)(now - g_shortUntil) < 0) return;

  uint16_t want = 0;
  if (g_ready) {
    if      (isActive(EXTEND))  want = EXTEND_HZ;
    else if (isActive(RETRACT)) want = RETRACT_HZ;
  }
  if (want == g_holdHz) return;
  if (want) tone(PIN_BUZZER, want);
  else      noTone(PIN_BUZZER);
  g_holdHz = want;
}

void startRestCheck() {
  g_ready = false;
  g_wrongMask = 0xFF;
  g_lastChange = millis();
  Serial.println(F("Rest check: don't touch anything yet."));
}

void setup() {
  // The bridge first, before anything else can happen.
  for (uint8_t i = 0; i < sizeof(BRIDGE_PINS); i++) {
    pinMode(BRIDGE_PINS[i], OUTPUT);
    digitalWrite(BRIDGE_PINS[i], LOW);
  }
  pinMode(PIN_BUZZER, OUTPUT);

  for (uint8_t i = 0; i < N; i++) {
    pinMode(PINS[i], INPUT_PULLUP);
  }
  delay(20);
  const unsigned long now = millis();
  for (uint8_t i = 0; i < N; i++) {
    g_raw[i] = g_stable[i] = digitalRead(PINS[i]);
    g_changed[i] = now;
    g_count[i] = 0;
  }

  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("=== switch_test -- motor bridge held OFF ==="));
#if LIMITS_NORMALLY_CLOSED
  Serial.println(F("Limits expected NC (COM-NC to GND)."));
#else
  Serial.println(F("Limits expected NO (COM-NO to GND)."));
#endif
  printAll();
  startRestCheck();
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    printAll();
    startRestCheck();
  }

  const unsigned long now = millis();
  for (uint8_t i = 0; i < N; i++) {
    const bool raw = digitalRead(PINS[i]);
    if (raw != g_raw[i]) { g_raw[i] = raw; g_changed[i] = now; continue; }
    if (raw == g_stable[i] || now - g_changed[i] < DEBOUNCE_MS) continue;

    g_stable[i] = raw;
    g_lastChange = now;
    if (g_ready && isActive(i)) g_count[i]++;
    printInput(i);

    if (i == HOME && isActive(i)) {
      tone(PIN_BUZZER, 2000, HOME_BEEP_MS);
      g_longBeepUntil = now + HOME_BEEP_MS;
      g_holdHz = 0;
    } else if (i == HOME || i == EXTEND || i == RETRACT) {
      // Home's release is silent so it cannot cut the 2 s beep short;
      // extend and retract are handled by holdToneTick().
    } else if ((long)(now - g_longBeepUntil) >= 0) {
      // High note going active, low note going back -- unless the home beep
      // is still sounding, which a short one would cut off.
      tone(PIN_BUZZER, isActive(i) ? 2000 : 1000, 60);
      g_shortUntil = now + 60;
      g_holdHz = 0;
    }
  }

  if (!g_ready) restCheck(now);
  holdToneTick(now);
}
