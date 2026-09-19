// ===========================================================================
//  switch_test  --  do the buttons, the limit switches and the home switch
//                   read the way actuator_v2 expects?
// ===========================================================================
//
//  115200 baud, nothing to type. Prints every input once at start, then one
//  line each time any of them changes, and beeps on D11 so you can trip a
//  limit switch out at the pivot and hear it from the bench. Send any
//  character to reprint the lot.
//
//  No motor code. The L298N pins are all held LOW, so the bridge stays off
//  whether or not the 25 V is on -- including dead channel A's.
//
//  Pins are actuator_v2's (WIRING.md):
//    D3 extend, D4 retract, D7 kill   push buttons, NO to GND
//    A0 limit 1 (+15), A1 limit 2 (-15)   microswitches, NC to GND
//    A2 home                          microswitch, NO to GND

// Match actuator_v2/Config.h. 1 = limits wired COM-NC, 0 = wired COM-NO.
#define LIMITS_NORMALLY_CLOSED 1

const uint8_t PIN_BUZZER = 11;
const uint8_t BRIDGE_PINS[] = {9, 6, 5, 10, 8, 12};   // ENA IN1 IN2 ENB IN3 IN4

const uint8_t N = 6;
const uint8_t PINS[N]  = {3, 4, 7, A0, A1, A2};
const bool    IS_NC[N] = {false, false, false,
                          LIMITS_NORMALLY_CLOSED, LIMITS_NORMALLY_CLOSED, false};
const bool    IS_LIMIT[N] = {false, false, false, true, true, true};

const uint8_t DEBOUNCE_MS = 20;

static bool          g_stable[N];     // debounced raw level, true = HIGH
static bool          g_raw[N];
static unsigned long g_changed[N];
static uint16_t      g_count[N];      // times it has gone active

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

// Everything should be up / clear with nobody touching anything. Anything
// else at rest has a short list of causes.
void checkAtRest() {
  bool any = false;
  for (uint8_t i = 0; i < N; i++) {
    if (!isActive(i)) continue;
    any = true;
    printName(i);
    if (IS_LIMIT[i] && IS_NC[i]) {
      Serial.println(F(" reads TRIPPED at rest:"));
      Serial.println(F("   - it really is pressed, or"));
      Serial.println(F("   - it is wired COM-NO: set LIMITS_NORMALLY_CLOSED 0, or"));
      Serial.println(F("   - its wire or GND return is off (NC reads that as tripped)"));
    } else {
      Serial.println(F(" reads active at rest:"));
      Serial.println(F("   - held down, stuck, or wired to the NC contact, or"));
      Serial.println(F("   - its wire is shorted to GND"));
    }
  }
  if (!any) Serial.println(F("All up / clear at rest. Now press and trip each one."));
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
  checkAtRest();
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    printAll();
  }

  const unsigned long now = millis();
  for (uint8_t i = 0; i < N; i++) {
    const bool raw = digitalRead(PINS[i]);
    if (raw != g_raw[i]) { g_raw[i] = raw; g_changed[i] = now; continue; }
    if (raw == g_stable[i] || now - g_changed[i] < DEBOUNCE_MS) continue;

    g_stable[i] = raw;
    if (isActive(i)) g_count[i]++;
    printInput(i);
    // High note going active, low note going back.
    tone(PIN_BUZZER, isActive(i) ? 2000 : 1000, 60);
  }
}
