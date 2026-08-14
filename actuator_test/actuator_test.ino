const uint8_t PIN_RPWM   = 9;    // extend
const uint8_t PIN_LPWM   = 10;   // retract
const uint8_t PIN_REED   = 2;
const uint8_t PIN_BUTTON = 4;    // to GND, INPUT_PULLUP
const uint8_t PIN_SPEED  = A0;   // pot wiper
const uint8_t PIN_EN     = 8;
const uint16_t RUN_MS_MIN = 100;
const uint16_t RUN_MS_MAX = 5000;

const unsigned long MAX_HOLD_MS = 10000;

const uint16_t REVERSE_DEAD_MS = 200;

const uint8_t SPEED_FLOOR = 60;

uint16_t g_runMs = 2000;
char     g_dirSel = 'e';         // which way the button will drive

const unsigned long DEBOUNCE_US = 3000;   // as settled by reed_switch_test

volatile unsigned long g_pulses      = 0;
volatile unsigned long g_lastPulseUs = 0;

void onPulse() {
  const unsigned long now = micros();
  if (now - g_lastPulseUs < DEBOUNCE_US) return;
  g_lastPulseUs = now;
  g_pulses++;
}

unsigned long pulseCount() {
  noInterrupts();
  const unsigned long n = g_pulses;
  interrupts();
  return n;
}

// ---------------------------------------------------------------------------
//  Speed knob
// ---------------------------------------------------------------------------

uint8_t g_speed = 0;

uint8_t readSpeedKnob() {
  // Four samples: the ADC sits next to a bridge switching amps, and a single
  // reading jitters enough to make the motor audibly hunt.
  uint16_t raw = 0;
  for (uint8_t i = 0; i < 4; i++) raw += analogRead(PIN_SPEED);
  raw /= 4;

  if (raw < 40) return 0;                       // bottom of travel is off
  const uint16_t s = map(raw, 40, 1023, SPEED_FLOOR, 255);
  return (uint8_t)constrain(s, 0, 255);
}

// ---------------------------------------------------------------------------
//  Button -- debounced, and refuses to act on a press it did not see begin
// ---------------------------------------------------------------------------

const uint8_t BUTTON_DEBOUNCE_MS = 25;

bool          g_btnStable   = false;   // true = pressed
bool          g_btnLastRaw  = false;
unsigned long g_btnChangeMs = 0;

// A button already held at reset must not start the motor. Nothing is honoured
// until the pin has been seen released at least once.
bool g_btnArmed = false;

void updateButton() {
  const bool raw = (digitalRead(PIN_BUTTON) == LOW);

  if (raw != g_btnLastRaw) {
    g_btnLastRaw  = raw;
    g_btnChangeMs = millis();
    return;
  }
  if (millis() - g_btnChangeMs < BUTTON_DEBOUNCE_MS) return;

  g_btnStable = raw;

  // Armed whenever the button is genuinely up -- not only on the edge into it.
  // Checking the transition instead would leave a button that was never
  // touched permanently unarmed, since a button released at boot never
  // transitions to released, and the first press would be swallowed.
  if (!g_btnStable) g_btnArmed = true;
}

bool buttonHeld() { return g_btnStable && g_btnArmed; }

// ---------------------------------------------------------------------------
//  Motor
// ---------------------------------------------------------------------------

char          g_dir        = 0;    // 'e', 'r', or 0 for stopped
uint8_t       g_appliedSpeed = 0;
unsigned long g_startMs    = 0;
unsigned long g_startCount = 0;
unsigned long g_stopAtMs   = 0;    // timed jog only; 0 while button-driven
bool          g_byButton   = false;

// digitalWrite rather than analogWrite(0): the AVR core turns the PWM off on
// its way through, so the pin is genuinely low whichever timer drove it.
void motorOff() {
  digitalWrite(PIN_RPWM, LOW);
  digitalWrite(PIN_LPWM, LOW);
  digitalWrite(PIN_EN,   LOW);
  g_appliedSpeed = 0;
}

void motorApply(char dir, uint8_t speed) {
  // Idle side hard off before the driven side goes anywhere near on. With the
  // enables tied high there is no second line to fall back on -- these two
  // pins are the only thing between the supply and a shoot-through.
  if (dir == 'e') {
    digitalWrite(PIN_LPWM, LOW);
    digitalWrite(PIN_EN,   HIGH);
    analogWrite(PIN_RPWM, speed);
  } else {
    digitalWrite(PIN_RPWM, LOW);
    digitalWrite(PIN_EN,   HIGH);
    analogWrite(PIN_LPWM, speed);
  }
  g_appliedSpeed = speed;
}

void beginRun(char dir, bool byButton) {
  if (g_dir != 0 && g_dir != dir) {
    motorOff();
    delay(REVERSE_DEAD_MS);
  }

  g_dir        = dir;
  g_byButton   = byButton;
  g_startCount = pulseCount();
  g_startMs    = millis();
  g_stopAtMs   = byButton ? (g_startMs + MAX_HOLD_MS) : (g_startMs + g_runMs);

  Serial.print(F("  -> "));
  Serial.print(dir == 'e' ? F("EXTEND") : F("RETRACT"));
  Serial.print(byButton ? F("  (button held)") : F("  (timed jog)"));
  Serial.println(F("   watch the rod and the supply's CC light."));
}

void endRun(const __FlashStringHelper* why) {
  motorOff();

  const unsigned long moved   = pulseCount() - g_startCount;
  const unsigned long elapsed = millis() - g_startMs;

  g_dir      = 0;
  g_byButton = false;

  // Disarm, so a button still held when the run ends cannot restart it on the
  // next pass through loop(). Without this every stop is undone microseconds
  // later: the hold ceiling becomes a stutter rather than a limit, 'x' does
  // nothing, and a direction change reverses under load with no coast, because
  // beginRun's reversal guard only fires when g_dir is still set. The operator
  // has to let go and press again -- which is what a dead-man control means.
  g_btnArmed = false;

  Serial.print(F("  stopped ("));
  Serial.print(why);
  Serial.print(F(")  "));
  Serial.print(moved);
  Serial.print(F(" pulses in "));
  Serial.print(elapsed);
  Serial.print(F(" ms"));

  if (moved > 0 && elapsed > 0) {
    Serial.print(F("  -> "));
    Serial.print((moved * 1000.0) / elapsed, 1);
    Serial.print(F(" Hz"));
  }
  Serial.println();

  if (moved == 0) {
    Serial.println(F("    No pulses. If the rod moved, the sensor is the"));
    Serial.println(F("    problem, not the driver. If it did not, check the CC"));
    Serial.println(F("    light, then whether a cam has cut this direction."));
  }
}

// ---------------------------------------------------------------------------

void printHelp() {
  Serial.println(F("--- commands -------------------------------------------"));
  Serial.println(F("  e / r    select EXTEND / RETRACT (does not move)"));
  Serial.println(F("  j        timed jog in the selected direction"));
  Serial.println(F("  t <ms>   jog duration, 100-5000"));
  Serial.println(F("  x        stop now"));
  Serial.println(F("  p / z    reed pulse count / zero it"));
  Serial.println(F("  ?        this help"));
  Serial.println(F("  Speed comes from the knob. Hold the button to run."));
  Serial.println(F("--------------------------------------------------------"));
}

void handleCommand(char* line) {
  if (line[0] == '\0') return;

  const char cmd = tolower(line[0]);
  const char* arg = line + 1;
  while (*arg == ' ' || *arg == '\t') arg++;
  const bool hasArg = (*arg != '\0');
  const long argVal = hasArg ? atol(arg) : 0;

  switch (cmd) {
    case 'e':
    case 'r':
      // Selecting a new direction mid-run would reverse under load without
      // the coast, so stop first and let the operator press again.
      if (g_dir != 0) endRun(F("direction changed"));
      g_dirSel = cmd;
      Serial.print(F("direction = "));
      Serial.println(cmd == 'e' ? F("EXTEND") : F("RETRACT"));
      break;

    case 'j':
      if (g_speed == 0) {
        Serial.println(F("  knob is at zero -- turn it up first"));
        break;
      }
      if (g_dir == 0) beginRun(g_dirSel, false);
      break;

    case 't':
      if (!hasArg) {
        Serial.print(F("duration = ")); Serial.print(g_runMs);
        Serial.println(F(" ms"));
        break;
      }
      g_runMs = (uint16_t)constrain(argVal, (long)RUN_MS_MIN, (long)RUN_MS_MAX);
      Serial.print(F("duration = ")); Serial.print(g_runMs);
      Serial.println(F(" ms"));
      break;

    case 'x':
      if (g_dir != 0) endRun(F("commanded"));
      else            Serial.println(F("  already stopped"));
      break;

    case 'p':
      Serial.print(F("pulses = ")); Serial.print(pulseCount());
      Serial.print(F("   knob = ")); Serial.println(g_speed);
      break;

    case 'z':
      noInterrupts();
      g_pulses = 0;
      interrupts();
      Serial.println(F("pulse count zeroed"));
      break;

    case '?': printHelp(); break;

    default:
      // A typo while the motor is running is more likely a slip than an
      // intention, and stopping is the cheap outcome.
      if (g_dir != 0) endRun(F("unknown command"));
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
  // Outputs low before anything that can block. With the enables tied high,
  // these two pins are the whole of the off switch.
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
  motorOff();

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_REED,   INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_REED), onPulse, FALLING);

  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println();
  Serial.println(F("=== Actuator Test (tutorial wiring, knob + button) ==="));
  Serial.println(F("RPWM on D9, LPWM on D10. R_EN and L_EN on either +5V or"));
  Serial.println(F("D8 -- this sketch drives D8 and runs the same way with both."));
  Serial.println();
  Serial.println(F("Knob sets speed. Hold the button to run. 'e' and 'r' choose"));
  Serial.println(F("the direction and do not move anything by themselves."));
  Serial.println();
  Serial.println(F("Supply: 24 V, current limit 3-5 A. If the CC light comes on,"));
  Serial.println(F("the supply is limiting and the motor is not getting 24 V --"));
  Serial.println(F("that alone will stop it breaking away."));
  Serial.println();
  Serial.println(F("Start the actuator away from both ends."));
  Serial.println();
  printHelp();
}

void loop() {
  pollSerial();
  updateButton();

  const uint8_t knob = readSpeedKnob();
  g_speed = knob;

  // --- button: hold to run --------------------------------------------------
  if (buttonHeld()) {
    if (g_dir == 0 && knob > 0) {
      beginRun(g_dirSel, true);
    }
  } else if (g_byButton) {
    endRun(F("button released"));
    return;
  }

  if (g_dir == 0) return;

  // --- ceilings -------------------------------------------------------------
  if (millis() >= g_stopAtMs) {
    endRun(g_byButton ? F("held too long -- release and press again")
                      : F("time elapsed"));
    return;
  }

  // Turning the knob to zero mid-run is a stop request like any other.
  if (knob == 0) {
    endRun(F("knob at zero"));
    return;
  }

  // Track the knob live so speed can be dialled in while it moves. Only write
  // on a real change: analogWrite every pass is wasted work, and small ADC
  // jitter would otherwise reload the timer constantly.
  if (knob != g_appliedSpeed) motorApply(g_dir, knob);
}
