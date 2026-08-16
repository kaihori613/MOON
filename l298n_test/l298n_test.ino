
#define HAS_KNOB    0
#define HAS_BUTTON  0

#define SWITCH_IS_MAINTAINED  1

const uint8_t PIN_ENA    = 9;    // speed, Timer1
const uint8_t PIN_IN1    = 6;    // direction pair, digitalWrite only
const uint8_t PIN_IN2    = 5;
const uint8_t PIN_SWITCH = 4;    // to GND, INPUT_PULLUP
const uint8_t PIN_POT    = A0;

// A backstop, not a limit you should be reaching. A full stroke on these
// actuators is well under this; if a run hits it, something did not stop.
const unsigned long MAX_RUN_MS = 30000;

// Reversing a loaded DC motor instantly is the worst moment the supply and
// the bridge will ever see. Coast between directions.
const uint16_t REVERSE_DEAD_MS = 200;

// Below this the actuator generally buzzes without breaking away, so the
// bottom of the range is off rather than a useless crawl. Measuring the real
// figure is the first job this sketch exists for.
const uint8_t SPEED_FLOOR = 60;

// How long a lowercase jog runs. Long enough to see the rod move, short
// enough that a jog into a failed cam is not a problem.
const uint16_t JOG_MS = 1000;

const uint8_t SPEED_STEP = 15;   // one press of '+' or '-'

const uint16_t TICK_MS = 1000;   // how often the elapsed time prints

// ---------------------------------------------------------------------------
//  Speed
// ---------------------------------------------------------------------------

uint8_t g_speed = 180;           // used when HAS_KNOB is 0

#if HAS_KNOB
uint8_t readKnob() {
  // Four samples: the ADC sits near a bridge switching amps, and one reading
  // jitters enough to make the motor audibly hunt.
  uint16_t raw = 0;
  for (uint8_t i = 0; i < 4; i++) raw += analogRead(PIN_POT);
  raw /= 4;

  if (raw < 40) return 0;
  return (uint8_t)constrain(map(raw, 40, 1023, SPEED_FLOOR, 255), 0, 255);
}
#endif

// ---------------------------------------------------------------------------
//  Switch
// ---------------------------------------------------------------------------

#if HAS_BUTTON
const uint8_t DEBOUNCE_MS = 25;

bool          g_swStable  = false;   // true = pressed / closed
bool          g_swLastRaw = false;
unsigned long g_swChanged = 0;
bool          g_swPressEdge = false; // set for one pass on each fresh press

// A switch already closed at reset, or still closed after an automatic stop,
// must not start the motor. It has to be opened first.
bool g_armed = false;

void updateSwitch() {
  g_swPressEdge = false;

  const bool raw = (digitalRead(PIN_SWITCH) == LOW);

  if (raw != g_swLastRaw) {
    g_swLastRaw = raw;
    g_swChanged = millis();
    return;
  }
  if (millis() - g_swChanged < DEBOUNCE_MS) return;

  if (raw != g_swStable) {
    g_swStable = raw;
    if (raw) g_swPressEdge = true;
  }

  // Armed whenever the switch is genuinely open -- not only on the edge into
  // open. Keying on the transition would leave a switch that was never touched
  // permanently unarmed, and the first press would be silently swallowed.
  if (!g_swStable) g_armed = true;
}
#endif

// ---------------------------------------------------------------------------
//  Motor
// ---------------------------------------------------------------------------

char          g_dirSel  = 'e';   // what the next run will drive
bool          g_running = false;
bool          g_jogging = false; // true = this run expires by itself
uint8_t       g_applied = 0;
unsigned long g_startMs = 0;
unsigned long g_stopAtMs = 0;    // only meaningful while g_jogging
unsigned long g_nextTick = 0;

// Accumulated running time each way. Without a sensor this is the only record
// of where the actuator has been, and it is what a stroke measurement is.
unsigned long g_totalMs[2] = {0, 0};   // [0] extend, [1] retract

uint8_t dirIndex(char d) { return d == 'e' ? 0 : 1; }

void motorOff() {
  // ENA first: cut the output stage before touching direction, so the INs
  // never change state with the bridge live.
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  g_applied = 0;
}

void motorApply(char dir, uint8_t speed) {
  // Direction set while the output stage is off, then speed. IN1 and IN2 must
  // always be opposites: equal levels are brake (both high) or coast (both
  // low), and neither of those turns the motor.
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_IN1, dir == 'e' ? HIGH : LOW);
  digitalWrite(PIN_IN2, dir == 'e' ? LOW  : HIGH);
  analogWrite(PIN_ENA, speed);
  g_applied = speed;
}

void startMotor(bool jog) {
  g_running  = true;
  g_jogging  = jog;
  g_startMs  = millis();
  g_stopAtMs = g_startMs + JOG_MS;
  g_nextTick = g_startMs + TICK_MS;

  motorApply(g_dirSel, g_speed);

  Serial.print(F("  RUN  "));
  Serial.print(g_dirSel == 'e' ? F("EXTEND") : F("RETRACT"));
  Serial.print(F("  speed ")); Serial.print(g_speed);
  Serial.println(jog ? F("  (jog)") : F("  (continuous -- 'x' stops)"));
}

void stopMotor(const __FlashStringHelper* why) {
  motorOff();

  const unsigned long elapsed = millis() - g_startMs;
  g_running = false;
  g_jogging = false;
  g_totalMs[dirIndex(g_dirSel)] += elapsed;

  Serial.print(F("  STOP ("));
  Serial.print(why);
  Serial.print(F(")  ran "));
  Serial.print(elapsed / 1000.0, 2);
  Serial.print(F(" s "));
  Serial.println(g_dirSel == 'e' ? F("extending") : F("retracting"));
}

void printTotals() {
  Serial.println(F("--- accumulated run time -------------------------------"));
  Serial.print(F("  extend  : ")); Serial.print(g_totalMs[0] / 1000.0, 2);
  Serial.println(F(" s"));
  Serial.print(F("  retract : ")); Serial.print(g_totalMs[1] / 1000.0, 2);
  Serial.println(F(" s"));

  // The two should match once the actuator has been run stop to stop each way
  // at the same speed. A persistent difference means the directions are not
  // symmetric -- gravity, or a linkage that binds one way -- and open-loop
  // timing will drift accordingly.
  Serial.println(F("  Stop-to-stop at the same speed, these should agree."));
  Serial.println(F("--------------------------------------------------------"));
}

// ---------------------------------------------------------------------------
//  Console
// ---------------------------------------------------------------------------

void printHelp() {
  Serial.println(F("--- commands -------------------------------------------"));
  Serial.println(F("  e / r    jog EXTEND / RETRACT, stops on its own"));
  Serial.println(F("  E / R    run continuously until 'x' or MAX_RUN_MS"));
  Serial.println(F("  x        stop now"));
  Serial.println(F("  + / -    speed up / down"));
  Serial.println(F("  s        accumulated run time each direction"));
  Serial.println(F("  z        zero the accumulated times"));
  Serial.println(F("  ?        this help"));
#if HAS_KNOB
  Serial.println(F("  Knob sets speed; + and - are ignored."));
#endif
#if HAS_BUTTON
  Serial.println(F("  The switch starts and stops."));
#endif
  Serial.println(F("--------------------------------------------------------"));
}

void startFromCommand(char dirChar, bool jog) {
  // Changing direction under load without the coast is the transition
  // REVERSE_DEAD_MS exists to prevent, so stop and settle before reversing.
  if (g_running) {
    const bool reversing = (dirChar != g_dirSel);
    stopMotor(reversing ? F("direction changed") : F("restarted"));
    if (reversing) delay(REVERSE_DEAD_MS);
  }

  g_dirSel = dirChar;

  if (g_speed == 0) {
    Serial.println(F("  speed is at zero -- turn it up first"));
    return;
  }
  startMotor(jog);
}

void adjustSpeed(int8_t delta) {
#if HAS_KNOB
  (void)delta;
  Serial.println(F("  speed comes from the knob -- + and - do nothing"));
#else
  const int next = (int)g_speed + delta;
  g_speed = (uint8_t)constrain(next, 0, 255);

  // Below the floor the actuator buzzes without moving, so snap to off rather
  // than leave it sitting in the dead band drawing current for nothing.
  if (g_speed < SPEED_FLOOR) g_speed = (delta < 0) ? 0 : SPEED_FLOOR;

  Serial.print(F("  speed = ")); Serial.println(g_speed);
  if (g_running) motorApply(g_dirSel, g_speed);
#endif
}

void handleCommand(char c) {
  switch (c) {
    case 'e': startFromCommand('e', true);  break;
    case 'r': startFromCommand('r', true);  break;
    case 'E': startFromCommand('e', false); break;
    case 'R': startFromCommand('r', false); break;

    case 'x':
      if (g_running) {
        stopMotor(F("commanded"));
      } else {
        Serial.println(F("  already stopped"));
      }
      break;

    case '+': adjustSpeed(SPEED_STEP);  break;
    case '-': adjustSpeed(-SPEED_STEP); break;

    case 's': printTotals(); break;

    case 'z':
      g_totalMs[0] = g_totalMs[1] = 0;
      Serial.println(F("  accumulated times zeroed"));
      break;

    case '?': printHelp(); break;

    default:
      // A typo while the motor is running is more likely a slip than an
      // intention, and stopping is the cheap outcome.
      if (g_running) stopMotor(F("unknown command"));
      Serial.println(F("  unknown -- ? for help"));
      break;
  }
}

void pollSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;
    handleCommand(c);           // case matters: E/R are the committed versions
  }
}

// ---------------------------------------------------------------------------

void setup() {
  // Outputs low before anything that can block, so a reset brings the bridge
  // down immediately rather than after the serial handshake.
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  motorOff();

#if HAS_BUTTON
  pinMode(PIN_SWITCH, INPUT_PULLUP);
#endif

  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println();
  Serial.println(F("=== L298N Test -- physical control, open loop ==="));
  Serial.println(F("Lowercase e/r jog. Uppercase E/R run until stopped."));
  Serial.println();
  Serial.println(F("No sensor. Position is not tracked and cannot be. What this"));
  Serial.println(F("measures is time -- which is enough for breakaway duty,"));
  Serial.println(F("stroke duration each way, and coast after stop."));
  Serial.println();
  Serial.println(F("ENA jumper off, or speed control does nothing. 5V_EN jumper"));
  Serial.println(F("off at 24 V, or the onboard regulator cooks."));
  Serial.println();
  Serial.println(F("Supply 24 V, current limit 3-5 A. If the CC light comes on,"));
  Serial.println(F("the supply is limiting and the motor is not getting 24 V."));
  Serial.println(F("Check the L298N heatsink after the first full strokes."));
  Serial.println();
  printHelp();
}

void loop() {
  pollSerial();

#if HAS_KNOB
  const uint8_t knob = readKnob();
  g_speed = knob;
#endif

#if HAS_BUTTON
  updateSwitch();
  #if SWITCH_IS_MAINTAINED
    if (g_swStable && g_armed && !g_running && g_speed > 0) {
      startMotor(false);        // held, so the release is the stop
    } else if (!g_swStable && g_running) {
      stopMotor(F("switch opened"));
      g_armed = false;
    }
  #else
    if (g_swPressEdge && g_armed) {
      if (g_running) {
        stopMotor(F("switch"));
        g_armed = false;        // released state re-arms it in updateSwitch
      } else if (g_speed > 0) {
        startMotor(false);
      } else {
        Serial.println(F("  speed is at zero -- turn it up first"));
      }
    }
  #endif
#endif

  if (!g_running) return;

  // --- the jog expiring is a normal stop, not an exception -------------------
  if (g_jogging && millis() >= g_stopAtMs) {
    stopMotor(F("jog complete"));
    return;
  }

  // --- backstop -------------------------------------------------------------
  if (millis() - g_startMs >= MAX_RUN_MS) {
    stopMotor(F("MAX_RUN_MS -- nothing else stopped it"));
    Serial.println(F("  A full stroke should be well short of this. Either a"));
    Serial.println(F("  cam did not cut, or the actuator is not moving."));
    return;
  }

  // Speed dropped to zero mid-run is a stop request like any other.
  if (g_speed == 0) {
    stopMotor(F("speed at zero"));
    return;
  }

  // --- live elapsed, so a stroke can be timed off the screen -----------------
  if (millis() >= g_nextTick) {
    g_nextTick += TICK_MS;
    Serial.print(F("    "));
    Serial.print((millis() - g_startMs) / 1000.0, 1);
    Serial.print(F(" s   speed "));
    Serial.println(g_applied);
  }

#if HAS_KNOB
  // Track the knob live so speed can be dialled in while it moves. Only write
  // on a real change -- small ADC jitter would otherwise reload the timer
  // constantly for no reason.
  if (g_speed != g_applied) motorApply(g_dirSel, g_speed);
#endif
}
