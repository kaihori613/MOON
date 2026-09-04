/*
  Actuator Test  --  physical control, open loop
  ------------------------------------------------
  Knob sets speed, switch starts and stops, keyboard picks the direction.
  Nothing reads the reed. The actuator's position is not known, not tracked,
  and not knowable from here -- the only thing this sketch measures is TIME.

  That is deliberate, and it is the whole point of this stage. Time and speed
  are enough to characterise the mechanism: how much duty it takes to break
  away, how long a full stroke lasts each way, how far it coasts after power
  is cut, how the current behaves. Every one of those is a number the closed
  loop will need, and none of them requires the sensor. Get them now, while
  there is nothing else that can be wrong.

  The reed comes next. Then position, then homing, then angle.

  WIRING
    RPWM  -> D9        LPWM -> D10        R_EN / L_EN -> +5V or D8
    VCC   -> Arduino 5V
    GND   -> Arduino GND *and* the motor supply negative. Both.
    B+/B- -> motor supply     M+/M- -> actuator motor leads

    Pot     -> outer legs to 5V and GND, wiper to A0
    Switch  -> one leg to D4, other leg to GND (internal pull-up; the pin
               reads LOW when pressed or closed)

    Set SWITCH_IS_MAINTAINED below to match the part you fitted.

  PINS 9 AND 10, NOT 5 AND 6
    Pins 5 and 6 are Timer0, which also runs millis(). The Arduino reference
    warns that analogWrite(0) may not fully turn the output off there. With the
    enables tied high those two pins are the whole of the off switch, so a
    residual duty on both at once is a shoot-through path -- and a BTS7960
    latched into overcurrent protection is indistinguishable from a dead
    module. Pins 9 and 10 are Timer1, where zero means zero. Stopping also uses
    digitalWrite(LOW), which detaches the PWM whichever timer drove it.

    D8 is driven high while running and low otherwise, so this runs unchanged
    whether the enables went to +5V or to a pin.

  THE THING TO BE CAREFUL ABOUT
    Open loop with a latching switch means the motor runs until something
    stops it. What normally stops it is a cam limit switch cutting current
    inside the actuator -- which is fine, and is how these things are built.
    But if a cam has failed, the actuator drives into its own mechanical stop
    and sits there at stall until the winding cooks.

    So: current limit on the supply, 3-5 A. Watch the ammeter. MAX_RUN_MS is a
    backstop, not a plan.

  COMMANDS
    e / r    select EXTEND / RETRACT -- does not move anything
    x        stop now
    z        zero the accumulated run times
    s        summary of accumulated time each direction
    ?        help
*/

// 0 = momentary pushbutton: press to start, press again to stop.
// 1 = maintained toggle switch: closed runs, open stops.
#define SWITCH_IS_MAINTAINED 0

const uint8_t PIN_RPWM   = 9;    // extend
const uint8_t PIN_LPWM   = 10;   // retract
const uint8_t PIN_EN     = 8;    // harmless if the enables went to +5V instead
const uint8_t PIN_SWITCH = 4;    // to GND, INPUT_PULLUP
const uint8_t PIN_POT    = A0;

// A backstop, not a limit you should be reaching. A full stroke on these
// actuators is well under this; if a run hits it, something did not stop.
const unsigned long MAX_RUN_MS = 30000;

// Reversing a loaded DC motor instantly is the worst moment the supply and
// the bridge will ever see. Coast between directions.
const uint16_t REVERSE_DEAD_MS = 200;

// Below this the actuator generally buzzes without breaking away, so the
// bottom of the knob's travel is off rather than a useless crawl. Measuring
// the real figure is the first job this sketch exists for -- see the header.
const uint8_t SPEED_FLOOR = 60;

const uint16_t TICK_MS = 1000;   // how often the elapsed time prints

// ---------------------------------------------------------------------------
//  Speed knob
// ---------------------------------------------------------------------------

uint8_t g_speed = 0;

uint8_t readKnob() {
  // Four samples: the ADC sits near a bridge switching amps, and one reading
  // jitters enough to make the motor audibly hunt.
  uint16_t raw = 0;
  for (uint8_t i = 0; i < 4; i++) raw += analogRead(PIN_POT);
  raw /= 4;

  if (raw < 40) return 0;
  return (uint8_t)constrain(map(raw, 40, 1023, SPEED_FLOOR, 255), 0, 255);
}

// ---------------------------------------------------------------------------
//  Switch
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
//  Motor
// ---------------------------------------------------------------------------

char          g_dirSel = 'e';    // what the switch will drive
bool          g_running = false;
uint8_t       g_applied = 0;
unsigned long g_startMs = 0;
unsigned long g_nextTick = 0;

// Accumulated running time each way. Without a sensor this is the only record
// of where the actuator has been, and it is what a stroke measurement is.
unsigned long g_totalMs[2] = {0, 0};   // [0] extend, [1] retract

uint8_t dirIndex(char d) { return d == 'e' ? 0 : 1; }

void motorOff() {
  digitalWrite(PIN_RPWM, LOW);
  digitalWrite(PIN_LPWM, LOW);
  digitalWrite(PIN_EN,   LOW);
  g_applied = 0;
}

void motorApply(char dir, uint8_t speed) {
  // Idle side hard off before the driven side goes anywhere near on. If the
  // enables are tied high there is no second line to fall back on -- these two
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
  g_applied = speed;
}

void startMotor() {
  g_running  = true;
  g_startMs  = millis();
  g_nextTick = g_startMs + TICK_MS;

  motorApply(g_dirSel, g_speed);

  Serial.print(F("  RUN  "));
  Serial.print(g_dirSel == 'e' ? F("EXTEND") : F("RETRACT"));
  Serial.print(F("  speed ")); Serial.println(g_speed);
}

void stopMotor(const __FlashStringHelper* why) {
  motorOff();

  const unsigned long elapsed = millis() - g_startMs;
  g_running = false;
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

void printHelp() {
  Serial.println(F("--- commands -------------------------------------------"));
  Serial.println(F("  e / r    select EXTEND / RETRACT (does not move)"));
  Serial.println(F("  x        stop now"));
  Serial.println(F("  s        accumulated run time each direction"));
  Serial.println(F("  z        zero the accumulated times"));
  Serial.println(F("  ?        this help"));
  Serial.println(F("  Knob sets speed. The switch starts and stops."));
  Serial.println(F("--------------------------------------------------------"));
}

void handleCommand(char c) {
  switch (c) {
    case 'e':
    case 'r':
      // Changing direction under load without the coast is the transition
      // REVERSE_DEAD_MS exists to prevent, so stop and make the operator
      // start again rather than reversing on the spot.
      if (g_running) {
        stopMotor(F("direction changed"));
        g_armed = false;
        delay(REVERSE_DEAD_MS);
      }
      g_dirSel = c;
      Serial.print(F("  direction = "));
      Serial.println(c == 'e' ? F("EXTEND") : F("RETRACT"));
      break;

    case 'x':
      if (g_running) {
        stopMotor(F("commanded"));
        g_armed = false;      // the switch must be cycled before it runs again
      } else {
        Serial.println(F("  already stopped"));
      }
      break;

    case 's': printTotals(); break;

    case 'z':
      g_totalMs[0] = g_totalMs[1] = 0;
      Serial.println(F("  accumulated times zeroed"));
      break;

    case '?': printHelp(); break;

    default:
      // A typo while the motor is running is more likely a slip than an
      // intention, and stopping is the cheap outcome.
      if (g_running) {
        stopMotor(F("unknown command"));
        g_armed = false;
      }
      Serial.println(F("  unknown -- ? for help"));
      break;
  }
}

void pollSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;
    handleCommand(tolower(c));
  }
}

// ---------------------------------------------------------------------------

void setup() {
  // Outputs low before anything that can block, so a reset brings the bridge
  // down immediately rather than after the serial handshake.
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
  motorOff();

  pinMode(PIN_SWITCH, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println();
  Serial.println(F("=== Actuator Test -- physical control, open loop ==="));
  Serial.println(F("Knob sets speed. Switch starts and stops. 'e' and 'r'"));
  Serial.println(F("choose the direction and do not move anything themselves."));
  Serial.println();
  Serial.print(F("Switch mode: "));
  Serial.println(SWITCH_IS_MAINTAINED ? F("maintained (closed = run)")
                                      : F("momentary (press = toggle)"));
  Serial.println();
  Serial.println(F("No sensor. Position is not tracked and cannot be. What this"));
  Serial.println(F("measures is time -- which is enough for breakaway duty,"));
  Serial.println(F("stroke duration each way, and coast after stop."));
  Serial.println();
  Serial.println(F("Supply 24 V, current limit 3-5 A. If the CC light comes on,"));
  Serial.println(F("the supply is limiting and the motor is not getting 24 V."));
  Serial.println();
  printHelp();
}

void loop() {
  pollSerial();
  updateSwitch();

  const uint8_t knob = readKnob();
  g_speed = knob;

  // --- start / stop from the switch ----------------------------------------
#if SWITCH_IS_MAINTAINED
  if (g_swStable && g_armed && !g_running && knob > 0) {
    startMotor();
  } else if (!g_swStable && g_running) {
    stopMotor(F("switch opened"));
  }
#else
  if (g_swPressEdge && g_armed) {
    if (g_running) {
      stopMotor(F("switch"));
      g_armed = false;              // released state re-arms it in updateSwitch
    } else if (knob > 0) {
      startMotor();
    } else {
      Serial.println(F("  knob is at zero -- turn it up first"));
    }
  }
#endif

  if (!g_running) return;

  // --- backstop -------------------------------------------------------------
  if (millis() - g_startMs >= MAX_RUN_MS) {
    stopMotor(F("MAX_RUN_MS -- nothing else stopped it"));
    g_armed = false;
    Serial.println(F("  A full stroke should be well short of this. Either a"));
    Serial.println(F("  cam did not cut, or the actuator is not moving."));
    return;
  }

  // Turning the knob to zero mid-run is a stop request like any other.
  if (knob == 0) {
    stopMotor(F("knob at zero"));
    g_armed = false;
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

  // Track the knob live so speed can be dialled in while it moves. Only write
  // on a real change -- small ADC jitter would otherwise reload the timer
  // constantly for no reason.
  if (knob != g_applied) motorApply(g_dirSel, knob);
}
