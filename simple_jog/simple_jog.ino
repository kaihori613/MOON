/*
  Simple Jog  --  the dumbest thing that could possibly work
  -----------------------------------------------------------
  Press a key, the motor runs for five seconds, the motor stops. No sensor, no
  watchdog, no speed control, nothing to misconfigure.

  This exists because driver_test.ino is too clever to debug with. Its pulse
  watchdog cuts the motor when the reed goes quiet -- which is correct
  behaviour, and exactly wrong when the reed is the thing that is not working:
  every run dies after ~2 seconds and you never find out whether five seconds
  of power would have moved anything. Nothing here looks at the reed at all.

  Full speed only. analogWrite(255) is a constant high rather than a chopped
  waveform, so the bridge output is clean DC -- a handheld meter can read it,
  and there is no PWM duty cycle in the way if the actuator is struggling to
  break away.

  WIRING
    Unchanged from driver_test:
      RPWM -> D9    LPWM -> D10    R_EN + L_EN -> D8    VCC -> 5V    GND -> GND

  BEFORE YOU RUN IT
    Current limit. A TVRO actuator wants 2-4 A to break away and more under
    load -- a supply limited near 1 A will hold the voltage down and the motor
    will sit there doing nothing while everything looks nominally correct. If
    the supply has a CC indicator, watch it: lit means it is limiting, and the
    number on the voltage display is not what the motor is getting.

  COMMANDS
    e   extend  for 5 seconds
    r   retract for 5 seconds
    x   stop now
*/

const uint8_t PIN_RPWM = 5;    // extend
const uint8_t PIN_LPWM = 6;   // retract
const uint8_t PIN_EN   = 7;    // R_EN and L_EN tied together

const unsigned long RUN_MS = 5000;

// Reversing a loaded DC motor instantly is the worst moment the supply and
// the bridge will ever see. Coast between directions.
const unsigned long REVERSE_DEAD_MS = 200;

unsigned long g_stopAtMs = 0;
bool          g_running  = false;
char          g_dir      = 0;   // 'e' or 'r', for the stop message

void motorStop() {
  analogWrite(PIN_RPWM, 0);
  analogWrite(PIN_LPWM, 0);
  digitalWrite(PIN_EN, LOW);
}

void startRun(char dir) {
  if (g_running && dir != g_dir) {
    motorStop();
    delay(REVERSE_DEAD_MS);
  }

  // Idle side to zero before the driven side, always -- both inputs high at
  // once is a shoot-through path straight across the bridge.
  if (dir == 'e') {
    analogWrite(PIN_LPWM, 0);
    digitalWrite(PIN_EN, HIGH);
    analogWrite(PIN_RPWM, 255);
  } else {
    analogWrite(PIN_RPWM, 0);
    digitalWrite(PIN_EN, HIGH);
    analogWrite(PIN_LPWM, 255);
  }

  g_dir     = dir;
  g_running = true;
  g_stopAtMs = millis() + RUN_MS;

  Serial.print(F("  -> "));
  Serial.print(dir == 'e' ? F("EXTEND") : F("RETRACT"));
  Serial.println(F(" at full speed for 5 s. Watch the rod and the supply."));
}

void endRun(const __FlashStringHelper* why) {
  motorStop();
  g_running = false;
  Serial.print(F("  stopped ("));
  Serial.print(why);
  Serial.println(F(")"));
}

void setup() {
  // Safe state before anything that could block, so a reset brings the bridge
  // down immediately rather than after the serial handshake.
  pinMode(PIN_EN,   OUTPUT);
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  motorStop();

  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println();
  Serial.println(F("=== Simple Jog ==="));
  Serial.println(F("  e   extend  5 s"));
  Serial.println(F("  r   retract 5 s"));
  Serial.println(F("  x   stop now"));
  Serial.println();
  Serial.println(F("Check the supply's current limit first -- 2-4 A, not 1.3."));
  Serial.println(F("If it has a CC light and that light comes on, the supply is"));
  Serial.println(F("limiting and the motor is not getting 24 V."));
}

void loop() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;

    if      (c == 'e' || c == 'E') startRun('e');
    else if (c == 'r' || c == 'R') startRun('r');
    else if (g_running)            endRun(F("stopped by key"));
    else                           Serial.println(F("  e / r / x"));
  }

  if (g_running && millis() >= g_stopAtMs) endRun(F("5 s elapsed"));
}
