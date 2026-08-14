/*
  Driver Test  --  does the HW-039 actually drive the actuator?
  --------------------------------------------------------------
  First sketch in this repo that puts the Arduino in charge of the motor.
  Everything before it ran off the bench supply with a human doing the
  switching, so this is the step where a software bug can turn into a burnt
  winding. The whole design here is built around that.

  ONE QUESTION
    Does the bridge respond to the pins the way ActuatorConfig.h says it does?
    Extend on RPWM, retract on LPWM, both enables on one pin. Nothing about
    position, homing, or closing a loop -- that is actuator_system's job, and
    it comes after this.

  WHY EVERY MOVE IS TIMED
    There is no key-up event over a serial monitor. You cannot hold a key to
    jog and release it to stop -- the host sends a line and then nothing, so a
    "run until I say stop" command means the motor keeps running if the USB
    cable falls out, the monitor closes, or you get distracted. So every move
    here is a fixed-duration jog that expires on its own. Press again to go
    further. The motor cannot outlive a command.

  THE PULSE WATCHDOG
    While driving, the sketch watches the reed. If the motor is commanded and
    pulses stop, it cuts power immediately. That is a cam limit switch doing
    its job, or a jam, or the sensor dropping out -- indistinguishable from
    here, as always, but all three want the same response.

  BEFORE YOU RUN IT
    - Set the supply's current limit. Note the running current mid-travel
      first, then set the limit around 1.5x that. This is your real protection;
      everything in software is secondary.
    - Park the actuator away from both ends, so the first jog moves freely.
    - Keep a hand on the supply's output switch for the first few jogs.

  WIRING
    HW-039 RPWM -> D9      B+ / B-  -> motor supply
           LPWM -> D10     M+ / M-  -> actuator motor leads
           R_EN -> D8      (cams stay in series inside the actuator)
           L_EN -> D8
           VCC  -> 5V      <- logic supply, NOT motor voltage
           GND  -> GND
           R_IS, L_IS -> not connected

    Reed switch -> D2 and GND, as before.

    Ground: supply negative, B-, and Arduino GND meet at the supply terminal,
    not daisy-chained. The logic GND on this module is tied to B- internally,
    so motor return current will find your Arduino ground if you let it.

  IF IT GOES THE WRONG WAY
    Swap M+ and M- at the module. Do not invert it in software -- every other
    sketch in this repo assumes RPWM extends.
*/

// --- pins, matching actuator_system/ActuatorConfig.h ------------------------
const uint8_t PIN_RPWM = 5;    // extend
const uint8_t PIN_LPWM = 6;   // retract
const uint8_t PIN_EN   = 7;    // R_EN and L_EN tied together
const uint8_t PIN_REED = 2;

// --- limits -----------------------------------------------------------------
const uint16_t JOG_MS_MIN     = 50;
const uint16_t JOG_MS_MAX     = 5000;   // hard ceiling on a single move
const uint8_t  SPEED_MIN_USEFUL = 60;   // below this it usually just buzzes

// The actuator's slowest measured gap was ~500 ms, so a second of silence
// while driving means something actually stopped rather than merely slowed.
const uint16_t NO_PULSE_MS    = 1200;
// Breakaway takes a moment -- do not judge the pulse train until it has had a
// chance to start.
const uint16_t START_GRACE_MS = 1000;

// Reversing a loaded DC motor instantly puts the supply and the bridge through
// the worst moment they will ever see. Coast first.
const uint16_t REVERSE_DEAD_MS = 200;

// --- tunable at runtime -----------------------------------------------------
uint8_t  g_speed  = 120;   // matches SPEED_APPROACH -- deliberately not full
uint16_t g_jogMs  = 400;

// ---------------------------------------------------------------------------
//  Reed sensor
// ---------------------------------------------------------------------------

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
//  Motor
// ---------------------------------------------------------------------------

enum Dir : uint8_t { DIR_STOP = 0, DIR_EXTEND = 1, DIR_RETRACT = 2 };

uint8_t       g_dir        = DIR_STOP;
unsigned long g_moveEndMs  = 0;
unsigned long g_moveStart  = 0;
unsigned long g_startCount = 0;
unsigned long g_lastPulseMs = 0;
unsigned long g_lastSeen    = 0;   // pulse count at the last change

// Both PWM lines low and the enable deasserted. Called from setup() before
// anything else, on every stop, and on anything the parser does not recognise.
void motorStop() {
  analogWrite(PIN_RPWM, 0);
  analogWrite(PIN_LPWM, 0);
  digitalWrite(PIN_EN, LOW);
  g_dir = DIR_STOP;
}

void motorDrive(uint8_t dir, uint8_t speed) {
  // Never assert both inputs: that is a shoot-through path straight across the
  // bridge. Setting the idle side to zero before the driven side guarantees it
  // even if this is called while already moving.
  if (dir == DIR_EXTEND) {
    analogWrite(PIN_LPWM, 0);
    digitalWrite(PIN_EN, HIGH);
    analogWrite(PIN_RPWM, speed);
  } else {
    analogWrite(PIN_RPWM, 0);
    digitalWrite(PIN_EN, HIGH);
    analogWrite(PIN_LPWM, speed);
  }
  g_dir = dir;
}

void startJog(uint8_t dir) {
  // Coast through a reversal rather than slamming across it.
  if (g_dir != DIR_STOP && g_dir != dir) {
    motorStop();
    delay(REVERSE_DEAD_MS);
  }

  g_startCount  = pulseCount();
  g_lastSeen    = g_startCount;
  g_moveStart   = millis();
  g_lastPulseMs = g_moveStart;
  g_moveEndMs   = g_moveStart + g_jogMs;

  motorDrive(dir, g_speed);

  Serial.print(F("  -> "));
  Serial.print(dir == DIR_EXTEND ? F("EXTEND") : F("RETRACT"));
  Serial.print(F("  speed ")); Serial.print(g_speed);
  Serial.print(F("  for ")); Serial.print(g_jogMs);
  Serial.println(F(" ms"));
}

void endJog(const __FlashStringHelper* why) {
  motorStop();

  const unsigned long moved   = pulseCount() - g_startCount;
  const unsigned long elapsed = millis() - g_moveStart;

  Serial.print(F("  stopped ("));
  Serial.print(why);
  Serial.print(F(")  ")); Serial.print(moved);
  Serial.print(F(" pulses in ")); Serial.print(elapsed);
  Serial.print(F(" ms"));

  if (moved > 0 && elapsed > 0) {
    Serial.print(F("  -> "));
    Serial.print((moved * 1000.0) / elapsed, 1);
    Serial.print(F(" Hz"));
  }
  Serial.println();

  if (moved == 0) {
    Serial.println(F("  ! No pulses at all. Either nothing moved -- check the"));
    Serial.println(F("    supply, the current limit, and that a cam has not cut"));
    Serial.println(F("    this direction -- or the motor turned and the sensor"));
    Serial.println(F("    saw nothing, which is the more worrying one."));
  }

  Serial.print(F("  total ")); Serial.println(pulseCount());
}

// ---------------------------------------------------------------------------

void printHelp() {
  Serial.println(F("--- commands -------------------------------------------"));
  Serial.println(F("  e        jog EXTEND  for the current duration"));
  Serial.println(F("  r        jog RETRACT for the current duration"));
  Serial.println(F("  x        stop now"));
  Serial.println(F("  s <0-255> speed"));
  Serial.println(F("  t <ms>   jog duration (50 - 5000)"));
  Serial.println(F("  p        pulse count"));
  Serial.println(F("  z        zero the pulse count"));
  Serial.println(F("  ?        this help"));
  Serial.println(F("  anything else stops the motor"));
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
    case 'e': startJog(DIR_EXTEND);  break;
    case 'r': startJog(DIR_RETRACT); break;

    case 'x':
      if (g_dir != DIR_STOP) endJog(F("commanded"));
      else                   Serial.println(F("  already stopped"));
      break;

    case 's':
      if (!hasArg) {
        Serial.print(F("speed = ")); Serial.println(g_speed);
        break;
      }
      g_speed = (uint8_t)constrain(argVal, 0L, 255L);
      Serial.print(F("speed = ")); Serial.print(g_speed);
      if (g_speed > 0 && g_speed < SPEED_MIN_USEFUL) {
        Serial.print(F("   (likely too low to break away -- it will buzz)"));
      }
      Serial.println();
      break;

    case 't':
      if (!hasArg) {
        Serial.print(F("jog = ")); Serial.print(g_jogMs);
        Serial.println(F(" ms"));
        break;
      }
      g_jogMs = (uint16_t)constrain(argVal, (long)JOG_MS_MIN, (long)JOG_MS_MAX);
      Serial.print(F("jog = ")); Serial.print(g_jogMs);
      Serial.println(F(" ms"));
      break;

    case 'p':
      Serial.print(F("pulses = ")); Serial.println(pulseCount());
      break;

    case 'z':
      noInterrupts();
      g_pulses = 0;
      interrupts();
      Serial.println(F("pulse count zeroed"));
      break;

    case '?': printHelp(); break;

    default:
      // An unrecognised command while moving is more likely a typo or a
      // mashed key than an intention, and stopping is the cheap outcome.
      if (g_dir != DIR_STOP) endJog(F("unknown command"));
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
  // Safe state first, before Serial, before anything that could block. If the
  // board resets while the bridge is enabled, this is what brings it down.
  pinMode(PIN_EN,   OUTPUT);
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  motorStop();

  pinMode(PIN_REED, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_REED), onPulse, FALLING);

  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println();
  Serial.println(F("=== HW-039 Driver Test ==="));
  Serial.println(F("Every move is a timed jog -- press again to go further."));
  Serial.println(F("The motor stops on its own, on any unknown command, and if"));
  Serial.println(F("pulses stop while it is driving."));
  Serial.println();
  Serial.println(F("Set the supply's current limit before you start, and park the"));
  Serial.println(F("actuator away from both ends."));
  Serial.println();
  printHelp();
}

void loop() {
  pollSerial();

  if (g_dir == DIR_STOP) return;

  const unsigned long now = millis();

  // Live pulse feedback, and the timestamp the watchdog runs on.
  const unsigned long n = pulseCount();
  if (n != g_lastSeen) {
    g_lastSeen    = n;
    g_lastPulseMs = now;
  }

  if (now >= g_moveEndMs) {
    endJog(F("jog complete"));
    return;
  }

  // Past breakaway, a silent reed while driving means the motor is not turning
  // -- cam stop, jam, or sensor dropout. All three want the power off now.
  if (now - g_moveStart > START_GRACE_MS && now - g_lastPulseMs > NO_PULSE_MS) {
    endJog(F("PULSES STOPPED"));
    Serial.println(F("  Cut early. Cam limit, a jam, or the sensor dropping out"));
    Serial.println(F("  -- the Arduino cannot tell these apart. Check the supply"));
    Serial.println(F("  current: near zero means a cam cut it, still flowing"));
    Serial.println(F("  means the motor is loaded and the sensor went quiet."));
  }
}
