/*
  Actuator System -- Bench Test Console
  -------------------------------------
  Drives the HTS TVRO linear actuator from the Arduino and closes the loop
  around its internal reed-switch position sensor.

  This is the step after actuator_sensor_bench_test.ino. That sketch only
  watched pulses while you drove the actuator from a bench supply; this one
  takes over the motor as well, so before running it you should already know:

    - the reed switch produces clean, single pulses (no double counts), and
    - roughly how many pulses you get across the full stroke.

  BENCH PROCEDURE
    1. Motor leads to the driver output, driver supply from the bench PSU
       with the current limit set just above the actuator's running draw.
       Arduino and driver must share a ground.
    2. Open the Serial Monitor at 115200, line ending "Newline".
    3. Type  ?  for the command list.
    4. Type  e  and be ready to hit Enter (stop) -- confirm it extends and
       that POS counts up. Then  r  and confirm POS counts down.
       If POS counts the wrong way, swap the motor leads (not the reed wires).
    5. Type  c  to calibrate: it retracts into the near hard stop, calls that
       zero, then extends into the far stop and records the travel.
    6. Type  g 500  and similar to check it lands on target repeatably.
       Run the same target from both directions -- the difference between
       the two resting positions is your backlash/coast error.

  SAFETY ON THE BENCH
    - Keep the PSU current limit low enough that a jam trips it rather than
      cooking the actuator or the driver.
    - Bare Enter is the stop command. Hit it whenever anything looks wrong.
    - Nothing here can override the actuator's internal cam limit switches;
      they still cut motor current at both extremes independently of this
      code, which is exactly what STALL detection below listens for.
*/

#include "ActuatorSystem.h"
#include "ActuatorSim.h"

ActuatorSystem act;

uint8_t  g_speed        = SPEED_MAX;   // default speed for jogs and moves
bool     g_telemetry    = true;        // stream position while moving
uint32_t g_lastTelemetry = 0;
const uint16_t TELEMETRY_MS = 250;

char     g_line[32];
uint8_t  g_len = 0;

// ---------------------------------------------------------------------------

void printHelp() {
  Serial.println(F("--- commands -------------------------------------------"));
  Serial.println(F("  ?        this help"));
  Serial.println(F("  s        status"));
  Serial.println(F("  e / r    jog extend / retract (runs until stop or limit)"));
  Serial.println(F("  <Enter>  STOP  (also: x)"));
  Serial.println(F("  g <n>    go to absolute count n"));
  Serial.println(F("  m <n>    move relative n counts (n may be negative)"));
  Serial.println(F("  h        home: retract into the hard stop, call it 0"));
  Serial.println(F("  c        calibrate: home, then learn full travel"));
  Serial.println(F("  z        zero the counter at the current position"));
  Serial.println(F("  v <n>    set speed 0-255 (ignored on a relay driver)"));
  Serial.println(F("  t        toggle telemetry while moving"));
  Serial.println(F("  k        clear a fault"));
  Serial.println(F("  f        forget the saved position in EEPROM"));
#if SIMULATE_ACTUATOR
  Serial.println(F("--- simulator ------------------------------------------"));
  Serial.println(F("  q        truth report: real position vs what we believe"));
  Serial.println(F("  j        toggle jam (motor runs, carriage does not move)"));
  Serial.println(F("  n <n>    inject n noise pulses on the sensor line"));
  Serial.println(F("  w <n>    set the virtual travel to n counts"));
  Serial.println(F("  p <n>    shove the carriage to n with no pulses"));
#endif
  Serial.println(F("--------------------------------------------------------"));
}

void printStatus() {
  Serial.print(F("state="));    Serial.print(act.stateName());
  Serial.print(F("  pos="));    Serial.print(act.position());
  Serial.print(F("  target=")); Serial.print(act.target());

  Serial.print(F("  travel="));
  if (act.travelCounts() > 0) Serial.print(act.travelCounts());
  else                        Serial.print(F("? (run c)"));

  Serial.print(F("  homed="));  Serial.print(act.isHomed() ? F("yes") : F("NO"));
  Serial.print(F("  dir="));
  Serial.print(act.direction() == ActuatorSystem::EXTEND  ? F("ext") :
               act.direction() == ActuatorSystem::RETRACT ? F("ret") : F("--"));
  Serial.print(F("  pwm="));    Serial.print(act.pwm());
  Serial.print(F("  hz="));     Serial.print(act.pulseHz(), 1);
  Serial.print(F("  pulses=")); Serial.print(act.pulseTotal());

  if (act.hitHardLimit()) Serial.print(F("  [at hard stop]"));
  if (act.fault() != ActuatorSystem::FAULT_NONE) {
    Serial.print(F("  FAULT: "));
    Serial.print(act.faultName());
  }
  Serial.println();
}

#if SIMULATE_ACTUATOR
// The one thing the real bench can never show you: what the actuator is
// actually doing, next to what the controller thinks it is doing. Any drift
// between these two numbers is a counting bug or a lost pulse.
void printTruth() {
  const long believed = act.position();
  const long truth    = Sim.truePosition();

  Serial.print(F("SIM  true="));     Serial.print(truth);
  Serial.print(F("  believed="));    Serial.print(believed);
  Serial.print(F("  error="));       Serial.print(believed - truth);
  Serial.print(F("  travel="));      Serial.print(Sim.travel());

  if (Sim.atRetractStop()) Serial.print(F("  [at RETRACT stop]"));
  if (Sim.atExtendStop())  Serial.print(F("  [at EXTEND stop]"));
  if (Sim.jammed())        Serial.print(F("  [JAMMED]"));
  Serial.println();

  if (believed != truth) {
    Serial.println(F("     ^ drift. Counts were gained or lost -- if you did not"));
    Serial.println(F("       inject noise or teleport, that is a real bug."));
  }
}
#endif

// ---------------------------------------------------------------------------

void handleCommand(char* line) {
  // Bare Enter is the panic stop -- keep it first and unconditional.
  if (line[0] == '\0') {
    act.stop();
    Serial.println(F("STOP"));
    return;
  }

  const char cmd = tolower(line[0]);
  const char* arg = line + 1;
  while (*arg == ' ' || *arg == '\t') arg++;
  const bool hasArg = (*arg != '\0');
  const long argVal = hasArg ? strtol(arg, nullptr, 10) : 0;

  switch (cmd) {
    case '?': printHelp();   break;
    case 's': printStatus(); break;

    case 'x':
      act.stop();
      Serial.println(F("STOP"));
      break;

    case 'e':
      act.jog(ActuatorSystem::EXTEND, g_speed);
      Serial.println(F("jog EXTEND  (Enter to stop)"));
      break;

    case 'r':
      act.jog(ActuatorSystem::RETRACT, g_speed);
      Serial.println(F("jog RETRACT  (Enter to stop)"));
      break;

    case 'g':
      if (!hasArg) { Serial.println(F("usage: g <count>")); break; }
      if (act.moveTo(argVal, g_speed)) {
        Serial.print(F("seeking ")); Serial.println(act.target());
      } else {
        Serial.println(F("rejected -- not homed (run h) or faulted (run k)"));
      }
      break;

    case 'm':
      if (!hasArg) { Serial.println(F("usage: m <+/-count>")); break; }
      if (act.moveBy(argVal, g_speed)) {
        Serial.print(F("seeking ")); Serial.println(act.target());
      } else {
        Serial.println(F("rejected -- not homed (run h) or faulted (run k)"));
      }
      break;

    case 'h':
      Serial.println(F("homing: retracting into the hard stop..."));
      act.home();
      break;

    case 'c':
      Serial.println(F("calibrating: retract to zero, then extend to learn travel..."));
      act.calibrate();
      break;

    case 'z':
      act.zeroHere();
      Serial.println(F("counter zeroed here"));
      break;

    case 'v':
      if (!hasArg) { Serial.print(F("speed = ")); Serial.println(g_speed); break; }
      g_speed = (uint8_t)constrain(argVal, (long)SPEED_MIN, (long)SPEED_MAX);
      Serial.print(F("speed = ")); Serial.println(g_speed);
#if MOTOR_DRIVER == DRV_RELAY
      Serial.println(F("(relay driver: speed is ignored)"));
#endif
      break;

    case 't':
      g_telemetry = !g_telemetry;
      Serial.print(F("telemetry ")); Serial.println(g_telemetry ? F("on") : F("off"));
      break;

    case 'k':
      act.clearFault();
      Serial.println(F("fault cleared"));
      break;

    case 'f':
      act.forgetPosition();
      Serial.println(F("EEPROM cleared -- home again before absolute moves"));
      break;

#if SIMULATE_ACTUATOR
    case 'q': printTruth(); break;

    case 'j': {
      const bool j = !Sim.jammed();
      Sim.setJammed(j);
      Serial.print(F("sim jam "));
      Serial.println(j ? F("ON -- next move should fault as a STALL")
                       : F("off"));
      break;
    }

    case 'n':
      if (!hasArg) { Serial.println(F("usage: n <pulses>")); break; }
      Sim.injectNoise((uint8_t)constrain(argVal, 1L, 255L));
      Serial.print(F("injecting ")); Serial.print(argVal);
      Serial.println(F(" noise pulses -- watch pos drift away from truth (q)"));
      break;

    case 'w':
      if (!hasArg) { Serial.print(F("sim travel = ")); Serial.println(Sim.travel()); break; }
      Sim.setTravel(argVal);
      Serial.print(F("sim travel = ")); Serial.println(Sim.travel());
      break;

    case 'p':
      if (!hasArg) { Serial.println(F("usage: p <count>")); break; }
      Sim.teleport(argVal);
      Serial.print(F("carriage moved to ")); Serial.print(Sim.truePosition());
      Serial.println(F(" with no pulses -- the controller does not know. Run q."));
      break;
#endif

    default:
      Serial.println(F("unknown command -- ? for help"));
      break;
  }
}

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
    // Overlong lines just have their tail dropped; the command is still the
    // first character, so nothing dangerous can come out of a mashed keyboard.
  }
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }          // needed on native-USB boards, harmless on Uno

  act.begin();
#if SIMULATE_ACTUATOR
  Sim.begin(&act);
#endif

  Serial.println();
  Serial.println(F("=== Actuator System Bench Console ==="));
#if SIMULATE_ACTUATOR
  Serial.println(F("*** SIMULATION MODE -- no motor required ***"));
  #if SIM_LOOPBACK_PIN != 255
  Serial.print(F("*** jumper pin "));
  Serial.print(SIM_LOOPBACK_PIN);
  Serial.print(F(" to pin "));
  Serial.print(PIN_REED);
  Serial.println(F(" for the fake pulses ***"));
  #else
  Serial.println(F("*** pulses injected in software, no wire needed ***"));
  #endif
  Serial.println(F("*** set SIMULATE_ACTUATOR 0 before using real hardware ***"));
#endif
#if MOTOR_DRIVER == DRV_PWM_DIR
  Serial.println(F("driver: PWM + DIR"));
#elif MOTOR_DRIVER == DRV_DUAL_PWM
  Serial.println(F("driver: dual PWM (BTS7960 style)"));
#elif MOTOR_DRIVER == DRV_RELAY
  Serial.println(F("driver: relay pair (no speed control)"));
#endif

  if (act.isHomed()) {
    Serial.print(F("restored position from EEPROM: "));
    Serial.println(act.position());
    Serial.println(F("If the actuator was moved with the Arduino off, this is"));
    Serial.println(F("wrong -- run 'h' to re-home before any absolute move."));
  } else {
    Serial.println(F("position unknown -- run 'h' to home, or 'c' to calibrate."));
  }

  printHelp();
  printStatus();
}

void loop() {
#if SIMULATE_ACTUATOR
  Sim.update();          // before act.update(), so pulses land this pass
#endif
  act.update();
  pollSerial();

  static ActuatorSystem::State lastState = ActuatorSystem::IDLE;
  const ActuatorSystem::State now = act.state();

  // Telemetry while something is moving.
  if (g_telemetry && act.isBusy() && millis() - g_lastTelemetry >= TELEMETRY_MS) {
    g_lastTelemetry = millis();
    Serial.print(F("  pos="));  Serial.print(act.position());
    Serial.print(F("  hz="));   Serial.print(act.pulseHz(), 1);
    Serial.print(F("  pwm="));  Serial.println(act.pwm());
  }

  // Announce the end of every move, so an unattended run leaves a record.
  if (now != lastState && !act.isBusy()) {
    Serial.print(F("--> "));
    printStatus();
  }
  lastState = now;
}
