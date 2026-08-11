# MOON — Actuator Control

Arduino control code for an HTS TVRO linear actuator, closing the loop around
the actuator's internal reed-switch position sensor.

## Status

**Nothing here has run on hardware yet.** Everything compiles in every
configuration, and the control logic can be exercised end-to-end in
simulation, but no line of this has driven a real motor. Treat the timing
constants as placeholders until the bench measurements replace them.

| Component | State |
|---|---|
| Sensor bench test | Ready to run — needs only the Arduino and the reed switch |
| Actuator control system | Complete, compiles, unvalidated. Waiting on the power control unit |
| Simulator | Working — exercises the full state machine with no motor |

## Sketches

### `actuator_sensor_bench_test/`

Counts reed-switch pulses while the actuator is driven from a bench DC supply.
Detects the end of each stroke automatically (pulses stop when the cam limit
switch cuts motor current) and prints a run summary with the measured pulse
rates and the config values derived from them.

Wiring: reed switch to **D2** and **GND**. Nothing else.

This is the source of every timing constant used downstream. Run it first.

### `actuator_system/`

Closed-loop position control. Homing against the retract hard stop, travel
calibration, absolute and relative moves, soft limits, stall/jam detection,
and EEPROM position persistence across power cycles. Driven from a serial
console — type `?` for commands.

Supports three motor drivers, selected in `ActuatorConfig.h`:
PWM+DIR (Cytron), dual PWM (BTS7960), or a relay pair.

## Two things worth knowing before reading the code

**The reed switch counts, it does not tell direction.** Position is tracked as
pulses signed by the direction last commanded, which is correct as long as
nothing back-drives the actuator while the motor is off.

**The cam limit switches are not wired to the Arduino.** They cut motor
current internally at both extremes. So "we reached the end" is inferred from
pulses stopping while motion is still commanded — the same signature as a jam,
which is why every stall stops the motor immediately.

## Configuration

Everything tunable lives in `actuator_system/ActuatorConfig.h`. That is the
only file that should need editing for a hardware change.

`SIMULATE_ACTUATOR` defaults to `1`. **Set it to `0` before connecting a real
actuator.**

## Simulator

With `SIMULATE_ACTUATOR 1`, a virtual carriage responds to whatever the driver
layer commands and feeds back reed pulses, stopping at its ends exactly the way
the real cam switches do. One jumper from pin 6 to pin 2 routes the fake pulses
through the real interrupt and the real debounce filter.

The `q` command reports true position against believed position — the one thing
the real bench can never show you. `j` jams the carriage, `n` injects noise
pulses, `p` moves it with no pulses at all.

Simulation passing is necessary, not sufficient. It says nothing about current
draw, real pulse timing, wiring, or EMI.

## Known open issues

- Reed noise rejection is debounce-only. A PWM bridge switching amps near an
  unshielded sensor run will inject counts that permanently corrupt position.
  Wants an RC filter and a stronger pull-up.
- `zeroHere()` keeps the learned travel figure after moving the origin, so the
  soft limits no longer line up with the mechanical stops.
- `jog()` returns void, so the console reports a jog it may have ignored.
- Console argument parsing accepts garbage as `0`.
- No off-target tests for the state machine.
