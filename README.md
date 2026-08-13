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
| Bench tests (3 sketches) | Ready to run — need only the Arduino and the reed switch. Not yet compiled |
| Actuator control system | Complete, compiles, unvalidated. Waiting on the power control unit |
| Simulator | Working — exercises the full state machine with no motor |
| Host yaw pointing | Written, never executed — no Python on the build machine yet |

The single actuator is assigned to **yaw**. The field test on 11 Aug 2026 showed
yaw needs tighter pointing than pitch, so pitch stays set by hand.

## Sketches

### Bench tests

Three sketches, all driven from the bench supply — none of them touches the
motor. Wiring for all three: reed switch to **D2** and **GND**, nothing else.

| sketch | question it answers |
|---|---|
| `reed_switch_test/` | Is the sensor clean? |
| `cam_switch_test/` | Do the end stops fire in the same place every time? |
| `actuator_sensor_bench_test/` | Both, condensed — plus scale, the config block, and a go/no-go gate |

**`reed_switch_test/`** — one pulse per magnet pass, or more? The interrupt
records every falling edge and rejects nothing; debounce is applied afterwards
in `loop()`, so the report can show what it *would* have discarded. A sensor
that needs heavy filtering to look clean is one about to lose counts at speed,
and a sketch that hides its rejects will never show you that. Prints a gap
histogram — one cluster is healthy, two means double-counting. Debounce is
adjustable live with `d`, no reflashing between guesses. Also measures the
noise floor with the actuator stopped, where every edge is electrical pickup.

**`cam_switch_test/`** — homing drives into the retract stop and calls that
zero, so if the stop moves, the entire coordinate system moves with it. Keeps
statistics per end (they are different switches, with no reason to assume equal
repeatability) and cross-checks that both directions measure the same travel.

Because the cam switches cut motor current internally and are not wired to the
Arduino, "the switch fired" is inferred from pulses stopping — which is also
exactly what a sensor dropout looks like. So the sketch asks you to confirm the
supply current after every stroke. You are the missing sensor.

**`actuator_sensor_bench_test/`** — the acceptance gate. Everything above in one
session, plus millimetres per count (enter the measured stroke with `m`),
degrees per count (enter the moment arm with `a`), a paste-ready
`ActuatorConfig.h` block, and a PASS/FAIL line per criterion so the answer to
"is this good enough to close a loop around?" is stated rather than eyeballed.

Start here. Drop to the focused sketches when this one reports a problem.

### `actuator_system/`

Closed-loop position control. Homing against the retract hard stop, travel
calibration, absolute and relative moves, soft limits, stall/jam detection,
and EEPROM position persistence across power cycles. Driven from a serial
console — type `?` for commands.

Supports three motor drivers, selected in `ActuatorConfig.h`:
PWM+DIR (Cytron), dual PWM (BTS7960), or a relay pair.

### `host/`

Python, runs on the PC over USB. Homes the actuator, computes where GOES-18 is
from the site coordinates, drives yaw there, then hands the keyboard over for
manual peaking with the arrow keys.

The manual step stores a **trim** — an offset added to every computed target —
rather than a position, so re-homing or recomputing does not discard it. GOES-18
is geostationary, so from a fixed site the look angle never changes: peak once
and the trim is a permanent site correction.

The satellite math lives here rather than in the sketch because it wants
floating-point trig and a config file. See [host/README.md](host/README.md) for
the linkage calibration procedure.

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

- **Nothing in `host/` has ever been executed** — there is no Python on the
  build machine. `test_geometry.py` was written alongside the math but has not
  been run, so treat the pointing angles as unchecked until it passes.
- Yaw resolution is unknown until `mm_per_count` is measured on the bench. One
  reed count is the floor on pointing accuracy; if it turns out coarser than the
  link needs, the fix is a longer moment arm on the linkage, not software.
- Reed noise rejection is debounce-only. A PWM bridge switching amps near an
  unshielded sensor run will inject counts that permanently corrupt position.
  Wants an RC filter and a stronger pull-up.
- `zeroHere()` keeps the learned travel figure after moving the origin, so the
  soft limits no longer line up with the mechanical stops.
- `jog()` returns void, so the console reports a jog it may have ignored.
- Console argument parsing accepts garbage as `0`.
- No off-target tests for the state machine.
