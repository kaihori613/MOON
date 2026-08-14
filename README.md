# MOON — Actuator Control

Arduino control code for an HTS TVRO linear actuator, closing the loop around
the actuator's internal reed-switch position sensor.

## Status

**The sensor has been measured. Nothing has driven the motor yet.**
`reed_switch_test/` has run on the real actuator; everything downstream of it
is still unvalidated, and the timing constants stay placeholders until the
bench measurements replace them.

| Component | State |
|---|---|
| `reed_switch_test/` | **Run on hardware.** Sensor is clean — see below |
| `actuator_test/` | Written, not compiled. The HW-039 has not yet turned the motor |
| Actuator control system | Complete, compiles, unvalidated |
| Simulator | Working — exercises the full state machine with no motor |
| Host yaw pointing | Written, never executed — no Python on the build machine yet |

The single actuator is assigned to **yaw**. The field test on 11 Aug 2026 showed
yaw needs tighter pointing than pitch, so pitch stays set by hand.

## Sketches

### `reed_switch_test/`

Is the sensor clean? One pulse per magnet pass, or more? The interrupt records
every falling edge and rejects nothing; debounce is applied afterwards in
`loop()`, so the report can show what it *would* have discarded. A sensor that
needs heavy filtering to look clean is one about to lose counts at speed, and a
sketch that hides its rejects will never show you that. Prints a gap histogram,
tunes the debounce live with `d`, and measures the noise floor with the
actuator stopped, where every edge is electrical pickup.

Wiring: reed switch to **D2** and **GND**, nothing else. The motor runs from
the bench supply; this sketch does not drive it.

**Result on the real actuator:** two cleanly separated populations — 636 bounce
edges all under 0.2 ms, 176 real pulses no faster than 50 ms apart, and nothing
whatever in between. A 250× separation with the 3 ms debounce sitting almost
exactly at the geometric mean, so `REED_DEBOUNCE_US = 3000` stands with roughly
16× margin either side.

Note that the sketch calls this NOISY. It is wrong. `looksBimodal()` assumes
the travel cluster is the largest histogram bucket, which fails when a contact
bounces three or four times per closure, and the verdict then falls through to
a reject-share test that cannot see *where* the rejected gaps sit. Both want
fixing to key on cluster separation rather than counts.

### `actuator_test/`

Does the HW-039 bridge turn the motor? Speed knob on **A0**, hold-to-run button
on **D4**, direction chosen from the keyboard with `e` and `r`. The button is
the point: a serial monitor has no key-up event, so keyboard control means
timed jogs that expire on their own, while a released button stops the motor
that instant.

Uses the DeepBlue tutorial's wiring — enables tied high, control through the
two PWM inputs — because during bring-up every wire removed is one fewer
suspect. Two deliberate departures:

- **PWM on D9/D10, not D5/D6.** Pins 5 and 6 are Timer0, where `analogWrite(0)`
  may not fully turn the output off. With the enables tied high those two pins
  are the whole of the off switch, so residual duty on both at once is a
  shoot-through path — and a BTS7960 latched into overcurrent protection is
  indistinguishable from a dead module.
- **Stops with `digitalWrite(LOW)`**, which detaches the PWM whichever timer
  drove it.

It also drives **D8**, so it runs unchanged whether the enables are tied to +5V
or landed on a pin.

Reed pulses are counted and reported per run but nothing acts on them. A run
that moves the rod while reporting zero pulses says the driver is fine and the
sensor is the next problem.

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
- **End-stop repeatability has never been measured, and nothing here measures
  it any more.** `cam_switch_test/` and `actuator_sensor_bench_test/` were
  deleted in favour of the two sketches above. Homing calls the retract stop
  zero, so if that stop lands somewhere different each time, the whole
  coordinate system moves with it — recover those sketches from git history
  before trusting an absolute position.
- Yaw resolution is unknown until `mm_per_count` is measured. One reed count is
  the floor on pointing accuracy; if it turns out coarser than the link needs,
  the fix is a longer moment arm on the linkage, not software. The sketch that
  computed it went with the deletion above.
- Reed noise rejection is debounce-only, and the noise floor has only been
  measured with no bridge in the circuit — which is the one configuration where
  a clean result proves nothing. Re-run `n` once the HW-039 is switching amps
  near the sensor run. If pickup appears, the fix is 4.7k pull-up to 5V, 220R
  in series with the reed, 220nF to ground at the pin, and a shared ground that
  does not carry motor return current.
- `zeroHere()` keeps the learned travel figure after moving the origin, so the
  soft limits no longer line up with the mechanical stops.
- `jog()` returns void, so the console reports a jog it may have ignored.
- Console argument parsing accepts garbage as `0`.
- No off-target tests for the state machine.
