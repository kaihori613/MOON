# MOON — Actuator Control

Arduino control code for an HTS TVRO linear actuator, closing the loop around
the actuator's internal reed-switch position sensor.

## Status

**The sensor has been measured and the motor has been driven — but never both
at once.** Everything that closes the loop between them is still unvalidated,
and the timing constants stay placeholders until the bench measurements
replace them.

| Component | State |
|---|---|
| `reed_switch_test/` | **Run on hardware.** Sensor is clean — see below |
| `l298n_test/`, `actuator_test/` | **Bring-up. The motor turned.** Superseded by `actuator_v1/` |
| `actuator_v1/` | **Compiles clean, 63% flash / 17% RAM on a 328P. Never run.** |
| Host yaw pointing | Written, never executed — no Python on the build machine yet |

An earlier lineage, `actuator_system/`, was deleted in favour of v1. It was
written before any hardware existed and never ran, but it carried a simulator
that exercised the whole state machine with no motor attached, which v1 has no
equivalent of. Recover it from git history if that turns out to be missed.

Neither bring-up sketch read the reed while it drove, so the numbers those runs
were built to produce — breakaway duty, stroke time each way, coast after stop
— are the ones `actuator_v1/`'s placeholder constants are still waiting on.

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

### `actuator_v1/`

The bring-up sketches with the loop closed around them. The motor code is
unchanged from the code that actually turned the motor — same pins, same
ordering, same dead time. Two things are new, and only two.

**The input method.** The bring-up sketches offered a fixed one-second jog and
a speed that moved in steps of fifteen, which is fine for "does the bridge turn
the motor" and useless for "put the dish two counts to the left". Every command
here takes an argument instead: `+3` is three counts extend, `e 250` is a
quarter second of open-loop extend, `v 120` is a speed rather than eight
presses of `-`. Bare Enter is STOP from any state, which is the reason the
parser reads whole lines rather than single keys — the one input that must
never need an argument.

**The reed is read.** Position is pulses signed by the direction last
commanded, so homing, absolute moves, soft limits and stall detection all
follow. Relative moves work from boot; only absolute ones need an origin.

Corrections only ever run in the original direction of travel. A move that
settles short creeps on; a move that overshoots is reported and left alone.
Reversing to recover one count means taking up the linkage backlash again, so
the correction lands less predictably than the error it is fixing, and at trim
speed it will hunt. One count of honest residual beats one count of
oscillation — which makes the resolution of this system one reed count plus
coast. If that is coarser than yaw needs, the fix is a longer moment arm on the
linkage, not software.

**Zero sits at the middle of the stroke.** Homing still drives into the retract
cam — that is the only direction-unambiguous move there is, so it stays the
reference — but the origin is then shifted to the midpoint, so negative is
retracted, positive is extended, and the remaining headroom each way is obvious.
Set `ORIGIN_AT_MIDPOINT` to `0` to put zero back on the retract stop. The
midpoint cannot be located until the travel is known, so `c` is required; a
plain `h` before any calibration falls back to zero at the stop.

**It reads out in degrees.** A straight-line fit of heading against counts,
calibrated on the bench with `a` and `b` — sight the boom, type the heading,
drive along the stroke, sight again. That solves both constants and saves them.
Until it is calibrated the readout shows `?` rather than a confident wrong
number. The fit is anchored to the retract stop rather than to position zero,
so choosing the midpoint origin does not silently invalidate it.

This is a *readout*, not the pointing authority. `host/geometry.py` keeps the
physically-correct triangle model; the sketch carries the linear approximation
so the bench can show degrees with no PC attached.

**It remembers, but does not trust.** Position, travel and the angle fit are
saved to EEPROM on every stop. On boot they are loaded and displayed — and
`homed` stays false, so absolute moves still refuse until you home. What the
saved position is *for* is the next home: the difference between where the cam
actually trips and where the saved position said it would be is drift, printed
in counts. That is the same measurement as end-stop repeatability, and there is
currently no other way to see it. A saved position used to skip homing would be
silently wrong exactly when something back-drove the dish; used as a check, the
same number becomes an instrument.

**Optional 16x2 I2C LCD** on A4/A5, showing position, degrees, and state.
`USE_LCD` currently defaults to `0` — no display is wired yet, and compiling it
out costs 6 KB of flash and 361 bytes of RAM (63% → 81% flash with it on). Set
it to `1` when the hardware arrives.

The HD44780-behind-a-PCF8574 driver is written directly onto `Wire` rather than
pulling in a library — the several `LiquidCrystal_I2C` forks disagree about
constructor arguments, and the whole driver is under a hundred lines. If nothing
acknowledges at the I2C address, the sketch says so at boot and runs without it.

Set `MOTOR_DRIVER` in `Config.h` — L298N (default) or HW-039. Everything
tunable lives in that file; constants marked PLACEHOLDER have not been measured
yet. The `n` command re-runs the reed noise floor with the bridge powered and in
circuit, which is the configuration `reed_switch_test/` could never test.

The status line and the `s` / `h` / `c` / `g` / `k` / bare-Enter commands match
what `host/link.py` already speaks, so the host code can point at v1 unchanged.

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

**The cams have been confirmed to cut on the bench.** That matters more than it
sounds: the whole end-of-travel inference above was being taken on faith, and
homing drives deliberately into a stop expecting something to cut current
before the winding does. It also means a run into an end is a normal ending
rather than a stall against a mechanical stop.

What it does *not* establish is where they cut, or whether they cut in the same
place twice — see the open issue below.

## Configuration

Everything tunable lives in `actuator_v1/Config.h` — pins, speeds, tolerances,
timeouts. That is the only file that should need editing for a hardware change.
**Check `MOTOR_DRIVER` matches the module that is actually wired up** before the
first run; it defaults to `DRV_L298N`.

Constants marked PLACEHOLDER are guesses awaiting bench numbers. `COAST_SETTLE_MS`
is the load-bearing one: set too short, every move is judged before the carriage
has finished moving and the landing reports lie to you.

## Known open issues

- **Nothing in `host/` has ever been executed** — there is no Python on the
  build machine. `test_geometry.py` was written alongside the math but has not
  been run, so treat the pointing angles as unchecked until it passes.
- **End-stop repeatability has never been measured, and nothing here measures
  it any more.** The cams are now known to cut, which is what makes homing
  viable at all — but a cam that cuts reliably and a cam that cuts in the *same
  place* every time are different claims, and only the first has been checked.
  Homing calls the retract stop zero, so if that stop lands a few counts
  different each time, the whole coordinate system moves with it and every
  absolute target inherits the error. `cam_switch_test/` measured exactly this
  and was deleted; recover it from git history, or add a repeat-home command to
  `actuator_v1/` (it has no such command yet), before trusting an absolute
  position.
- Yaw resolution is unknown until `mm_per_count` is measured. One reed count is
  the floor on pointing accuracy; if it turns out coarser than the link needs,
  the fix is a longer moment arm on the linkage, not software. The sketch that
  computed it went with the deletion above.
- Reed noise rejection is debounce-only, and the noise floor has only been
  measured with no bridge in the circuit — which is the one configuration where
  a clean result proves nothing. `actuator_v1/` carries the test over as `n` so
  it can be re-run with the driver powered; do that before trusting any move. If
  pickup appears, the fix is 4.7k pull-up to 5V, 220R in series with the reed,
  220nF to ground at the pin, and a shared ground that does not carry motor
  return current.
- **Coast has never been measured**, so `COAST_SETTLE_MS` is a guess. Set too
  short, every move is judged before the carriage has finished moving and the
  landing report lies. It is the first constant to nail down on the bench.
- No off-target tests for the state machine, and no simulator any more, so
  there is currently no way to exercise it without hardware.
- **The degrees readout is uncalibrated and shows `?`.** `a`/`b` fix that in a
  couple of minutes with a compass, but it needs the actuator drivable first.
- Flash sits at 63% with the LCD compiled out, 81% with it in. `Wire` and
  `snprintf`'s formatting machinery are most of that difference. Room to work
  either way, but not a lot once the display goes in.
