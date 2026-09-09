# MOON — Actuator Control

Arduino control code for an HTS TVRO linear actuator, closing the loop around
the actuator's internal reed-switch position sensor.

## Status

**The sensor has been measured and the motor has been driven — but never both
at once.** Everything that closes the loop between them is still unvalidated,
and the timing constants stay placeholders until the bench measurements
replace them. `reed_switch_test/` now drives and counts in the same run, which
is the first sketch here that does; that part has not been on hardware yet.

| Component | State |
|---|---|
| `reed_switch_test/` | **Run on hardware, sensor is clean.** The timed run added since is unrun |
| `l298n_test/`, `actuator_test/` | **Bring-up. The motor turned.** Superseded by `actuator_v1/` |
| `actuator_v1/` | **Compiles clean, 63% flash / 17% RAM on a 328P. Never run.** |
| `actuator_v1/` IMU + buzzer | **Written, never compiled.** No `arduino-cli` on the machine it was written on |
| Host yaw pointing | **`test_geometry.py` now passes, 21/21.** The serial path is still unrun |
| Host axis fit | **`test_axis_fit.py` passes, 24/24**, and the CLI runs end to end on synthetic sweeps |

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

Two questions, in order. **Is the sensor clean** — one pulse per magnet pass,
or more? The interrupt records every falling edge and rejects nothing; debounce
is applied afterwards in `loop()`, so the report can show what it *would* have
discarded. A sensor that needs heavy filtering to look clean is one about to
lose counts at speed, and a sketch that hides its rejects will never show you
that. Prints a gap histogram, tunes the debounce live with `d`, and measures
the noise floor with the actuator stopped, where every edge is electrical
pickup.

**And how far is one pulse?** `e 10000` drives extend for ten seconds, counts
the pulses that arrive while it does, and stops; you then measure how far the
rod moved and type `m 47.5`. Distance over pulses is mm per count — a property
of the mechanism, true at any duty — and distance over time is mm per second,
true only at the duty that run used. That first number is the floor on pointing
resolution and the one measurement nothing in this repo could produce.

**And `+50` / `-50` stop on the sensor instead of the clock** — same fingering
as `actuator_v1`, where `+` and `-` are counts and `e`/`r` are open-loop time.
Ten seconds out and ten seconds back does not return the rod to where it
started: the load is not symmetric, the two directions travel at different
rates, and that is the entire reason nothing here is positioned by stopwatch.
Fifty counts out and fifty counts back should return it, and what it misses by
is backlash on its own, with the speed difference taken out.

A count run reports its **overshoot**, which is the second number this bench
was missing. The motor is cut when the target count arrives and the carriage
keeps moving, so the overshoot is coast measured in the unit the controller
actually acts on — and it is the floor on where any move can land. `Config.h`
asks for `DEADBAND_COUNTS = 1`; if the overshoot at a given duty is larger than
that, a move at that duty can never settle inside the deadband. Dropping the
duty until it fits is how `SPEED_TRIM` gets found. Every run also prints coast
in counts *and* in milliseconds, which is `COAST_SETTLE_MS`.

The run stops itself if the pulses stop while the motor is still commanded on,
which is an internal cam cutting at the end of travel, and says so: the count
and the distance still agree, but the rod stopped before the window did. It
keeps counting for 400 ms after the current is cut, because the carriage does
not stop when the current does and that coast is part of the distance you are
about to measure. Per-pulse printing is suppressed for the duration — the
serial writes are what would overflow the ring buffer and cost the count.

Wiring: reed switch to **D2** and **GND**, plus the bridge — L298N on
**D9/D6/D5**, HW-039 on **D9/D10/D8**, set by `MOTOR_DRIVER`. The bridge is
held hard off except during a timed run, and claimed low in `setup()` before
the serial handshake: unconfigured driver inputs are floating inputs, and an
L298N will happily decide for itself what that means during a noise-floor test
whose whole premise is that the carriage is stationary.

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

Nothing here reads the reed — the pin is not even claimed. What this sketch
measures is time, which is enough for breakaway duty, stroke duration each way
and coast after stop, and none of which needs the sensor. Pulses against
distance is `reed_switch_test/`'s job.

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

## Pointing by gravity

The single biggest change since the bench sketches. Homing into the retract cam
is no longer how an absolute origin gets established.

**Why gravity and not a compass.** The yaw axis on this mount is *tilted* — a
TVRO polar mount points its axis at the celestial pole, so at Davis it sits
about 51.5° off vertical. Rotating about a tilted axis tilts the dish, and an
accelerometer measures tilt directly. In the sensor's own frame the gravity
vector traces a **circle** as the dish sweeps; the plane of that circle is
perpendicular to the rotation axis, and the angle around it is yaw, 1:1.

Error propagates as `sigma_yaw = sigma_tilt / sin(gamma)`, where gamma is the
angle between the axis and gravity. At Davis that is a 1.28× penalty, so 0.1°
of tilt noise costs 0.13° of yaw.

**What the budget actually is.** The 8 ft dish at 1694 MHz has a 5.08° beam:

| off boresight | loss |
|---|---|
| 0.5° | 0.12 dB |
| 1.0° | 0.46 dB |
| 2.0° | 1.86 dB |
| 2.54° | 3.00 dB |

So the working budget is about **1°**, and every magnetometer in the parts
drawer — the MMC5603, the GY-271, the MPU-9150's own AK8975 — misses it. Not
because they are bad parts: an 8 ft steel reflector is an enormous soft-iron
distorter, and while it rotates *with* the sensor (so an ellipsoid fit removes
it) the pier does not. The pier stays fixed in the earth frame while the sensor
sweeps through it, leaving a heading-dependent residual no static calibration
can reach. Gravity has no such problem. Keep the magnetometers for a mount
whose axis is vertical, where gravity has nothing to say.

**The absolute anchor is the SDR, not a compass.** The fit knows where the dish
points relative to itself. Tying that to the sky takes one peak on GOES-18,
stored as the trim — a better absolute reference than any magnetometer, and
already what `host/` does.

### Sweep length is the thing that matters

Fitting a circle to a **short arc** is famously ill-posed: over a few degrees an
arc is nearly a straight line, and a straight line lies in many planes. The
plane normal is the rotation axis, and yaw is arc length over radius — so a
short sweep mis-scales every angle derived from it. It fails *quietly*: the
wrong circle still passes through every point, so the residuals look fine.

Measured over ~2700 synthetic sweeps (median of 21 seeds each):

| arc swept | 0.2 mg noise | 1.0 mg noise |
|---|---|---|
| 10° | 2.2 % scale error | 76 % |
| 16° | 0.7 % | 7.0 % |
| 30° | 0.18 % | 1.1 % |
| 60° | 0.04 % | 0.24 % |
| 120° | 0.01 % | 0.06 % |

**Sweep as much of the travel as you can.** Sagitta grows with the square of
the arc while noise does not, so doubling the sweep is worth about four times
as much as quartering the noise. `fit_axis` reports a `conditioning` number
(sagitta over measured noise, both taken from the raw points so it stays honest
when the fit is not) and refuses to quote an error bar below 25 — because below
there the error stops merely growing and starts diverging, and an optimistic
error bar exactly where the method collapses is worse than none.

### On the bench

```
c                     home and learn the travel  (needed: the sweep uses it)
w                     step, stop, settle, read -- one row per sample
```

Capture the console, then:

```bash
python calibrate_axis.py --from-log sweep.txt --save
```

The sweep is deliberately **step-stop-read**. An accelerometer cannot be read
while the motor is running — it would measure the motor — and an 8 ft dish is a
large sail, so every reading is averaged over 64 samples and gated on batch
variance. Rows taken while the mount was moving are flagged and dropped.

Set `IMU_SENSOR` in `Config.h`. Both the **MPU-9150** and the
**FXOS8700** are supported behind one define, the same way `MOTOR_DRIVER` is.
The FXOS8700's accel is 14-bit against the MPU's 16, so it loses on raw
resolution — but resolution is not the limit here, noise and temperature drift
are, and the NXP part is the quieter one. Bench them against each other and
keep the winner; the temperature column in every sweep row exists for exactly
that comparison.

## The buzzer

An active buzzer on **D7**, with **D4** held low next door so it plugs into two
adjacent headers. It beeps on a finished move, on homing, on a captured
calibration sample, and holds a long tone on a fault.

This exists because the sweep has you standing at an 8 ft dish while the laptop
is indoors. Everything it says could be read off the serial console instead, if
you were in front of it. You are not. Idea taken from SARCnet's rotator, which
beeps as it captures calibration extremes for the same reason.

It is an **active** buzzer, not a passive one — it makes its own tone from DC,
so driving it is a `digitalWrite`. Timer0 is `millis()`, Timer1 and Timer2 are
motor PWM, and there is no spare timer to hand a `tone()` to.

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

- **`actuator_v1/` has never been compiled since the IMU and buzzer went in.**
  There is no `arduino-cli` on the machine this was written on. Brace and
  preprocessor balance were checked by script and the register maps came from
  the datasheets, but neither is a compiler and neither is silicon. Expect to
  fix something on the first build.
- The host serial path is still unrun. `link.py`'s new `read_gravity`,
  `declare_position` and `stream_command` have no hardware behind them yet;
  `calibrate_axis.py --from-log` is the tested path, and the live `--port`
  path joins it only after the parse.
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
  the fix is a longer moment arm on the linkage, not software. `reed_switch_test/`
  measures it now — timed run, count the pulses, measure the rod, `m <mm>` —
  but that has not been done yet. The gravity sweep below now gives
  counts→degrees directly, which is what pointing actually needs, so
  `mm_per_count` matters mainly to the triangle linkage model.
- **The sweep must cover most of the travel.** See the arc-length table under
  "Pointing by gravity" — a short sweep produces a confident-looking fit whose
  angle scale is quietly wrong, and the residuals do not warn you.
- Reed noise rejection is debounce-only, and the noise floor has only been
  measured with no bridge in the circuit — which is the one configuration where
  a clean result proves nothing. `actuator_v1/` carries the test over as `n` so
  it can be re-run with the driver powered; do that before trusting any move. If
  pickup appears, the fix is 4.7k pull-up to 5V, 220R in series with the reed,
  220nF to ground at the pin, and a shared ground that does not carry motor
  return current.
- **Coast has never been measured**, so `COAST_SETTLE_MS` is a guess. Set too
  short, every move is judged before the carriage has finished moving and the
  landing report lies. It is the first constant to nail down on the bench, and
  `reed_switch_test/` now reports it on every run — in milliseconds for this
  constant, and in counts for the deadband. Still needs doing.
- No off-target tests for the state machine, and no simulator any more, so
  there is currently no way to exercise it without hardware.
- **The degrees readout is uncalibrated and shows `?`.** `a`/`b` fix that in a
  couple of minutes with a compass, but it needs the actuator drivable first.
  The gravity path (`w` then `calibrate_axis.py`) supersedes it and needs no
  compass at all.
- **Accelerometer temperature drift has not been measured**, and it is the
  dominant real-world error in the gravity method. One mg of zero-g offset is
  0.057° of tilt; a part drifting ~1 mg/°C over a 30 °C day/night swing would
  eat most of the pointing budget. Every sweep row carries the die temperature
  so the coefficient can be extracted; nobody has extracted it yet.
- Flash sits at 63% with the LCD compiled out, 81% with it in. `Wire` and
  `snprintf`'s formatting machinery are most of that difference. Room to work
  either way, but not a lot once the display goes in.
