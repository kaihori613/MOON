# MOON — Actuator Control

Arduino control code for an HTS TVRO linear actuator driving the yaw axis of
a satellite dish, closed around an absolute encoder on the pivot.

**Rev B** replaced the reed-counting design with an AS5600 on the yaw pivot.
`actuator_v1/` still holds the reed version and is kept because it contains
the only motor code that has ever turned the motor. See [WIRING.md](WIRING.md)
for what changed and why.

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
| `actuator_v1/` | **Compiles clean, 63% flash / 17% RAM on a 328P. Never run.** Rev A |
| `as5600_test/` | **Rev B bring-up. Syntax-checked only, never run.** No motor code in it |
| `actuator_v2/` | **Rev B. Syntax-checked against stubs, never compiled for AVR, never run** |
| `host/metric.py`, `host/satdump.py` | **Run.** Exercised against synthetic statsd and a stand-in HTTP server; neither has seen a real receiver |
| `host/iq_snr.py` | **Estimators verified** to 0.06 dB against synthetic signals of known SNR. The capture function has never moved a sample |
| Rest of `host/` | Written, never executed — no Python on the build machine yet |

An earlier lineage, `actuator_system/`, was deleted in favour of v1. It was
written before any hardware existed and never ran, but it carried a simulator
that exercised the whole state machine with no motor attached, which v1 has no
equivalent of. Recover it from git history if that turns out to be missed.

Neither bring-up sketch read the reed while it drove, so the numbers those runs
were built to produce — breakaway duty, stroke time each way, coast after stop
— are the ones `actuator_v1/`'s placeholder constants are still waiting on.

The single actuator is assigned to **yaw**. The field test on 11 Aug 2026 showed
yaw needs tighter pointing than pitch, so pitch stays set by hand.

## Which input is in charge

Two sensors could plausibly be called the main input, and it is worth being
exact about it, because the answer decides what happens when either one goes
away.

**The AS5600 is the main input.** It is the only thing the actuator is ever
commanded against — every PWM value and every direction bit in the system
comes from it, and nothing else is in that loop.

**The signal metric never touches the motor.** All it can do is change *what
number the encoder loop is aiming at*. It is a setpoint generator, not a
feedback signal. Cruise control is the shape of it: the speedometer is the
feedback, and the speed you dialled in is the setpoint.

The asymmetry that settles it:

> The whole positioning system runs with **no receiver at all**. It cannot run
> for one second with **no encoder**.

Unplug the SDR and the dish still points, holds, respects its limits and
reports its angle. Lose the encoder and the axis is blind, so `actuator_v2`
raises `F_ENCODER` and stops — feeding a stale angle to the PID is worse than
not moving.

| | AS5600 | Signal metric |
|---|---|---|
| Rate | 50 Hz, continuous | ~0.2 Hz, on demand |
| Runs on | the Arduino | the host |
| Drives | PWM and direction, directly | the target angle, and nothing else |
| Available | always | only under lock |
| When it is lost | fault, motor stopped | hold the last target; the dish stays pointed |

### Where a target angle comes from

Three sources, in descending authority:

1. **Manual** — you typed `g -3.2`
2. **The stored trim** — what the signal search produced, once
3. **The geometry model** — computed from the site and 137.0°W

### For a geostationary bird, the metric is a calibration input

Not a runtime one. GOES-18 does not move relative to a fixed site, so the
search runs rarely, produces one number, and is then not needed again. Day to
day the system points from encoder plus geometry plus stored trim, with the
receiver contributing nothing to where the dish goes.

The metric's whole job is to correct source 3 into source 2, permanently.
`host/config.py` stores a trim rather than a position for exactly this reason:
a position is invalidated by the next recompute, a trim survives it.

So if one sensor is the heart of the machine, it is the encoder. The signal
path is what tells you, once, that the encoder's zero is aimed at the right
patch of sky.

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

### `as5600_test/`

Encoder bring-up, with no motor code in it at all, so it is safe to run with
the driver unpowered. Three questions in order.

**Is the magnet mounted right?** The AS5600 will happily report a
plausible-looking angle from a magnet that is too far, off centre, or axially
magnetised instead of diametrically. It reports all of that in `STATUS` and
`AGC`, and this shows both live. A reading that looks fine on the bench and
drifts once it is on the mount is almost always a magnet that was never in
spec — and there is no way to tell from the angle alone, which is the entire
reason this sketch exists.

**Which way does it count?** `b` starts a sweep, you move the boom by hand,
and each sample prints `counting UP` or `counting DOWN`. That answers
`ENCODER_INVERT`. Getting it wrong makes the loop positive feedback, and the
first move runs to a cam at whatever duty the PID asked for.

**Does the travel straddle the wrap?** `x` ends the sweep and reports the
span. If the range crosses the 0/4095 rollover it says so and tells you to
rotate the magnet on its hub. It also suggests an `ENCODER_ZERO_DEFAULT` at
the middle of the measured travel.

### `actuator_v2/`

Rev B. The inner loop of a cascade, and only the inner loop.

**This is not a cascaded PID, and the distinction matters when tuning.** A
cascaded PID is an outer PID whose output is the setpoint of an inner PID, both
with gains, the way a servo drive stacks position over velocity over current.
Here there is exactly ONE set of gains. The outer thing is a hill-climbing
search: no setpoint, no error term, no gains, and no way to have any, because
signal against angle is a peak rather than a ramp. Go looking for outer-loop
gains and you will be looking for something that does not exist.

A velocity inner loop is the one place a real cascade could go, and the encoder
cannot feed it: at 0.0879° per count and 50 Hz the smallest measurable velocity
is 4.4°/s, which the final approach is far below, so the feedback would be
mostly zeros with occasional spikes exactly where the loop needs it most.
Averaging over more ticks fixes the resolution and makes the inner loop slower
than the outer one, which defeats the point. What such a loop would buy —
linearising away stiction — is already what `KFF_FRICTION` does, with one
number instead of two more gains. If it is ever wanted anyway, the sensor for
it is the *reed*, timed period-between-pulses rather than counts-per-tick, so
resolution comes from the timer instead of from quantisation.

**It does not chase SNR, and that is deliberate.** Signal strength against
pointing angle is a peak, not a ramp: the same reading occurs on both sides of
it, so a controller fed SNR has no sign to act on and cannot know which way to
move. Peaking is a *search*. It belongs on the host, it runs on demand rather
than continuously — GOES-18 is geostationary, so from a fixed site the look
angle never changes — and it talks to this sketch by handing it target angles.
What runs here is a servo on angle, where the error does have a sign.

**Position is absolute, so homing stops being a prerequisite.** With it go the
EEPROM position, the travel calibration, the midpoint origin, the `a`/`b`
compass fit, and the direction-signed counting. Most of what was expensive in
v1 existed to work around not knowing where the boom was.

**A reference switch still earns its place, for a different job.** The encoder
is absolute; its *zero* is not. Zero is a raw count in EEPROM, and nothing in
the encoder can reveal that the magnet has crept on its hub or that the EEPROM
was wiped — every angle would simply be wrong by a constant, confidently, with
no symptom. That is the same shape of failure v1 had with a back-driven dish,
and it gets the same answer: a physical reference you can check against.

**It is a third switch, not one of the cams.** Reusing the −10° limit was the
first attempt and it was wrong. The cams are hard stops protecting the
antenna: homing into one drives deliberately into a safety device, wearing the
cam-to-lever alignment that *is* the protection; it destroys "SW− is tripped"
as an alarm, because that would also mean "we are homing"; and with the escape
diodes fitted, current is cut the instant the cam opens, so every home would
end with the mechanism hard-cut by a safety circuit rather than decelerating
under control. `actuator_v2` now treats driving into a cam as a **fault**.

The reference switch sits mid-travel, carries no safety duty, and is crossed
in transit. That turns checking from a procedure into **passive monitoring** —
the firmware captures every ordinary crossing and warns only when it drifts.

- `h` — deliberate slow pass in both directions, report drift, change nothing
- `H` — the same, but adopt the crossings and re-derive zero

**Two references, one per direction.** A microswitch's trip and release points
differ, so a crossing is only repeatable per direction; each is compared
against its own stored value. Their difference is the lobe width plus
hysteresis, a constant of the mechanism and a free diagnostic — if it changes,
the lever is bending or the cam is loose. Zero is derived from the *midpoint*
of the two, which cancels the hysteresis rather than inheriting whichever
direction happened to be captured.

**The reed is kept as a witness.** It measures nothing, but it catches two
failures neither sensor sees alone:

- reed silent while the motor is commanded on → stall, cam cut, or blown fuse
- reed pulsing while the boom does not move → **the linkage has broken**, and
  the motor is driving nothing

The second is the reason to keep it wired. An encoder on the pivot cannot tell
a broken coupling from a stalled motor; a reed on the motor side cannot tell
you where the boom is. Together they can do both.

#### Tuning the PID

A jack-driven dish is friction-dominated, which changes what the letters are
worth. Gains are live — `p`, `i`, `d`, `f` — so none of this needs a reflash,
and `t <deg>` runs a step and prints `ms,target,pos,duty` as CSV to plot.

Work in this order:

1. **`f` first, not `p`.** Set `kff` to the breakaway duty — the same number
   v1 called `SPEED_FLOOR`. Below it the motor draws current, heats up and
   does not turn, so every correction should start there and let the gains
   trim from there. On this plant the feedforward does more work than the
   integrator does.
2. **Then `p`.** Raise `kp` until a step either oscillates or overshoots
   badly, then halve it.
3. **`i` only if a real offset persists** outside the deadband after the move
   settles. Often it will not, and an integrator you do not need is an
   integrator that hunts.
4. **`d` last, and probably never.** Derivative on a 12-bit encoder reading a
   slow mechanism is mostly quantisation noise amplified. Add it only if
   overshoot survives step 2. It is taken on the *measurement*, not the error,
   so a new target from the host does not put a spike through the output.
5. **`w`** to save the zero and the gains.

Two things are already handled and should not be re-invented during tuning.
**Anti-windup**: integration stops whenever the output is saturated, which it
will be for most of every long move. **The deadband**: inside it the output is
zero and the integrator is dumped, because integral action across a mechanical
dead zone is the textbook limit cycle and the dish will hunt all night.

The deadband is the cheapest fix in the design, and it works because the
pointing requirement is loose. Half-power beamwidth is roughly `70·λ/D`, which
at 1694 MHz on a 1 m dish is about 12°, and pointing loss is about
`12·(θ/θ₃dB)²` dB — so 0.1 dB costs ±1.1° of error. **Set the deadband wider
than the measured backlash, not tighter.** A deadband several times the
backlash still lands inside a pointing error nobody can measure, and it is
what ends every move cleanly.

#### Protocol delta

`host/link.py` speaks v1's console. v2 keeps the same command letters and the
same `key=value` status format, with three differences:

- **`g <n>` takes degrees, not counts.** Status carries `unit=deg` so a host
  can tell which sketch it is talking to.
- **`h` no longer homes into a stop.** It runs a reference pass across the
  mid-travel switch and reports drift, changing nothing. `H` is the version
  that adopts the result.
- **`c` (calibrate) is an alias for `z`**, set-zero-here.

`link.py` has gained `move_to_deg()`, `unit_is_degrees()`, `set_zero()`,
`set_gain()` and `save_settings()` alongside `move_to()`, and `status()` now
parses both revisions. Only `metric.py`, `satdump.py` and `iq_snr.py` have
ever been executed; the rest of `host/` has not.

### `host/`

Python, runs on the PC over USB. Homes the actuator, computes where GOES-18 is
from the site coordinates, drives yaw there, then hands the keyboard over for
manual peaking with the arrow keys.

The manual step stores a **trim** — an offset added to every computed target —
rather than a position, so recomputing the look angle does not discard it. GOES-18
is geostationary, so from a fixed site the look angle never changes: peak once
and the trim is a permanent site correction.

The satellite math lives here rather than in the sketch because it wants
floating-point trig and a config file. See [host/README.md](host/README.md) for
the linkage calibration procedure.

## Three things worth knowing before reading the code

**The reed switch counts, it does not tell direction.** That is why Rev B
stopped using it for position. In `actuator_v1/` position is pulses signed by
the direction last commanded, which is correct only as long as nothing
back-drives the actuator while the motor is off — and wind on a dish does
exactly that, with no symptom. In `actuator_v2/` the reed measures nothing; it
witnesses, as described under `actuator_v2/` above.

**There are two different sets of cams, and only one is wired to the
Arduino.** The actuator has its own internal cams at the ends of the stroke,
which cut motor current themselves and are not connected to anything — so from
outside, "we reached the end" can only be inferred from the reed going quiet
while motion is still commanded, which is the same signature as a jam. That is
why a stall stops the motor immediately rather than trying to tell the two
apart. Separate from those, Rev B adds **cam switches at ±10°** on the pivot,
which do report to D3 and D4 as well as cutting current in hardware.

**The internal cams have been confirmed to cut on the bench.** The whole
end-of-travel inference above was being taken on faith before that, and it
also means a run into an end is a normal ending rather than a stall against a
mechanical stop. What it does not establish is *where* they cut, or whether
they cut in the same place twice.

That last point used to matter a great deal, because Rev A called one of those
stops zero. It no longer does: the encoder is absolute and its zero is checked
against the mid-travel reference switch instead, so end-stop repeatability has
stopped being load-bearing. Nothing in `actuator_v2/` ever drives into a stop
on purpose.

## Wiring

[WIRING.md](WIRING.md) is the pin map, the power scheme and the grounding.
Read the grounding section before wiring anything: the Arduino ground must
star at the supply negative rather than hang off the driver's GND terminal,
because that terminal carries the motor return current and the drop along it
lands on the reed input.

None of it has been built. The motor and reed rows describe hardware that has
run; the buttons, buzzer and IMU rows are reservations, so that the document
and `Config.h` cannot drift apart.

## Configuration

Everything tunable lives in `actuator_v2/Config.h` — pins, gains, tolerances,
timeouts. That is the only file that should need editing for a hardware change.
`actuator_v1/Config.h` is Rev A's and still carries `MOTOR_DRIVER`, which Rev B
does not have: the G2's two-pin interface is the only one v2 speaks.

**Set `ENCODER_INVERT` and `ENCODER_ZERO_DEFAULT` from an `as5600_test/` run
before enabling the loop.** An inverted encoder makes the PID positive
feedback, and the first move runs to a cam at full commanded duty.

Constants marked PLACEHOLDER are guesses awaiting bench numbers. In Rev B the
load-bearing ones are `KFF_DEFAULT`, which should be the measured breakaway
duty, and `DEADBAND_DEG`, which should be set *wider* than the measured
backlash rather than tighter — see the tuning notes above.

## Known open issues

- **Most of `host/` has never been executed** — there is no Python on the
  build machine. `metric.py`, `satdump.py` and `iq_snr.py` have been run
  against synthetic inputs, but `geometry.py`, `config.py`, `link.py` and
  `moon_yaw.py` have not, and `test_geometry.py` has never passed. Treat the
  pointing angles as unchecked until it does.
- **No part of `host/` has met real hardware.** The estimators were checked
  against synthetic signals and the metric sources against stand-in servers;
  none of it has seen a receiver, an SDR or a serial port.
- **End-stop repeatability no longer matters, and that is the main thing Rev B
  bought.** Rev A called the retract stop zero, so a stop that landed a few
  counts different each time moved the whole coordinate system with it. An
  absolute encoder has no such dependency: zero is a stored raw count, not a
  place the mechanism has to find. The issue is closed by the design rather
  than by a measurement.
- **Yaw resolution is now set by the encoder, not the mechanism**: 0.0879° per
  count, against a pointing requirement of roughly ±1.1° on a 12° beam. That is
  about 12× more resolution than the beam can use, so do not gear the encoder
  up — a belt would add backlash inside the feedback path in exchange for
  resolution nothing can see. `mm_per_count` is still worth measuring, but only
  to calibrate `LINKAGE_MIN_DEG` for the broken-linkage check.
- Reed noise rejection is debounce-only, and the noise floor has only been
  measured with no bridge in the circuit — which is the one configuration where
  a clean result proves nothing. `actuator_v2/` carries the test over as `n` so
  it can be re-run with the driver powered; do that before trusting any move. If
  pickup appears, the fix is 4.7k pull-up to 5V, 220R in series with the reed,
  220nF to ground at the pin, and a shared ground that does not carry motor
  return current.
- **Breakaway duty has never been measured**, and in Rev B it is `KFF_DEFAULT`
  — the first term to set during tuning, and the one the others are trimming
  on top of. `actuator_test/` measures it.
- **Backlash has never been measured**, and it sets `DEADBAND_DEG`. With the
  encoder on the pivot it is now directly observable: drive to a target, then
  drive back to it from the other direction, and the difference in landed
  angle is the backlash.
- No off-target tests for the state machine, and no simulator any more, so
  there is currently no way to exercise it without hardware.
- **The front panel and the IMU are reserved, not implemented.** `Config.h`
  sections 11 and 12 claim pins for three buttons, a buzzer and an MPU6050,
  and `USE_BUTTONS` / `USE_BUZZER` / `USE_IMU` all default to `0` because no
  code reads them. Two things to know before writing that code: `<Wire.h>` is
  guarded on `USE_LCD` alone and needs to become `USE_LCD || USE_IMU`, and the
  MPU6050 **cannot measure yaw** — it is a six-axis part with no magnetometer,
  so it cannot replace the compass sighting behind `a` and `b`. What it can
  usefully do instead is in [WIRING.md](WIRING.md).
- **`REF_ANGLE_DEG` is a placeholder.** It is where the reference switch
  physically sits, and `H` adopts it as truth, so the whole coordinate system
  inherits whatever is in there. Measure it against the boom once.
- **`REF_DRIFT_WARN_DEG` is a placeholder** and needs the switch's own
  repeatability measured first. Set below that and every ordinary crossing
  cries wolf; set well above it and real drift goes unreported.
- **The step-track search does not exist yet.** `actuator_v2/` is the inner
  loop only. `host/metric.py` now supplies the number it would climb — and is
  the first file in `host/` that has actually been executed, against synthetic
  statsd traffic — but nothing yet walks the target angle toward a peak. That
  is the next piece of real work.
- **The metric's own noise floor has never been measured on real signal**, and
  it sets both the dwell and the smallest usable step. `metric.py --noise`
  does it; it has not been pointed at a live receiver.
- **The AS5600 magnet has never been mounted**, so the whole Rev B measurement
  chain is unverified end to end.
- **Flash on Rev B is unmeasured** — there is no AVR toolchain on the build
  machine, so `actuator_v2/` has only been syntax-checked against stubs. Rev A
  sat at 63% without the LCD. v2 adds `Wire` and float PID but drops homing,
  the EEPROM position machinery and the angle fit, so it may well come out
  smaller. It prints with integer arithmetic and never `%f`, which is where
  the 1.5 kB of AVR float-printf support would otherwise have gone.
