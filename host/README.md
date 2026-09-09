# Host-side yaw pointing

Runs on the PC, drives the Arduino over USB. Two jobs:

1. **Automatic cycle** — home the actuator, work out where GOES-18 is from
   your coordinates, drive the yaw axis there.
2. **Manual tuning** — hand the keyboard over so you can peak on signal with
   arrow keys, and store the correction.

The satellite math lives up here rather than in the sketch because it needs
floating-point trig and a config file, neither of which an Uno should be asked
to carry. The sketch stays what it already is: a closed-loop position
controller that does not know or care what a satellite is.

## Why the tuning is a *trim* and not a position

Everything the automatic cycle does is open loop with respect to the **signal**.
The controller knows exactly where the carriage is; nothing in the system knows
where the beam actually is. Survey error, mount flex, a linkage measurement off
by a few millimetres, and backlash all stack up.

So the manual step exists, and what it produces is an offset:

```
commanded = computed(satellite) + trim
```

Store a *position* and it is invalidated by the next re-home or recompute.
Store a *trim* and it survives all of them.

This matters more than it sounds, because **GOES-18 is geostationary at 137.0°W**
— from a fixed site the look angles never change. There is nothing to track.
Peak it once, save the trim, and that number is your permanent site correction.
It is the most valuable output of this program.

## Setup

```bash
pip install -r requirements.txt
```

Python 3.8+. The only dependency is pyserial.

First run writes a blank `config.json` and stops:

```bash
python moon_yaw.py --dry-run
```

Fill in `site.latitude_deg` / `site.longitude_deg` — **East-positive**, so
122.3° W is `-122.3`. There are no default coordinates on purpose: a dish
pointed confidently at the wrong sky because a placeholder went unnoticed is
worse than a program that refuses to start.

That is enough for `--dry-run` to print the look angles, which is worth checking
against a map before anything can move. The linkage below can stay blank until
the actuator is drivable — the azimuth depends only on where you are standing.

## Calibrating the linkage

Three ways now, and the first one supersedes the other two if you have an
accelerometer on the dish.

### `gravity` — sweep once, no compass, no tape measure

The yaw axis is tilted, so rotating the dish tilts it, and an accelerometer
measures that directly. Across a sweep the gravity vector traces a circle in
the sensor's frame; the circle's plane is perpendicular to the rotation axis,
and the angle around it is yaw.

```
c                     # home and learn the travel
w                     # step, stop, settle, read -- capture the console
```

```bash
python calibrate_axis.py --from-log sweep.txt --save
```

This gives the counts→degrees curve **as measured**, over the whole stroke,
which is what the `linear` and `triangle` models below are both trying to
approximate. It also answers, with a number, the question this file has so far
been answering on faith: `linear worst error` is exactly how far the linkage
departs from a straight line.

Read two things before believing the result:

- **`conditioning`** — must be above 25. Below that the sweep was too short and
  the angle scale is wrong even though the residuals look clean. The fix is to
  sweep more travel, not to average harder.
- **`error amplification`** — tilt error into yaw error, `1/sin(gamma)`. Around
  1.3 on a polar mount. If it is above 3 the axis is closer to vertical than
  this method likes.

No hardware to hand? `make_test_sweep.py` emits a synthetic sweep in exactly
the sketch's output format:

```bash
python make_test_sweep.py > sweep.txt && python calibrate_axis.py --from-log sweep.txt
python make_test_sweep.py --deg-per-count 0.012 > short.txt   # the short-arc failure
```

### The older two

The program needs to convert a heading into reed counts. Two models, picked by
`linkage.model`:

### `linear` — start here

Two measured points, no tape measure:

1. Home the actuator (`h` in the Arduino console). That is `counts_a = 0`.
2. Sight along the boom, read the heading → `angle_a_deg`.
3. Drive well along the stroke, e.g. `g 400` → `counts_b = 400`.
4. Sight it again → `angle_b_deg`.

Accurate to a fraction of a degree if your two points bracket the arc you
actually use. It drifts as the arc widens, because the real relationship is a
triangle and this is a straight line through it.

### `triangle` — physically correct

Needs real measurements, and is worth it if yaw sweeps a wide arc:

| key | what it is |
|---|---|
| `pivot_to_base_mm` | pivot centre to the actuator's fixed mount |
| `pivot_to_carriage_mm` | pivot centre to the actuator's moving mount |
| `retracted_length_mm` | mount-to-mount distance with the actuator homed |
| `mm_per_count` | from the sensor bench test: stroke length ÷ total counts |
| `angle_at_retract_deg` | heading measured with the actuator homed |
| `direction` | `+1` if extending increases heading, `-1` if it decreases |

Note `mm_per_count` comes from `actuator_sensor_bench_test` — which has not
been run yet, so the triangle model cannot be calibrated until it has.

### Compass headings

`look_angles()` returns **true** azimuth. If you sighted the headings above
with a compass, set `headings.magnetic: true` and your local
`headings.declination_deg` (positive east) so the two frames agree.

## Running

```bash
python moon_yaw.py --list-ports
python moon_yaw.py --port COM3
```

Useful flags:

| flag | effect |
|---|---|
| `--dry-run` | compute and print the pointing, touch no hardware |
| `--home` | re-home even if the controller thinks it already knows where it is |
| `--no-auto` | skip the automatic cycle, go straight to manual tuning |
| `-v` | echo every serial line, for when the link misbehaves |

`--dry-run` also prints **degrees per reed count** at the target. That number is
the hard floor on yaw accuracy — the loop cannot hold tighter than one count no
matter how it is tuned, and measured backlash sits on top of it. If it is
coarser than the pointing accuracy yaw needs, the fix is mechanical (longer
moment arm from the actuator to the yaw pivot), not a change to this code.

## Manual tuning keys

```
left / right   nudge by the step size   (also - and +)
up / down      double / halve the step
space          STOP
s              save this trim to config.json
0              discard trim, return to the computed target
a              re-run the automatic cycle
k              clear a fault
q  or  Esc     quit
```

Nothing is written to disk until you press `s`.

Held keys are coalesced: a new move is only issued once the previous one has
finished, so leaning on an arrow key walks the actuator along instead of
queueing up a pile of moves and overshooting.

## Testing it with no actuator

There is still no simulator for the *serial* side — that belonged to
`actuator_system/`, which was deleted in favour of `actuator_v1/`, and is
recoverable from git history if it becomes painful.

Everything else runs with no hardware:

```bash
python test_geometry.py       # the pointing math          -- 21/21
python test_axis_fit.py       # the gravity axis fit       -- 24/24
python make_test_sweep.py > sweep.txt && python calibrate_axis.py --from-log sweep.txt
```

That last pair covers the whole gravity calibration path end to end, from
console text through parsing to the fitted curve. The live `--port` path joins
it immediately after the parse, so a failure there is a serial problem rather
than a maths one — which is the same reasoning as `link.py` only ever sending
commands you could have typed by hand.

## Files

| file | |
|---|---|
| `moon_yaw.py` | entry point: automatic cycle, then manual tuning |
| `geometry.py` | look angles to a geostationary satellite; linkage models |
| `link.py` | serial transport, speaks the sketch's existing console protocol |
| `config.py` | site, linkage and trim persistence |
| `keys.py` | cross-platform single-keypress reader |
| `test_geometry.py` | self-checks for the pointing math |
| `axis_fit.py` | rotation axis and counts↔degrees, fitted from a gravity sweep |
| `calibrate_axis.py` | runs that fit over a sweep log or a live port |
| `make_test_sweep.py` | synthetic sweeps, so the fit can be exercised dry |
| `test_axis_fit.py` | self-checks for the axis fit |

`link.py` deliberately sends only commands you could have typed by hand into
the Serial Monitor. When something misbehaves, unplug the script, open the
monitor, and drive the same commands yourself to find out whether the problem
is up here or down there.
