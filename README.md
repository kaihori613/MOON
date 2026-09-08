# MOON — Actuator Calibration Bench

Arduino code for calibrating an HTS TVRO linear actuator against its internal
reed-switch position sensor.

**This repo is a bench, not a system.** Nothing downstream of the reed count is
worth building until the count is trustworthy at 24 V through a real bridge
under real load. `reed_switch_test/` is the main sketch and the only one that
should be running right now.

## The one job

**Millimetres per reed count.** It is a property of the mechanism — motor
revolutions through a fixed gear ratio and screw pitch — so it holds at any
duty, any load, any direction. That invariance is what makes it measurable, and
what makes it the thing everything else is denominated in.

Until it is measured and *proven* measured, the pointing resolution of this
system is unknown and every constant downstream is a guess.

## Hardware under test

| | |
|---|---|
| Actuator | HTS TVRO linear, internal reed sensor, internal cam limits |
| Bridge | L298N on D9 (ENA) / D6 (IN1) / D5 (IN2) — see the warning below |
| Sensor | Reed to D2 and GND, `INPUT_PULLUP`, falling-edge interrupt |
| Supply | 24 V DC bench supply |
| Mounted | Yes — the dish is on, so both cams sweep the full arc |

Set `MOTOR_DRIVER` in `actuator_v1/Config.h` to match what is actually wired.
It defaults to `DRV_L298N`. The alternative is `DRV_HW039` on D9/D10/D8.

## `reed_switch_test/` — the main sketch

```
?        this help
e [ms]   run EXTEND  for ms and count pulses (10000)
r [ms]   run RETRACT for ms and count pulses
+[n]     extend  n counts and stop
-[n]     retract n counts and stop
x        stop the run now          <-- bare Enter does NOT stop here
w <n>    PWM duty, 60-255
m <mm>   distance the rod moved on the last run
s        sensor report
z        zero all statistics
d <us>   set the debounce window (no reflash needed)
t <ms>   stall timeout, 100-10000 (default 700)
n        noise floor test -- actuator must be STOPPED
v        toggle per-pulse printing
p        read the pin level right now
```

Note `x`, not Enter. `actuator_v1` stops on a bare Enter; this sketch ignores
empty lines. Different muscle memory, and worth knowing before you are leaning
on the keyboard with a dish swinging.

### Order of operations

1. **`n`** — noise floor, carriage still, **driver powered**. Every edge here is
   false by definition. If it reports pickup, stop: a count that gains phantom
   pulses makes every measurement below meaningless.
2. **`v`** off, then a short **`e 2000`** to watch pulses arrive.
3. **`s`** — the sensor verdict. Two cleanly separated populations, or not.
4. Only then the calibration: **`+400`**, measure the rod, **`m <mm>`**.

### Why `+n` rather than a timed run

A count-targeted run makes the count an *input*. You get exactly `n + overshoot`
pulses, the sketch reports the overshoot, and the only remaining error is your
distance measurement. A timed run leaves ±1 count of quantization at each end.

Cam-to-cam timed runs are still the right instrument for *cam repeatability* —
a different question, and one this bench has never answered.

### What each run produces

| measurement | feeds |
|---|---|
| mm per pulse | `linkage.mm_per_count`, and the resolution floor |
| coast, in ms | `COAST_SETTLE_MS` |
| coast, in counts | whether `DEADBAND_COUNTS = 1` is reachable at that duty |
| overshoot vs duty | `SPEED_TRIM` — drop the duty until the overshoot fits |
| slowest gap under load | `STALL_TIMEOUT_MS`, which wants ~3× it |
| `+50` then `-50` | backlash, with the direction speed difference removed |

## Known hardware issues

These came out of the bench session of 20 Aug 2026 and are unresolved.

**The supply current limit was 2 A, and that is too low.** A dish actuator draws
several amps running and more at breakaway. At 2 A the supply drops into
constant-current, the voltage collapses, the motor stalls, and the sketch cuts
the run. Watch the CC indicator during a run — if it is lit, nothing measured is
valid. Set the limit to 8–10 A.

**The L298N is marginal at 24 V.** It is a BJT bridge dropping roughly 3–5 V
across the output stage, which at a few amps is 6–12 W in a Multiwatt15 package.
The stock module heatsink handles maybe 2–3 W. Thermal shutdown in seconds to
tens of seconds is the expected behaviour, and it presents exactly as a run
dying early. Diagnostic: successive runs die progressively sooner, and the chip
is too hot to touch.

**Remove the module's 5 V-enable jumper at 24 V.** The onboard 78M05 would be
dropping 19 V. If it is also feeding the Arduino, that alone explains a run
ending early with no reset banner to show for it.

**The debounce and the stall detector are coupled.** The stall timeout watches
*accepted* pulses, so a debounce set near or above the real pulse gap starves the
stream and trips a stall while the motor runs on perfectly. The sketch now
reports which of the two happened — see below.

## Reading a run that ended early

Runs end for five reasons and the report names which:

| report says | meaning |
|---|---|
| `window elapsed` | normal |
| `count reached` | normal, `+n` runs |
| `stopped by hand` | you typed `x` |
| `never broke away -- no pulse at all` | nothing within `RUN_GRACE_MS` (900 ms) |
| `accepted pulses stopped` | the stall timeout fired — read the verdict |

For that last case the sketch now samples how long since the last **raw** edge,
timestamped ahead of the debounce, and prints one of two verdicts:

- **THE DEBOUNCE STARVED IT** — edges were still arriving and this filter
  rejected them. The rod kept moving. Lower the debounce; do not calibrate
  against that run.
- **THE EDGES STOPPED TOO** — the carriage really stopped. Cam, jam, thermal
  shutdown, or current limit; those four are indistinguishable from here.

That is the reading that separates a bad debounce from a bad supply, and there
is no other way to see it.

`t <ms>` retunes the stall timeout live. It is a diagnostic, not a fix —
raising it past the real pulse gap only delays the same verdict, and delays
cutting current into a genuine jam.

## Parked until the count is trusted

Neither of these should be touched before the calibration lands.

**`actuator_v1/`** — the closed-loop controller. Compiles clean, 63% flash on a
328P, **never run**. Six states, zero transitions ever executed. Its design
rationale is worth keeping and lives in [docs/design-notes.md](docs/design-notes.md)
and in the sketch's own comments.

**`host/`** — Python yaw pointing over USB. The **pointing math now runs and
passes**, 27/27, on 8 Sep 2026 — the first time any of it had been executed.
The serial half is still untouched: there is no `pyserial` installed, and no
Arduino has ever answered this program.

There is no Python on `PATH`, but there is one on the machine, bundled with
ANSYS. No install, no admin rights:

```
"/c/Program Files/ANSYS Inc/ANSYS Student/v261/optiSLang/lib/python3.10/python.exe"
```

Python 3.10.19. Recorded because it took a search to find, and the supposed
absence of an interpreter had been blocking the cheapest tests in the repo.

> **The count-origin defect is fixed, and the fix is tested.** It was worse
> than previously recorded: not only did `host/README.md` claim homing gives
> `counts_a = 0`, but `TriangleLinkage` hardcoded the same assumption, treating
> count zero as the retract stop. Under `ORIGIN_AT_MIDPOINT 1` (the shipped
> default) homing lands at `-(travel/2)`, so the model was asking about a point
> half a stroke away. It now takes `counts_at_retract` and measures from there,
> mirroring `degreesNow()` in the sketch.
>
> What the placeholder cost, measured rather than argued: a plausible two-point
> calibration sighted at 210° and 222°, entered with the config template's old
> `counts_a: 0` instead of the homed `-348`, reads **20.88° off at the homed
> position**. The dish beamwidth is a couple of degrees. That is not a tuning
> error, it is a different piece of sky.

## `archive/`

`l298n_test/` and `actuator_test/` — bring-up sketches. The motor turned. Both
are superseded by `actuator_v1/` and kept only as history.

Two earlier sketches, `actuator_system/` and `cam_switch_test/`, were deleted
outright and now have to be recovered from git history. That has been regretted
twice. Nothing else gets deleted here — it gets archived.

## Testing strategy

A full coverage plan — six tiers from build matrix to bench gates, with the
untested seams named — is written up separately. The short version: the bench is
the *most* expensive place to find a bug in this project, not the least, because
it charges in hardware. Push coverage down.
