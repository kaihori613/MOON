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

## The calibration run

`peak.py` sweeps the travel, stops to measure at each step, fits the readings
and parks on the best angle. It takes two callables — `move_to(angle)` and
`read_metric()` — so it is testable against a simulated dish and does not care
whether the number came from SatDump, goesrecv or `iq_snr`.

### Fit every point, not the best few

This is the one counter-intuitive part, and it is a property of the dish rather
than of the code. Pointing loss goes as `12·(θ/θ₃dB)²` dB, so **on a 12° beam,
being 1° off costs 0.083 dB** — far below the noise on any realistic SNR
estimate. The samples nearest the peak are therefore all within noise of each
other, and a vertex fitted through them is fitting noise. The information about
where the peak *is* lives on the flanks, where the curve has gradient.

Measured against a simulated 12° beam at σ = 0.4 dB, over 120 trials each:

| points fitted | median error | 90th | worst |
|---|---|---|---|
| 3 | 0.603° | 1.702° | 2.769° |
| 5 | 0.506° | 1.796° | **2.744°** |
| 7 | 0.225° | 0.567° | 1.079° |
| 9 | 0.134° | 0.437° | 1.138° |
| all | 0.121° | 0.306° | 0.598° |

Note that five points is *worse than three* at the tail. Accuracy improves
monotonically with the number fitted and saturates around nine, so the default
is to fit everything that produced a reading. **Do not lower it without
measuring.** The result holds while the sweep stays inside the main lobe, which
it does over ±9° on a beam this wide; a sweep reaching sidelobes would need the
fit narrowed, because a parabola stops describing the pattern out there.

### Checking that the aim is still good

`check_pointing()` is the reference switch, done in software. Calibration
records what the metric read at the peak; the check compares against it and
reports one of `ok`, `improved`, `degraded`, `failed` or `no-signal`. It never
moves anything — the caller decides what to do.

The threshold is `max(fixed, 3σ)`. The fixed part is 2 dB for `degraded` and
4 dB for `failed`. σ combines how much the reference average and the current
average each wander: the standard error, `stdev/√n`, not the scatter of the
single readings inside them. It exists because a system that reacts to its own
measurement noise will re-peak a perfectly good dish on a windy afternoon and
store whatever it happens to find.

An earlier version used the raw scatter, which overstated the noise by `√n`
and pushed the threshold to about 4 dB. Simulated with 1 dB of scatter per
reading, a 10-reading check against a 30-reading reference, and 2,000 trials
per row:

| drop | flagged, raw scatter | flagged, standard error |
|---|---|---|
| 0 dB | 0% | 0% |
| 2.5 dB | 0.4% | 92% |
| 3 dB (≈6° off on a 12° beam) | 3% | 99.8% |
| 4 dB | 40% | 100% |

Slow changes between sessions, like weather or gain drift, never appear inside
one dwell, so σ can't see them. The fixed floor is what absorbs those.

A reference saved without its `n` falls back to the raw scatter, which errs
toward missing a drop rather than raising a false one.

Given a beamwidth it also reports how much pointing error would explain the
drop, by inverting the loss law. A 3 dB drop on a 12° beam comes back as 6.0° —
exactly the half-power point, which is the sanity check on the arithmetic. The
number tells you whether re-peaking could plausibly recover this or whether it
is far too large for any error the mount can even reach.

`improved` is reported rather than swallowed: it usually means the reference
was taken in worse conditions than these and is not the standard to hold to.

### Failure is non-destructive

No usable reading anywhere, or a best reading at the edge of the swept range,
both return the dish to where it started and report why. A peak at the edge
means the real one is probably outside the range, so the answer is "widen the
sweep", not that angle.

## Where the pointing metric comes from

`metric.py` is the signal side of the loop. It does not point anything; it
produces the one number a peak search climbs, and tells you whether that
number is stable enough to climb.

**Nothing here does PID on signal strength.** SNR against pointing angle is a
peak, not a ramp: the same reading occurs on both sides of it, so a controller
fed SNR has no sign to act on. Peaking is a *search* — step, dwell, compare,
reverse and halve when it gets worse. The sketch runs a PID on **angle**,
where the error does have a sign, and this side hands it target angles.

### Without a decoder at all

`iq_snr.py` estimates SNR straight from IQ samples. It exists because a
decoder metric has one limit that matters for pointing: **it only exists once
the decoder has lock, and lock is exactly what a mispointed dish does not
have.** A decoder metric can refine an aim you already roughly have; it cannot
help you find the bird.

**BER is not available this way and cannot be.** Bit error rate is a decoder
output — measuring it means knowing what the bits should have been, which
means demodulating, synchronising and running the FEC, at which point you have
written a decoder and should have used SatDump. From raw IQ you get SNR or
C/N0. Anything advertised as "BER from the spectrum" is estimating SNR and
converting through an assumed modulation and code rate.

Two estimators, both checked against synthetic signals of known SNR and
accurate to better than 0.1 dB across 0–25 dB:

- `snr_m2m4(iq)` — second and fourth moments. **Pure stdlib**, no FFT, no need
  to know where in the band the signal sits. Assumes a constant-modulus signal
  (PSK) in Gaussian noise.
- `snr_psd(iq, fs, signal_bw)` — in-band against out-of-band power density.
  Needs numpy. Modulation-agnostic, and it rejects interference outside the
  signal band instead of counting it as signal.

Prefer `snr_psd` where numpy is available; the two disagreeing is itself
informative.

Note that `snr_m2m4` **refuses** rather than returning a number when the
moments are within three sigma of pure noise. That matters more than it
sounds: on noise the estimator would otherwise return something like −10 dB,
and a search will happily decide −10 beats −11 and climb noise uphill. Its
usable floor scales as n^-¼ — roughly −2 dB at 10k samples, −9 dB at 1M.

**The SDR is single-client.** While SatDump holds the device nothing here can
read samples, so the two paths are alternatives in time, not in parallel —
which fits a scheduled recording exactly:

```
before SatDump starts  ->  iq_snr, to acquire and coarse-peak
while SatDump runs     ->  satdump.py, to refine on the decoder metric
```

### If you decode with SatDump

`satdump.py` is the same interface over SatDump's HTTP status endpoint:

```bash
satdump live <pipeline> <output> --source ... --http_server 0.0.0.0:8080
python3 satdump.py --probe http://127.0.0.1:8080/
```

`--probe` prints every numeric field the endpoint serves, because what is in
there varies by SatDump version and pipeline. Pick the one that tracks link
quality. **If it is an SNR in dB then higher is better**, so construct
`Metric(..., lower_is_better=False)` — the opposite of the Viterbi case below,
and the easiest sign error in the loop.

Non-numeric fields are dropped rather than coerced. A status string like
`SYNCED` is useful to a human, but mapping it to `1.0` would put a constant
into the loop that looks like a measurement.

### Triggering on a recording, and why that is two signals

Starting the search when SatDump starts recording is the right idea on the
wrong edge. A recording starting means the **scheduler** fired. It does not
mean the demodulator has locked, and until it has, the metric is either
absent or measuring noise. So:

```
recording started   ->  ARM the search
metric is live      ->  RUN it
```

`RecordingWatcher` gives you the first, by watching the output directory for
a new entry. Filesystem rather than an API on purpose: it needs no SatDump
feature, no version agreement and no cooperation from the scheduler, so it
keeps working across upgrades.

```bash
python3 satdump.py --watch-dir /path/to/satdump/output
```

`wait_for_lock()` gives you the second. It returns the first real reading, or
raises — because a search that runs without signal does not fail loudly, it
walks the dish somewhere wrong and reports success.

### Getting the number out of goesrecv

goesrecv emits statsd over UDP, which is plain text on a datagram socket — no
dependency, no polling, and it pushes rather than being asked:

```
[monitor]
statsd_address = "udp://127.0.0.1:8125"
```

The metric names vary by goesrecv version, so they are **not** hardcoded here.
Find out what your build actually emits:

```bash
python3 metric.py --discover
```

Then measure how noisy the one you picked is, with the dish **not moving**:

```bash
python3 metric.py --noise goesrecv.decoder.viterbi_errors --seconds 180
```

### Why the noise floor comes first

This is the same test the reed switch got, for the same reason. There the
question was how many edges arrive when nothing is turning. Here it is how
much the metric wanders when nothing is pointing differently. Both answer the
only question worth asking before building a loop on a sensor: is the thing
you are about to react to actually a signal?

The output is a table of dwell time against the standard deviation of the
averaged reading. The rule it exists to serve:

> one pointing step must change the metric by more than about **3×** the
> standard deviation of the averaged reading at that dwell

If it does not, the search is climbing noise, and it will walk away from a
perfectly good peak with complete confidence. Longer dwell buys a quieter
reading at roughly the square root of the time — and buys it out of the
search's wall-clock budget, since every step pays it.

### Which metric

1. **Viterbi corrected errors** — steepest against pointing error, because it
   sits after the demodulator where a fraction of a dB moves it a lot. Note
   it goes **down** as the signal improves; `Metric(..., lower_is_better=True)`
   exists because that sign is the easiest thing in the loop to get backwards.
2. **Es/N0 or SNR** — smoother, slower, but it does not floor.
3. **Raw RSSI or band power** — avoid. It measures noise as much as signal and
   barely moves with pointing on a beam this wide. It is the obvious thing to
   reach for and the wrong one.

### If the metric floors at zero

Corrected errors sitting at exactly zero across a range is **not a failed
search**. It means you are comfortably inside the beam and pointing better
buys nothing measurable. Either switch to a metric that does not saturate, or
accept that anywhere in the flat region is a valid answer — which, for a
geostationary bird you only peak once, it is.

## Peaking on SDR SNR with `actuator_v2`

`snr_peak.py` closes the outer loop. It drives `actuator_v2` through
`peak.sweep()` with `g <deg>` targets, and reads SNR from the RTL-SDR with
`iq_metric.py` at each point once the PID has landed. `moon_yaw.py` cannot do
this: it speaks `actuator_v1`, whose `g` takes reed counts.

```bash
python snr_peak.py --simulate --sweep -8 8 1     # whole loop, fake dish, no hardware
python snr_peak.py --simulate --sweep -8 8 1 --plot   # ...and draw it to sweep.png
python iq_metric.py --watch                       # the SDR on its own, first
python snr_peak.py --port COM3 --sweep -8 8 1    # peak, park, save pointing.json
python snr_peak.py --port COM3 --check           # is SNR still what the sweep found?
```

Only run it on hardware once both halves work alone: the PID holding a `g`
target typed into the serial console, and `--watch` giving believable numbers.

### What the simulation randomizes, and Monte Carlo

The fake dish has its own random number generator, seeded with `--sim-seed`;
without a seed, each run differs. Two things in it are random:

- **Every SNR reading** is the beam's true value plus Gaussian noise of 1 dB.
  At 10 readings a second, a 3 s dwell averages 30 of them.
- **Every move** lands at the target plus a uniform offset of up to ±0.25°,
  standing in for the PID's deadband.

The noise model is an assumption until `iq_metric.py --noise` measures the
real one. It has no slow drift (gain, weather), which the real SDR will have.

One `--simulate` run is one random example. Two options turn it into a study:

```bash
python snr_peak.py --simulate --sweep -8 8 1 --sim-beam airy --sim-peak 5.5 --trials 200
python snr_peak.py --simulate --sweep -8 8 1 --sim-beam airy --sim-peak 5.5 --trials 200 --no-refine --plot mc.png
```

- **`--sim-beam airy`** replaces the parabolic beam with a real circular
  dish's pattern. The parabola is exactly what the fit assumes, so it can only
  ever show noise. The Airy beam matches it to within 0.03 dB out to half the
  beamwidth, then falls faster: 7.4 dB down at 9° against the parabola's 6.75.
  That difference is what makes a one-flank fit lean.
- **`--trials N`** runs N sweeps quietly and prints the median, 90th
  percentile, worst and **signed mean** of how far the peak landed from the
  truth. Noise scatters the answer evenly around zero. A model error shifts
  the whole scatter to one side, and only the signed mean shows that. With
  `--plot`, it draws a histogram instead of a sweep.

The two commands above, seeds 1000..1199:

| | median | worst | signed mean |
|---|---|---|---|
| one pass | 0.303° | 0.672° | **−0.290°** |
| two passes | 0.037° | 0.190° | +0.009° |

- **Before anything moves**, it refuses a sketch that isn't `actuator_v2`, a
  controller that is already faulted, a range past the ±13° soft limit, and an
  SDR that won't deliver samples.
- **During the sweep**, a move that lands more than 0.6° from its target aborts
  it. That means the move was refused, or a panel button took over, and a
  reading there would be credited to the wrong angle. Inside that tolerance,
  each reading is credited to where the move actually landed, not where it
  was sent.
- **It sweeps twice**, unless you pass `--no-refine`. The second pass is centred
  on the first pass's peak, half a beamwidth (6°) either side, at half the
  step. The peak is then fitted to both passes together. This roughly 2.5×
  the sweep time.
  - **Why it's wide, not narrow:** a narrow fine pass (±2° at 0.25°) measured
    about three times *worse* than no second pass at all. Every new reading
    sits on the flat top, where the beam changes less than the noise does.
  - **What it fixes:** a peak near the end of the `--sweep` range leaves the
    first pass with only one flank. On a real beam, which is not quite a
    parabola, the fit then leans toward that flank. In simulation on an Airy
    beam with the peak at +4° to +6.5° on a −8..8 sweep, the fitted peak was
    0.29° off (median), and averaging longer did not help. With the second
    pass it was 0.04°.
  - **It may go past LO..HI**, never past the ±13° soft limit. Kept inside the
    typed range, an edge peak gets one flank again, and the error went back to
    0.22°.
  - **What it doesn't fix:** where the dish finally parks is limited by the
    PID's ±0.25° deadband, about 0.12° median, which is now bigger than the
    fit's own error. On a 12° beam that costs under 0.01 dB. Even the
    one-pass worst case, 0.95°, costs 0.07 dB. The second pass matters more
    for a narrower beam: a bigger dish or a higher frequency.
- **Any failure** returns the dish to where it was before the sweep, not to the
  first swept angle.
- **`pointing.json`** holds the peak angle, the reference SNR and the exact SDR
  settings. `--check` reuses those settings, because an SNR from a different
  estimator or gain can't be compared with the reference. The file is
  gitignored because it belongs to this site, like `config.json`.
- **`--check` never re-peaks on its own.** A drop can be rain or a wet LNA, and
  re-peaking on that would store a worse aim than the saved one.

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

The program needs to convert a heading into reed counts. Two models, picked by
`linkage.model`:

### `linear` — start here

Two measured points, no tape measure. **Both counts are positions the sketch
reports back, never positions you asked for:**

1. Home the actuator (`h` in the Arduino console), then read the position it
   prints → `counts_a`. With `ORIGIN_AT_MIDPOINT` — the shipped default —
   that is `-(travel/2)`, not zero. A plain `h` before any `c` calibration
   does put zero at the retract stop, so this number tells you which of the
   two you are in.
2. Sight along the boom, read the heading → `angle_a_deg`.
3. Drive well along the stroke, e.g. `g 400`, then read the position it
   settled at → `counts_b`. Do not assume 400: `g` clamps to
   `SOFT_LIMIT_MARGIN` off each stop, and a move may finish a count or two
   short of its target.
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
| `retracted_length_mm` | **pin-to-pin** distance with the actuator homed |
| `mm_per_count` | from the sensor bench test: stroke length ÷ total counts |
| `angle_at_retract_deg` | heading measured with the actuator homed |
| `direction` | `+1` if extending increases heading, `-1` if it decreases |
| `counts_at_retract` | position the sketch reports once homed (default `0`) |

**Both length keys mean pin-to-pin, not the actuator's own axial length.** On
this machine the bottom pin joint is offset 48 mm perpendicular to the rod,
which makes those two distances differ by 2.249 mm at home and makes them
extend at slightly different rates — the pins gain 76.707 mm over a 77 mm
stroke. Measure the joints, not the tube. The full CAD derivation behind
those numbers was a dated bench-session doc, since removed as stale; see
git history if it's needed again.

The last one is the same trap as `counts_a` above. This model is anchored to
the **retract stop**, not to count zero, and those are the same place only
with `ORIGIN_AT_MIDPOINT` off. Leave it at `0` under the shipped default and
the model is asking about a point half a stroke away — 174 mm out, on a
different part of the arc. It mirrors `degreesNow()` in the sketch, which
measures from `retractStopPos()` for the same reason.

Note `mm_per_count` comes from `reed_switch_test` (now `archive/reed_switch_test/`,
superseded by the AS5600 approach in `actuator_v2`), and as of the 4 Sep 2026
bench session **it did not exist as a trustworthy number**: reed counts did
not map consistently to distance, disagreeing by 7.5× across two steps of the
same run at the same physical speed. The `triangle` model is unusable until
that is fixed on a sketch that still uses reed counts for position; `actuator_v2`
does not, so this whole model is only relevant if `actuator_v1` is ever revived.

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

`moon_yaw.py`'s serial side has no simulator any more — it belonged to
`actuator_system/`, which was deleted in favour of `actuator_v1/`, and is
recoverable from git history if that becomes painful. `snr_peak.py
--simulate` covers the `actuator_v2` loop instead, with a fake dish standing in
for both the actuator and the SDR.

Everything else runs under pytest with no hardware, from `host/` with the
project's `.venv` (`pip install -r requirements.txt numpy pytest`):

```bash
pytest
```

165 passed as of 27 Oct 2026. `test_geometry.py` alone still needs nothing
beyond the standard library: `python test_geometry.py`.

Both SNR estimators refuse pure noise rather than return a number from it.
`snr_psd` did not until 27 Oct 2026: it reported about half of all noise-only
captures as SNRs near -25 dB, which a sweep would happily climb. It now splits
the capture into 16 groups in time and needs the in-band excess three standard
errors above zero. Over 1000 noise-only captures that let 2 through, where the
old code let 504 through. Real signals read the same down to -15 dB. Fainter
than about -17 dB in a 2^18-sample capture, the estimator now refuses them as
well, because by then the signal is the size of the noise's own scatter.

## Files

| file | |
|---|---|
| `moon_yaw.py` | entry point for `actuator_v1`: automatic cycle, then manual tuning |
| `snr_peak.py` | entry point for `actuator_v2`: peak on SDR SNR, then check against it |
| `iq_metric.py` | live SNR from the RTL-SDR, in the shape `peak.sweep()` reads |
| `sweep_plot.py` | `snr_peak.py --plot`: the sweep as a chart (needs matplotlib) |
| `geometry.py` | look angles to a geostationary satellite; linkage models |
| `link.py` | serial transport, speaks the sketch's existing console protocol |
| `config.py` | site, linkage and trim persistence |
| `keys.py` | cross-platform single-keypress reader |
| `test_geometry.py` | self-checks for the math |
| `test_snr_peak.py` | the outer loop's safety rules, against a fake link and dish |
| `test_check_pointing.py` | when an SNR drop is real enough to call for a re-peak |
| `test_iq_metric.py` | both SNR estimators and `SdrSnrMetric` on synthetic IQ |

`link.py` deliberately sends only commands you could have typed by hand into
the Serial Monitor. When something misbehaves, unplug the script, open the
monitor, and drive the same commands yourself to find out whether the problem
is up here or down there.
