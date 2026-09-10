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

There is no simulator any more — it belonged to `actuator_system/`, which was
deleted in favour of `actuator_v1/`. So the serial side currently cannot be
exercised without a real Arduino and a real actuator. (The old simulator is
recoverable from git history if that becomes painful.)

The pointing math has its own test, which needs no hardware at all:

```bash
python test_geometry.py
```

## Files

| file | |
|---|---|
| `moon_yaw.py` | entry point: automatic cycle, then manual tuning |
| `geometry.py` | look angles to a geostationary satellite; linkage models |
| `link.py` | serial transport, speaks the sketch's existing console protocol |
| `config.py` | site, linkage and trim persistence |
| `keys.py` | cross-platform single-keypress reader |
| `test_geometry.py` | self-checks for the math |

`link.py` deliberately sends only commands you could have typed by hand into
the Serial Monitor. When something misbehaves, unplug the script, open the
monitor, and drive the same commands yourself to find out whether the problem
is up here or down there.
