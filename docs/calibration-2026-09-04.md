# Calibration bench session — 4 Sep 2026

Reed-switch bench (`reed_switch_test`), L298N **channel B**, duty 200,
debounce 1500 µs, stall timeout 800 ms.

## Headline

**Reed counts do not map consistently to distance.** Until that is fixed,
nothing built on counting pulses can be trusted — not stroke length, not
mm/count, not closed-loop positioning, not the degrees fit.

## Run 1 — home to the extend hard stop

Count-targeted moves (`+500`), rod measured at each stop. "Exposed" is rod
length from the housing face; home sits 251 mm off the retract limit.

| stop | counts (step) | counts (cum.) | exposed | Δ from home | counts/mm |
|------|--------------:|--------------:|--------:|------------:|----------:|
| home | —             | 0             | 251 mm  | 0           | —         |
| 1    | 501           | 501           | 272 mm  | 21 mm       | 23.9      |
| 2    | 196           | 697           | 328 mm  | 77 mm       | 3.5       |

Stop 2 ended on the stall watchdog (`accepted pulses stopped`) at the extend
hard stop — the watchdog did its job correctly.

**Total: 697 counts for 77 mm of travel.** Full stroke, retract limit to
extend limit, is therefore about **328 mm**.

## Why the counts are not believable

The two steps disagree by a factor of nearly seven on counts per mm. That
alone could be a nonlinearity. The timing rules that out:

| stop | travel | duration | physical speed | count rate |
|------|-------:|---------:|---------------:|-----------:|
| 1    | 21 mm  | 9.89 s   | 2.1 mm/s       | 50.7 /s    |
| 2    | 56 mm  | 29.2 s   | 1.9 mm/s       | 6.7 /s     |

The rod moved at essentially the same speed in both steps. The sensor
reported rates differing by 7.5×. One pulse therefore does not correspond to
a fixed distance, and the count rate is not a proxy for speed.

## What this invalidates from earlier the same day

Recorded so the numbers are not quoted later as if they meant something:

- **"Full stroke = 4219 counts"** (19 timed legs, ended on the extend cam).
  Cannot be reconciled with 697 counts for the same direction here.
- **"Position-dependent load profile"** — the slow region near home, the fast
  region past mid-stroke. Derived entirely from count rates, so it may be
  sensor behaviour rather than mechanism.
- **"Thermal limp"** — the collapse from ~300 counts/s to 5.3 counts/s. Same
  problem: measured in counts. The heat is real and the driver is still
  undersized, but the performance-collapse story is unsupported.
- **74% extend/retract count asymmetry**, and the 14,722-count retract that
  never reached a stop. Consistent with the same artefact.

## What still stands

- Full stroke ≈ **328 mm** (251 mm home→retract limit, 77 mm home→extend
  limit). Measured with a ruler, independent of the sensor.
- Physical travel speed ≈ **2 mm/s** at duty 200, steady across the stroke.
- Both hard stops are detected correctly by the stall watchdog.
- Noise floor with the motor off: **0 edges in 10 s** — this is not ambient
  pickup.
- Count-targeted moves (`+n`) overshoot by ~1 count with negligible coast,
  so the move primitive itself is well behaved.
- Bounce is a tight cluster under 0.2 ms, cleanly separated from real gaps
  and correctly rejected by the 1500 µs debounce.

## Next

1. **Inspect the reed and magnet physically.** Mounting security, air gap,
   whether the gap varies along the stroke, whether more than one magnet is
   in play. The noise floor being clean points at coupling, not electrical
   interference.
2. Re-run this table with more, shorter stops once the sensor is trusted.
   Steps must be time-capped — a `+n` move in a slow region silently becomes
   a 30 s continuous run, which is how the driver got cooked.
3. Only then: mm/count, and the degrees fit via `actuator_v1` (`a`/`b`).

## Hardware state

- L298N channel A is dead — OUT1 high-side shorted. `actuator_v1` and
  `reed_switch_test` are both on channel B, with channel A's pins parked low.
- The L298N runs hot and remains undersized for this actuator. A BTS7960 is
  the standing recommendation.
- The kill switch cannot stop a shorted output stage. Only opening the 24 V
  supply can.
