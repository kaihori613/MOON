# MOON

Repurposing a 30-year-old TVRO satellite dish to receive GEO weather
satellite imagery (GOES-18 HRIT, 1694.1 MHz). Background and motivation:
[docs/dish-log.md](docs/dish-log.md).

The yaw axis is driven by a linear actuator; this repo is its firmware and
the host-side pointing tools, not the SDR/decode chain itself.

## Status

**`actuator_v2/` is the active sketch.** Closed loop on yaw angle using an
AS5600 absolute encoder on the pivot, ±15° limit switches read by the
firmware (signal only — the actuator's own internal cams are the hard
stop), a kill switch, and a reed switch kept only as a motion witness (it
no longer measures position; see the sketch's own comments for why). Full
pin map, wiring notes and bring-up cautions: [WIRING.md](WIRING.md).

**Motor driver: HW-039 (BTS7960)**, as of the L298N burning out — both
channels, on two different boards, the same way. See
[WIRING.md](WIRING.md#the-l298n-is-retired--both-channels-are-now-dead) for
why, and the voltage-margin caution right below it before trusting the
HW-039 at this rail either.

**Nothing in `actuator_v2` has been confirmed on hardware yet.** The
PID/encoder loop is next to bench-test, in the order written up for that —
encoder direction check, switches, then a small powered move — before any
SDR-based peaking work builds on top of it.

**`host/`** is the Python side: compute where a geostationary satellite is
from your coordinates, drive the actuator there over USB, and (in
progress, on its own branch) close the loop using live SNR from an SDR
instead of a human watching a signal meter. See [host/README.md](host/README.md).

## `tools/`

`syntax_check.sh` compiles every sketch (including `archive/`) with a host
`g++` against stub Arduino headers — no `arduino-cli` needed. Catches typos,
missing declarations and `#ifdef` branches that have never been built;
it does not catch anything that needs real hardware. `--matrix` also builds
every `MOTOR_DRIVER`/feature-flag permutation for `actuator_v1` and
`actuator_v2`, which is exactly the kind of thing a driver swap like the
HW-039 one should have been checked against before it went on the bench.

## `archive/`

Sketches whose bring-up job is done and whose working knowledge now lives
in `actuator_v2` and `WIRING.md`. Nothing here is deleted outright — this
project's policy is to archive, not delete, after `actuator_system/` was
deleted outright once and had to be recovered from git history, twice.

| sketch | was for |
|---|---|
| `as5600_simple/` | first AS5600 bring-up: bus scan, 9600 baud, no commands |
| `as5600_characterise/` | AGC, noise floor, averaging depth, drift — fed the constants in `actuator_v2/Config.h` |
| `as5600_bench/` | the AS5600 working copy during bring-up, reading the angle two ways side by side |
| `as5600_test/` | magnet check, count direction, wrap check |
| `switch_test/` | buttons, ±15° limit switches and the home switch, exercised in isolation before wiring into `actuator_v2` |
| `reed_switch_test/` | the reed-as-position-sensor bench, from before the AS5600 replaced that approach entirely |
| `actuator_v1/` | Rev A: reed-counted position, superseded by `actuator_v2`'s absolute encoder. Design rationale worth keeping lives in [docs/design-notes.md](docs/design-notes.md) |
| `actuator_test/`, `l298n_test/` | early L298N/motor bring-up, superseded by `actuator_v1` |
| `cam_switch_test/` | cam end-stop repeatability |

## `docs/`

| | |
|---|---|
| [dish-log.md](docs/dish-log.md) | the project pitch: what this is and why |
| [design-notes.md](docs/design-notes.md) | rationale from `actuator_v1` that exists nowhere else — anti-backlash policy, why zero sat at mid-stroke, why a saved position is checked rather than trusted |

Dated bench-session logs (calibration runs, linkage-geometry derivations,
the reed-trust procedure) have been removed from the working tree as
superseded — `actuator_v2`'s absolute encoder does not need any of that
reed-counted calibration. They are still in git history if ever needed
again.
