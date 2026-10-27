# Bringing up the SNR peaking loop on hardware

`host/snr_peak.py` has only ever run in simulation. This is the order to bring
it up on the real dish, with what counts as a pass at each step. Record each
step in the [results log](#results-log) at the end, including failures.

**The rule: each half works alone before they run together.** From inside a
sweep, an untuned PID, a broken capture and a mispointed dish all look the
same: a reading that's lower than it should be. Separately, each one is
obvious.

Commands run from `host/`, with the project's environment (on Windows,
`..\.venv\Scripts\python`). `COM3` stands for whatever port the Arduino is
on; list the ports with:

```bash
python -c "from link import ActuatorLink; print(ActuatorLink.list_ports())"
```

## 0. Before the bench

- [ ] The branch has the firmware that was tested on hardware: Luis's PID work
      merged into `main`, and `main` merged into `sdr-snr-closed-loop`.
- [ ] `python -m pytest` passes.
- [ ] If the actuator's serial protocol changed in that merge (`g`, `s`, `k`,
      the status line, the boot banner), `link.py` and `snr_peak.py` were
      updated to match.
- [ ] `iq_metric.py`'s dongle overhead is dealt with, or knowingly accepted.
      `capture_rtlsdr()` currently reopens the dongle for every block of
      samples, so a sweep runs slower, and averages fewer blocks per dwell,
      than its settings suggest. See the `SdrSnrMetric` docstring.

## 1. The actuator alone

Flash `actuator_v2` and open a serial monitor at 115200 baud.

| type | pass |
|---|---|
| `s` | `unit=deg`, `encok=1`, `fault=none` |
| `g 0`, then `s` | `state=IDLE` and `pos` within ±0.25° of 0 (the deadband) |
| `g 5`, `g -5`, `g 0` | each lands within ±0.25° and holds there for a minute |
| a panel button during a move | the button wins, and the move stops |

Then check the host side reads the same thing:

```bash
python -c "from link import ActuatorLink; l = ActuatorLink('COM3').open(); print(l.status()); l.close()"
```

Pass: the dict shows `'unit': 'deg'`, `'encoder_ok': True`, `'fault': None`,
and the same `pos` as the serial monitor did.

## 2. The dongle alone, with no signal

Do the Zadig driver swap first ([dev-setup.md](dev-setup.md#5-the-rtl-sdr)).

| check | pass |
|---|---|
| `python -c "from rtlsdr import librtlsdr; print(librtlsdr.rtlsdr_get_device_count())"` | `1` |
| `python -c "from iq_snr import capture_rtlsdr; print(len(capture_rtlsdr(1694.1e6, n_samples=16384)))"` | `16384` (samples are flowing) |
| `python iq_metric.py --watch`, with the LNA unpowered or the dish well off target | every line says `no reading: no signal distinguishable from noise` |

**The last row is the important one.** On real noise, the estimator must refuse
rather than print a number. If it prints SNRs here, a sweep would climb toward
noise. Stop and investigate before going further.

Also note the `n=` that `--watch` prints, once a signal is present (step 3):
that's how many blocks one 3 s dwell actually averaged.

## 3. The dongle with the satellite

Point the dish roughly at GOES-18, by hand or with `g`. With `config.json`
filled in, `python moon_yaw.py --dry-run` prints the look angles. Power the
LNA.

```bash
python iq_metric.py --watch
```

| check | pass |
|---|---|
| readings | numbers, steady to within about ±1 dB of each other |
| move the dish a few degrees | the SNR falls, and comes back when you return |
| unpower the LNA | back to `no reading`, not a lower number |

The field test in [dish-log.md](dish-log.md) saw an SNR of about 13 dB, but
whatever measured that wasn't this estimator. `snr_m2m4` assumes a
constant-envelope signal, and pulse shaping biases it on a filtered signal, so
expect its number to differ somewhat. What matters is that it's steady and
follows the pointing.

## 4. How noisy the readings really are

With the dish **not moving** and pointed at the satellite:

```bash
python iq_metric.py --noise --seconds 300
```

Record the single-reading stdev, the readings per second, and the table of
"min detectable step" for each dwell.

**Compare with the simulation's assumptions: 1 dB per reading, 10 readings per
second.** Every number from `--simulate` and `--trials` rests on those two.

- **Close:** the simulated accuracy figures hold.
- **Far off:** raise it before trusting them. The simulation's noise level and
  reading rate are fixed in `SimDish`, and need changing to match.

Then pick the dwell. On a 12° beam, one degree of pointing changes the SNR by
about 1 dB on the flanks (around 6° off), and almost nothing at the top. The
dwell's "min detectable step" should be well under 1 dB.

## 5. Both together

Dry-run the exact command in simulation first, so a typo costs nothing:

```bash
python snr_peak.py --simulate --sweep -8 8 1 --dwell 3 --plot
```

Then for real:

```bash
python snr_peak.py --port COM3 --sweep -8 8 1 --dwell 3 --plot
```

It refuses before moving if the sketch isn't `actuator_v2`, the controller is
faulted, the range is past ±13°, or the SDR isn't delivering samples. `Ctrl+C`
stops the motor. The kill switch always works.

| check | pass |
|---|---|
| the coarse pass | a reading at most points, rising then falling |
| `coarse peak at …; refining …` | the second pass is centred on it, and its range is printed before it moves |
| the end | `peak at … parked at …`, a reference reading, `saved to pointing.json` |
| `sweep.png` | a hill with both flanks, and the fitted curve following the dots |
| no `asked for … stopped at …` abort | every move landed within 0.6° |

If the best reading is at the edge, it says so and returns the dish to where it
started. Widen the range and run again.

## 6. Check, and repeat

```bash
python snr_peak.py --port COM3 --check
```

Pass: `OK`. Run it straight after the sweep, so nothing has changed.

Then sweep once more without overwriting the reference, starting from a
different angle:

```bash
python snr_peak.py --port COM3 --sweep -8 8 1 --dwell 3 --no-save --plot
```

Pass: the new peak is within about 0.3° of the saved one. In simulation, 90% of
parked positions land within 0.25° of the true peak, so two sweeps should agree
to about that.

## Results log

Add a row per attempt, including failures: they are the useful ones.

| date | who | step | settings (port, gain, dwell, estimator) | result | notes |
|---|---|---|---|---|---|
| | | | | | |
