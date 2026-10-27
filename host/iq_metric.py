"""
iq_metric.py
------------
Bridge from iq_snr.py's raw-IQ SNR estimators to the (mean, stdev, n) shape
metric.py's Metric.sample() already produces, so a later peak.sweep() can
search on live SDR SNR exactly the way it already searches on a decoder
metric -- same call, same contract, peak.py does not need to know the
number came from an SDR instead of goesrecv or SatDump.

snr_peak.py now drives the actuator with this as its metric, but run this
file ON ITS OWN first. iq_snr.capture_rtlsdr() carries its own warning that
it has never pulled a sample from real hardware; this file has not either.
Point the dongle at the sky and get a believable SNR number out of --watch
before trusting this for anything that moves a motor -- a search has no
way to tell "good reading" from "capture is broken and returning garbage"
on its own, which is exactly the m2m4 floor-refusal problem in iq_snr.py
one level up. docs/snr-bringup.md has the order.

    python3 iq_metric.py --watch                          repeated readings
    python3 iq_metric.py --watch --signal-bw 1200000       use snr_psd (numpy)
    python3 iq_metric.py --noise --seconds 120             noise floor

Frequency defaults to 1694.1 MHz, GOES-18 HRIT -- the same number the
beamwidth math in actuator_v2/Config.h and host/README.md already assume.
Override with --freq for anything else.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time

from iq_snr import SnrError, capture_rtlsdr, snr_m2m4, snr_psd
from metric import block_means

GOES18_HRIT_HZ = 1_694_100_000.0

try:
    import numpy  # noqa: F401
    _HAVE_NUMPY = True
except ImportError:
    _HAVE_NUMPY = False


class IqMetricError(RuntimeError):
    pass


class SdrSnrMetric:
    """
    SNR read from a live RTL-SDR capture, one block at a time.

    .sample(dwell, settle) returns (mean, stdev, n) -- the same three
    numbers metric.py's Metric.sample() returns -- by capturing blocks back
    to back for about `dwell` seconds and averaging the per-block estimate.
    lower_is_better is always False: this is SNR in dB, higher is better,
    never the Viterbi-style count that falls as pointing improves.

    NOT TUNED FOR A TIGHT DWELL LOOP YET, AND NOW IT NEEDS TO BE.
    capture_rtlsdr() opens and closes the device on every call, which on most
    RTL-SDR units costs tens to hundreds of milliseconds of USB and PLL
    settling on top of the samples themselves. snr_peak.py calls this inside
    every sweep point, so that overhead is paid many times per dwell: the
    sweep runs slower, and a 3 s dwell averages fewer blocks than it looks
    like it should -- check n in --watch's output. The fix is to hold the
    device open across a dwell, or a whole sweep. Do it before the first
    real sweep; it can only be confirmed with a dongle plugged in.
    """

    lower_is_better = False

    def __init__(self, freq_hz, fs=2_400_000, n_samples=1 << 18,
                 gain="auto", device_index=0, signal_bw=None,
                 estimator="auto"):
        if estimator not in ("auto", "psd", "m2m4"):
            raise ValueError("estimator must be 'auto', 'psd' or 'm2m4'")
        if estimator == "psd" and not _HAVE_NUMPY:
            raise IqMetricError(
                "snr_psd needs numpy (pip install numpy), or pass "
                "estimator='m2m4' / leave signal_bw unset")

        self.freq_hz = freq_hz
        self.fs = fs
        self.n_samples = n_samples
        self.gain = gain
        self.device_index = device_index
        self.signal_bw = signal_bw
        self.estimator = estimator

    def _use_psd(self) -> bool:
        if self.estimator == "psd":
            return True
        if self.estimator == "m2m4":
            return False
        # auto: psd needs both numpy and a bandwidth nobody has guessed for
        # you here. Without both, fall back to the dependency-free estimator
        # rather than silently picking a signal_bw -- see iq_snr.py for why
        # that number is not something this project invents on your behalf.
        return _HAVE_NUMPY and self.signal_bw is not None

    def _one_reading(self) -> float:
        iq = capture_rtlsdr(self.freq_hz, fs=self.fs, n_samples=self.n_samples,
                            gain=self.gain, device_index=self.device_index)
        if self._use_psd():
            return snr_psd(iq, self.fs, self.signal_bw)
        return snr_m2m4(iq)

    def sample(self, dwell: float, settle: float = 0.0):
        """
        Average per-block SNR over about `dwell` seconds.

        Always captures at least one block, even if dwell is 0 -- a caller
        asking for an instant reading should get one, not an empty window.
        Raises IqMetricError if every block in the window failed, which is
        the honest outcome when there is nothing on the air at this
        frequency: a search must stop here, not treat silence as a number.
        """
        if settle > 0:
            time.sleep(settle)

        deadline = time.time() + dwell
        values = []
        last_error = None
        while True:
            try:
                values.append(self._one_reading())
            except SnrError as exc:
                last_error = str(exc)
            if time.time() >= deadline:
                break

        if not values:
            msg = f"no usable SNR reading in {dwell:.1f}s"
            if last_error:
                msg += f": {last_error}"
            raise IqMetricError(msg)

        mean = statistics.fmean(values)
        stdev = statistics.stdev(values) if len(values) > 1 else 0.0
        return mean, stdev, len(values)

    def better(self, a: float, b: float) -> bool:
        return a > b


# ---------------------------------------------------------------------------
#  Noise floor -- the same question metric.py asks, answered from the SDR
# ---------------------------------------------------------------------------

def noise_floor(metric: SdrSnrMetric, seconds: float,
                dwells=(1.0, 2.0, 5.0, 10.0)):
    """
    Capture SNR readings with the dish NOT MOVING and report how much the
    averaged reading still wanders -- metric.py's noise_floor(), aimed at
    the SDR instead of a statsd stream. See metric.py for why this question
    comes before building any search on a sensor.
    """
    print(f"measuring SNR at {metric.freq_hz / 1e6:.3f} MHz for {seconds:.0f}s "
          "-- DO NOT MOVE THE DISH")
    rows = []
    t0 = time.time()
    last_report = t0
    while time.time() - t0 < seconds:
        try:
            rows.append((time.time(), metric._one_reading()))
        except SnrError:
            pass
        if time.time() - last_report >= 10.0:
            last_report = time.time()
            print(f"  {time.time() - t0:5.0f}s  {len(rows)} readings")

    if len(rows) < 2:
        raise IqMetricError(
            f"only {len(rows)} usable readings in {seconds:.0f}s -- check the "
            "frequency, the gain, and that there is actually something on "
            "the air at this frequency right now")

    values = [v for (_, v) in rows]
    rate = len(rows) / (rows[-1][0] - rows[0][0]) if len(rows) > 1 else 0.0

    print()
    print(f"readings  {len(values)}  ({rate:.1f}/s)")
    print(f"mean      {statistics.fmean(values):.3f} dB")
    print(f"stdev     {statistics.stdev(values):.3f} dB   (single reading)")
    print(f"min/max   {min(values):.3f} / {max(values):.3f} dB")
    print()
    print("  dwell    blocks   sigma of the averaged reading   min detectable step")
    MIN_BLOCKS = 8
    thin = False
    for d in dwells:
        means = block_means(rows, d)
        if len(means) < MIN_BLOCKS:
            thin = True
            note = f"only {len(means)} blocks -- run at least {d * MIN_BLOCKS:.0f}s"
            print(f"  {d:5.1f}s   {len(means):5d}   {note:>22}")
            continue
        sd = statistics.stdev(means)
        print(f"  {d:5.1f}s   {len(means):5d}   {sd:>22.4g}   {3 * sd:.4g}")

    if thin:
        print()
        print(f"Rows marked above had fewer than {MIN_BLOCKS} blocks. Run longer "
              "before trusting them.")

    print()
    print("Pick the smallest dwell whose 'min detectable step' is below the SNR")
    print("change one pointing step actually produces. Measure that by nudging")
    print("the dish once -- by hand is fine for this test -- and comparing, "
          "before\ntrusting a search built on this number.")


# ---------------------------------------------------------------------------
#  CLI
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Standalone SDR SNR check -- no actuator, no search.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--freq", type=float, default=GOES18_HRIT_HZ,
                    help=f"center frequency, Hz (default {GOES18_HRIT_HZ/1e6:.1f} MHz, GOES-18 HRIT)")
    ap.add_argument("--fs", type=float, default=2_400_000.0, help="sample rate, Hz")
    ap.add_argument("--n-samples", type=int, default=1 << 18)
    ap.add_argument("--gain", default="auto")
    ap.add_argument("--device-index", type=int, default=0)
    ap.add_argument("--signal-bw", type=float, default=None,
                    help="occupied bandwidth, Hz -- enables snr_psd (needs numpy). "
                         "Left unset, this uses snr_m2m4 instead of guessing a "
                         "bandwidth nobody measured.")
    ap.add_argument("--estimator", choices=("auto", "psd", "m2m4"), default="auto")
    ap.add_argument("--watch", action="store_true",
                    help="print one averaged SNR reading, repeated")
    ap.add_argument("--noise", action="store_true",
                    help="noise floor over --seconds; the dish must not move")
    ap.add_argument("--dwell", type=float, default=3.0,
                    help="seconds averaged per --watch reading")
    ap.add_argument("--seconds", type=float, default=120.0,
                    help="total duration for --noise")
    args = ap.parse_args(argv)

    if not (args.watch or args.noise):
        ap.error("pick --watch or --noise")

    try:
        metric = SdrSnrMetric(args.freq, fs=args.fs, n_samples=args.n_samples,
                              gain=args.gain, device_index=args.device_index,
                              signal_bw=args.signal_bw, estimator=args.estimator)
    except IqMetricError as exc:
        print(exc, file=sys.stderr)
        return 2

    try:
        if args.watch:
            print(f"watching SNR at {args.freq / 1e6:.3f} MHz, {args.dwell:.1f}s "
                  f"dwell, estimator={'psd' if metric._use_psd() else 'm2m4'} "
                  "-- Ctrl+C to stop")
            while True:
                try:
                    mean, stdev, n = metric.sample(args.dwell)
                    print(f"  snr {mean:7.2f} dB   stdev {stdev:5.2f}   n={n}")
                except IqMetricError as exc:
                    print(f"  no reading: {exc}")

        elif args.noise:
            noise_floor(metric, args.seconds)

    except KeyboardInterrupt:
        pass
    except IqMetricError as exc:
        print(exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
