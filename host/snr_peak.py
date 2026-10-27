#!/usr/bin/env python3
"""
snr_peak.py
-----------
Peak the dish on live SDR SNR: the outer loop around actuator_v2's PID.

    peak.sweep()  ---- "g <deg>" over USB ---->  actuator_v2: PID on angle, 50 Hz
         ^                                              |
         +---- SdrSnrMetric.sample()  <---- RTL-SDR, read once the PID has landed

The Arduino runs PID on ANGLE, where the error has a sign. This runs a SEARCH
on SNR, where it does not: SNR against angle is a hill, and the same reading
occurs on both sides of it. See peak.py.

    python snr_peak.py --simulate --sweep -8 8 1      whole loop, no hardware
    python snr_peak.py --simulate --sweep -8 8 1 --plot   ...and draw it to sweep.png
    python snr_peak.py --port COM3 --sweep -8 8 1     find the peak, park, save
    python snr_peak.py --port COM3 --check            is the signal still what it was?

Angles are actuator_v2's: absolute degrees from wherever 'z' set zero.

Run this only once both halves work on their own -- the angle loop holding a
'g' target typed into the serial console, and `iq_metric.py --watch` giving
believable numbers from the real dongle. From in here, an untuned PID or a
broken capture looks exactly like a dish that is simply mispointed.

The RTL-SDR is single-client: close SDR# or SatDump before running this.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import random
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from iq_metric import GOES18_HRIT_HZ, IqMetricError, SdrSnrMetric
from iq_snr import parse_gain
from peak import PointingReference, check_pointing, fit_peak, sweep

# Mirrors of actuator_v2/Config.h. The sketch enforces its own copy of the
# soft limit, but a target past it is not merely refused: it latches
# fault=range, and every status reads as faulted until 'k'. So refuse here,
# before anything is sent.
SOFT_LIMIT_DEG = 13.0
DEADBAND_DEG = 0.25

# A landing further than this from the target means the move did not happen
# as asked -- refused, pre-empted by a panel button, or stopped short -- and a
# reading taken there would be credited to the wrong angle.
LAND_TOL_DEG = 0.6

BEAMWIDTH_DEG = 12.0        # 1 m dish at 1694 MHz, as in actuator_v2/Config.h

# The refining pass: half a beamwidth either side of the coarse peak, at half
# the coarse step. Narrower would be worse, not better -- see refine_range().
REFINE_HALF_WIDTH_DEG = BEAMWIDTH_DEG / 2
REFINE_STEP_FACTOR = 0.5
MOVE_TIMEOUT_S = 45.0       # the sketch's own MAX_RUN_MS is 30 s
DEFAULT_REF = Path(__file__).with_name("pointing.json")
DEFAULT_PLOT = Path(__file__).with_name("sweep.png")


class PeakRunError(RuntimeError):
    pass


# ===========================================================================
#  The actuator, as peak.sweep() needs it
# ===========================================================================

class Actuator:
    """move_to() sends the target and blocks until the PID has landed on it."""

    def __init__(self, link):
        self.link = link

    def position(self) -> float:
        st = self.link.status()
        self._check_encoder(st)
        return st["pos"]

    def move_to(self, angle: float) -> float:
        self.link.move_to_deg(angle)
        st = self.link.wait_idle(timeout=MOVE_TIMEOUT_S)    # raises on FAULT
        self._check_encoder(st)
        if abs(st["pos"] - angle) > LAND_TOL_DEG:
            raise PeakRunError(
                f"asked for {angle:+.3f} deg, stopped at {st['pos']:+.3f}. The "
                "move was refused or pre-empted (a panel button always wins "
                "over the host).")
        return st["pos"]

    @staticmethod
    def _check_encoder(st):
        # A 'g' with no encoder is refused without a fault, so the controller
        # goes straight back to IDLE where it was. Without this check the
        # sweep would read SNR at one angle and log it against another.
        if not st["encoder_ok"]:
            raise PeakRunError("the controller reports no encoder (encok=0)")


# ===========================================================================
#  A fake dish, so the loop can run before any hardware does
# ===========================================================================

def bessel_j1(x: float) -> float:
    """
    J1 by its power series, which is exact enough for the Airy beam below:
    there |x| stays under about 6, where 30 terms agree with scipy to 1e-12.
    Kept here rather than importing scipy so --simulate needs nothing.
    """
    term = x / 2.0
    total = term
    for m in range(1, 30):
        term *= -(x * x / 4.0) / (m * (m + 1))
        total += term
    return total


# Where the Airy pattern is 3 dB down: 2*J1(u)/u = 1/sqrt(2) at this u.
_AIRY_HALF_POWER_U = 1.6163


def beam_loss_db(err_deg, beamwidth_deg, beam="parabola") -> float:
    """
    How far below the peak a reading `err_deg` off-axis is, in dB (>= 0).

    parabola  12*(err/bw)^2: the law the fit assumes, so it can only show
              noise, never a model error
    airy      a uniformly lit circular aperture, scaled so it is 3 dB down
              at err = bw/2 like the parabola. It matches the parabola to
              0.03 dB out to bw/2, then falls FASTER down the flanks -- 7.4 dB
              at 0.75*bw against the parabola's 6.75, 16.3 dB at bw against
              12 -- which is what makes a fit to one flank lean. A real dish
              sits somewhere between the two.
    """
    if beam == "parabola":
        return 12.0 * (err_deg / beamwidth_deg) ** 2
    if beam == "airy":
        u = _AIRY_HALF_POWER_U * 2.0 * err_deg / beamwidth_deg
        if abs(u) < 1e-9:
            return 0.0
        g = (2.0 * bessel_j1(u) / u) ** 2
        return -10.0 * math.log10(max(g, 1e-12))
    raise ValueError(f"unknown beam {beam!r}")


class SimDish:
    """
    Stands in for both the actuator and the SDR.

    The beam is `beam` (see beam_loss_db). Every reading carries Gaussian
    noise, every move lands somewhere inside the PID's deadband, and below
    SIM_FLOOR_DB it refuses the way snr_m2m4 refuses on pure noise.
    """

    SIM_FLOOR_DB = 1.0

    def __init__(self, peak_deg, peak_snr_db=12.0, beamwidth_deg=BEAMWIDTH_DEG,
                 noise_db=1.0, readings_per_s=10.0, seed=None, beam="parabola"):
        beam_loss_db(0.0, beamwidth_deg, beam)          # refuse an unknown beam now
        self.peak_deg = peak_deg
        self.peak_snr_db = peak_snr_db
        self.beamwidth_deg = beamwidth_deg
        self.noise_db = noise_db
        self.readings_per_s = readings_per_s
        self.beam = beam
        self.pos = 0.0
        self.rng = random.Random(seed)

    def true_db(self, angle: float) -> float:
        return self.peak_snr_db - beam_loss_db(angle - self.peak_deg,
                                               self.beamwidth_deg, self.beam)

    def position(self) -> float:
        return self.pos

    def move_to(self, angle: float) -> float:
        if abs(angle) > SOFT_LIMIT_DEG:
            raise PeakRunError(f"(sim) {angle:+.2f} deg is past the soft limit")
        self.pos = angle + self.rng.uniform(-DEADBAND_DEG, DEADBAND_DEG)
        return self.pos

    def sample(self, dwell: float, settle: float = 0.0):
        true_db = self.true_db(self.pos)
        if true_db < self.SIM_FLOOR_DB:
            raise IqMetricError(f"(sim) no signal at {self.pos:+.2f} deg")
        n = max(2, int(dwell * self.readings_per_s))
        values = [true_db + self.rng.gauss(0.0, self.noise_db) for _ in range(n)]
        return statistics.fmean(values), statistics.stdev(values), n


# ===========================================================================
#  Sweep and check
# ===========================================================================

def validate_range(lo, hi, step) -> int:
    if step <= 0:
        raise PeakRunError("STEP must be positive")
    if hi <= lo:
        raise PeakRunError("HI must be greater than LO")
    if lo < -SOFT_LIMIT_DEG or hi > SOFT_LIMIT_DEG:
        raise PeakRunError(
            f"the sweep must stay inside +/-{SOFT_LIMIT_DEG:g} deg, the "
            "sketch's soft limit")
    n = int(math.floor((hi - lo) / step + 1e-9)) + 1
    if n < 5:
        # A best reading at either end counts as a failure by design, and
        # the fit needs readings on the flanks to mean anything.
        raise PeakRunError(f"{n} points is too few -- use at least 5")
    return n


def _return_to(mover, angle):
    if abs(angle) > SOFT_LIMIT_DEG:
        print(f"  not returning to {angle:+.3f} deg: it is past the soft limit",
              file=sys.stderr)
        return
    try:
        mover.move_to(angle)
        print(f"  returned to {angle:+.3f} deg, where the dish started")
    except Exception as exc:
        print(f"  could not return to {angle:+.3f} deg: {exc}", file=sys.stderr)


def refine_range(center, step):
    """
    The second pass: centred on the coarse peak, half a beamwidth each side,
    at half the coarse step, clipped only to the soft limit.

    Why this shape, from simulation (1 dB per reading, an Airy beam rather
    than the parabola the fit assumes, 500 runs per row):

      * NARROW (+/-2 deg at 0.25 deg) is about three times WORSE than no
        second pass at all, with 2 deg outliers. Every new reading sits on
        the flat top, where the beam changes by less than the noise.
      * WIDE and RECENTRED fixes a bias that no amount of averaging does.
        With the peak near the end of the coarse range, the coarse points
        all sit on one flank, and a parabola fitted to one side of a beam
        that is not quite a parabola leans toward it: median 0.25 deg off,
        worst 0.86, and the same with 2.5x the dwell. Recentred, it was
        0.04 deg median and 0.19 worst.

    That is also why it may run past the --sweep range: kept inside it, an
    edge peak gets only one flank again, and the error went back to 0.22 deg
    median. The soft limit is the sketch's own, so it still holds.
    """
    lo = max(center - REFINE_HALF_WIDTH_DEG, -SOFT_LIMIT_DEG)
    hi = min(center + REFINE_HALF_WIDTH_DEG, SOFT_LIMIT_DEG)
    return lo, hi, step * REFINE_STEP_FACTOR


def run_sweep(mover, metric, lo, hi, step, settle, dwell, ref_dwell, metric_name,
              refine=True):
    """
    Returns (PointingReference or None, PeakResult or None).

    With refine, a coarse pass finds the peak roughly, a second pass is swept
    around it (see refine_range), and the peak is fitted to the readings from
    both. The returned PeakResult is that combined fit; its .coarse is the
    first pass.

    Failure puts the dish back where it was before the sweep. peak.sweep()'s
    own return_on_failure goes back to the FIRST SWEPT angle instead, which
    is not the old aim -- so that is switched off and done here.
    """
    start = mover.position()
    n = validate_range(lo, hi, step)
    print(f"sweeping {lo:+.2f} .. {hi:+.2f} deg in {step:g} deg steps "
          f"({n} points), {settle:g} s settle + {dwell:g} s dwell each")
    print(f"  starting from {start:+.3f} deg")

    def on_point(angle, score, err):
        if score is None:
            print(f"  {angle:+7.2f} deg   no reading -- {err}")
        else:
            print(f"  {angle:+7.2f} deg   {score:6.2f} dB")

    def one_pass(a, b, s, park):
        return sweep(mover.move_to, lambda: metric.sample(dwell), a, b, s,
                     settle=settle, dwell=dwell, lower_is_better=False,
                     on_point=on_point, return_on_failure=False, park=park)

    try:
        result = one_pass(lo, hi, step, park=not refine)
        if refine and result.ok:
            coarse = result
            flo, fhi, fstep = refine_range(coarse.best_angle, step)
            nf = int(math.floor((fhi - flo) / fstep + 1e-9)) + 1
            print(f"coarse peak at {coarse.best_angle:+.3f} deg; refining "
                  f"{flo:+.2f} .. {fhi:+.2f} deg in {fstep:g} deg steps "
                  f"({nf} points)")
            fine = one_pass(flo, fhi, fstep, park=False)
            result = fit_peak(coarse.samples + fine.samples)
            if not result.ok:
                # Not expected -- the coarse fit already found a peak inside
                # these readings -- but if the combined fit says no, keep the
                # answer that was found rather than throw the sweep away.
                print(f"  combined fit failed ({result.reason}); "
                      "keeping the coarse peak")
                result = fit_peak(coarse.samples)
            result.coarse = coarse
            mover.move_to(result.best_angle)
    except Exception:
        _return_to(mover, start)
        raise

    if not result.ok:
        print(f"no peak: {result.reason}")
        _return_to(mover, start)
        return None, result

    # sweep() has already parked on the peak. Measure there again, for longer
    # than a sweep point: this reading is the standard every later --check is
    # held to, so it is worth the extra seconds.
    parked = mover.position()
    if settle:
        time.sleep(settle)
    mean, stdev, nread = metric.sample(ref_dwell)
    print(f"peak at {result.best_angle:+.3f} deg"
          f"{' (fitted between points)' if result.interpolated else ''}, "
          f"parked at {parked:+.3f}")
    print(f"  reference reading: {mean:.2f} dB, stdev {stdev:.2f}, n={nread}")

    ref = PointingReference(
        parked, mean, stdev, metric_name=metric_name, lower_is_better=False,
        when=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        beamwidth_deg=BEAMWIDTH_DEG, metric_n=nread)
    return ref, result


def run_check(mover, metric, ref, settle, dwell):
    print(f"moving to the reference angle {ref.angle:+.3f} deg")
    mover.move_to(ref.angle)
    if settle:
        time.sleep(settle)
    verdict = check_pointing(ref, lambda: metric.sample(dwell))
    print(f"  {verdict.status.upper()}: {verdict.message}")
    if verdict.needs_repeak:
        print("  run --sweep again to re-peak. This never re-peaks on its own:")
        print("  a drop can be rain or a wet LNA, and re-peaking on that stores")
        print("  a worse aim than the one already saved.")
    return verdict


# ===========================================================================
#  The saved reference
# ===========================================================================

def sdr_settings(args) -> dict:
    return {"freq_hz": args.freq, "fs": args.fs, "n_samples": args.n_samples,
            "gain": args.gain, "device_index": args.device_index,
            "signal_bw": args.signal_bw, "estimator": args.estimator}


def metric_from(s: dict) -> SdrSnrMetric:
    return SdrSnrMetric(s["freq_hz"], fs=s["fs"], n_samples=s["n_samples"],
                        gain=s["gain"], device_index=s["device_index"],
                        signal_bw=s["signal_bw"], estimator=s["estimator"])


def preflight_sdr(s: dict):
    """
    Pull one block of raw samples before anything moves.

    Inside a sweep every failure reads as "no reading at this angle", so a
    missing pyrtlsdr or an unplugged dongle would otherwise cost a full sweep
    of the dish to discover. No signal is NOT an error at this level -- only
    the estimators refuse on noise -- so a mispointed dish still passes.
    """
    from iq_snr import capture_rtlsdr
    try:
        iq = capture_rtlsdr(s["freq_hz"], fs=s["fs"], n_samples=16384,
                            gain=s["gain"], device_index=s["device_index"])
    except Exception as exc:
        raise PeakRunError(f"the SDR is not delivering samples: {exc}") from exc
    if len(iq) == 0:
        raise PeakRunError("the SDR returned an empty capture")


def describe_metric(metric: SdrSnrMetric) -> str:
    est = "psd" if metric._use_psd() else "m2m4"
    return f"sdr-snr {est} @ {metric.freq_hz / 1e6:.3f} MHz"


def save_reference(path: Path, ref, settings, result, lo, hi, step):
    data = {
        "_comment": "Written by snr_peak.py --sweep. --check reuses the 'sdr' "
                    "settings exactly: an SNR from a different estimator, gain "
                    "or bandwidth is not comparable to this reference.",
        "reference": ref.to_dict(),
        "sdr": settings,
        "sweep": {"lo": lo, "hi": hi, "step": step,
                  "interpolated": result.interpolated,
                  "points": [[a, s] for (a, s, _sd, _n) in result.samples]},
    }
    if result.coarse is not None:
        data["sweep"]["coarse_peak"] = result.coarse.best_angle
        data["sweep"]["coarse_points"] = len(result.coarse.samples)
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f"  saved to {path}")


def load_reference(path: Path):
    if not path.exists():
        raise PeakRunError(f"no reference at {path} -- run --sweep first")
    data = json.loads(path.read_text(encoding="utf-8"))
    return PointingReference.from_dict(data["reference"]), data["sdr"]


def save_plot(path, result, lo, hi, **kw):
    """
    Draw the sweep. A missing matplotlib is reported, not raised: by the time
    this runs the dish has already moved and parked, and a picture is not
    worth turning a good sweep into a failed one.
    """
    from sweep_plot import plot_sweep
    try:
        print(f"  plot saved to {plot_sweep(path, result, lo, hi, **kw)}")
    except (RuntimeError, OSError) as exc:
        print(f"  no plot: {exc}", file=sys.stderr)


def _pct(values, q):
    """q-th percentile by nearest rank; values need not be sorted."""
    s = sorted(values)
    return s[min(len(s) - 1, int(q / 100.0 * len(s)))]


def run_trials(n, peak_deg, beam, lo, hi, step, dwell, ref_dwell, refine,
               seed=None, plot=None):
    """
    Monte Carlo: n simulated sweeps, each with fresh noise and fresh landing
    offsets, and the spread of where the peak came out.

    The SIGNED mean is the number to watch. Noise scatters the answer around
    the true peak; a model error -- the parabola fitted to one flank of a
    beam that is not one -- moves the whole scatter to one side, and
    averaging over more trials does not move it back.

    Returns a dict of the statistics, and prints them.
    """
    if seed is None:
        seed = random.randrange(1 << 31)
    fitted, parked, points = [], [], []
    reasons = {}

    for i in range(n):
        dish = SimDish(peak_deg, seed=seed + i, beam=beam)
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                ref, result = run_sweep(dish, dish, lo, hi, step, settle=0.0,
                                        dwell=dwell, ref_dwell=ref_dwell,
                                        metric_name="simulated", refine=refine)
        except (PeakRunError, IqMetricError) as exc:
            reasons[str(exc)] = reasons.get(str(exc), 0) + 1
            continue
        if ref is None:
            reasons[result.reason] = reasons.get(result.reason, 0) + 1
            continue
        fitted.append(result.best_angle - peak_deg)
        parked.append(ref.angle - peak_deg)
        points.append(len(result.samples))

    passes = "two passes" if refine else "one pass (--no-refine)"
    print(f"{n} simulated sweeps: {beam} beam, true peak {peak_deg:+.2f} deg, "
          f"{lo:+g}..{hi:+g} deg step {step:g}, {passes}, "
          f"seeds {seed}..{seed + n - 1}")
    print(f"  peak found in {len(fitted)}/{n}")
    for reason, count in sorted(reasons.items(), key=lambda kv: -kv[1]):
        print(f"    {count:4d} x {reason}")
    if not fitted:
        return {"found": 0, "trials": n}

    print()
    print("                       |error|: median     p90     max    signed mean")
    for name, errs in (("fitted peak", fitted), ("where it parked", parked)):
        a = [abs(e) for e in errs]
        print(f"  {name:18s}           {statistics.median(a):6.3f}  "
              f"{_pct(a, 90):6.3f}  {max(a):6.3f}    {statistics.fmean(errs):+7.3f}")
    worst = max(abs(e) for e in parked)
    print()
    print(f"  worst parked error {worst:.3f} deg costs "
          f"{beam_loss_db(worst, BEAMWIDTH_DEG, beam):.3f} dB on this beam")
    print(f"  {statistics.fmean(points):.0f} points per sweep")

    if plot:
        from sweep_plot import plot_trials
        try:
            title = (f"{n} simulated sweeps, {beam} beam, true peak "
                     f"{peak_deg:+.2f}°, {lo:+g}..{hi:+g} step {step:g}, {passes}")
            print(f"  plot saved to {plot_trials(plot, fitted, parked, title)}")
        except (RuntimeError, OSError) as exc:
            print(f"  no plot: {exc}", file=sys.stderr)

    return {"found": len(fitted), "trials": n, "fitted": fitted,
            "parked": parked, "reasons": reasons, "seed": seed}


# ===========================================================================

def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sweep", nargs=3, type=float, metavar=("LO", "HI", "STEP"),
                      help="sweep LO..HI degrees, park on the peak, save it")
    mode.add_argument("--check", action="store_true",
                      help="go to the saved angle and compare SNR with the reference")

    ap.add_argument("--port", help="serial port, e.g. COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--ref", type=Path, default=DEFAULT_REF,
                    help=f"reference file (default {DEFAULT_REF.name})")
    ap.add_argument("--no-save", action="store_true",
                    help="sweep without overwriting the reference file")
    ap.add_argument("--plot", nargs="?", type=Path, const=DEFAULT_PLOT,
                    metavar="PNG",
                    help=f"draw the sweep to a PNG (default {DEFAULT_PLOT.name}); "
                         "needs matplotlib")
    ap.add_argument("--no-refine", dest="refine", action="store_false",
                    help="skip the second, finer pass around the coarse peak. It "
                         "runs half a beamwidth either side of that peak at half "
                         "the step, so it may go past LO..HI (never past the "
                         f"+/-{SOFT_LIMIT_DEG:g} deg soft limit), and roughly "
                         "2.5x the sweep time")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="seconds to wait after each move before measuring")
    ap.add_argument("--dwell", type=float, default=3.0,
                    help="seconds of SNR averaged at each sweep point")
    ap.add_argument("--ref-dwell", type=float, default=10.0,
                    help="seconds averaged for the saved reference reading")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="echo every serial line")

    sdr = ap.add_argument_group("SDR (sweep only; --check reuses the saved settings)")
    sdr.add_argument("--freq", type=float, default=GOES18_HRIT_HZ)
    sdr.add_argument("--fs", type=float, default=2_400_000.0)
    sdr.add_argument("--n-samples", type=int, default=1 << 18)
    sdr.add_argument("--gain", type=parse_gain, default="auto",
                     help="tuner gain in dB, or 'auto' for its AGC (default)")
    sdr.add_argument("--device-index", type=int, default=0)
    sdr.add_argument("--signal-bw", type=float, default=None,
                     help="occupied bandwidth, Hz; enables snr_psd (needs numpy)")
    sdr.add_argument("--estimator", choices=("auto", "psd", "m2m4"), default="auto")

    sim = ap.add_argument_group("simulation")
    sim.add_argument("--simulate", action="store_true",
                     help="fake dish and SDR: a sweep, then a check, nothing saved")
    sim.add_argument("--sim-peak", type=float, default=2.7,
                     help="where the fake beam peaks, deg")
    sim.add_argument("--sim-seed", type=int, default=None,
                     help="repeat a run exactly; with --trials, the first seed")
    sim.add_argument("--sim-beam", choices=("parabola", "airy"), default="parabola",
                     help="the fake beam's shape. parabola is what the fit "
                          "assumes, so it only ever shows noise; airy is a real "
                          "circular dish's pattern, which also shows the fit's "
                          "model error")
    sim.add_argument("--trials", type=int, default=1, metavar="N",
                     help="Monte Carlo: run N simulated sweeps and print how far "
                          "the peak landed from the truth; with --plot, a "
                          "histogram")
    args = ap.parse_args(argv)

    errors = (PeakRunError, IqMetricError)
    if args.plot and args.check:
        ap.error("--plot draws a sweep; use it with --sweep")
    if args.trials != 1 and not args.simulate:
        ap.error("--trials needs --simulate")
    if args.trials < 1:
        ap.error("--trials must be at least 1")

    # --- simulation --------------------------------------------------------
    if args.simulate:
        if args.check:
            ap.error("--simulate runs a sweep followed by a check; use --sweep")
        lo, hi, step = args.sweep
        try:
            validate_range(lo, hi, step)
        except PeakRunError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2
        if args.trials > 1:
            stats = run_trials(args.trials, args.sim_peak, args.sim_beam, lo, hi,
                               step, args.dwell, args.ref_dwell, args.refine,
                               seed=args.sim_seed, plot=args.plot)
            return 0 if stats["found"] else 1
        dish = SimDish(args.sim_peak, seed=args.sim_seed, beam=args.sim_beam)
        try:
            ref, result = run_sweep(dish, dish, lo, hi, step, settle=0.0,
                                    dwell=args.dwell, ref_dwell=args.ref_dwell,
                                    metric_name="simulated", refine=args.refine)
            verdict = None
            if ref is not None:
                print(f"  true peak {args.sim_peak:+.3f} deg, found "
                      f"{ref.angle:+.3f} -- off by {ref.angle - args.sim_peak:+.3f}")
                print()
                verdict = run_check(dish, dish, ref, settle=0.0, dwell=args.dwell)
        except errors as exc:
            print(f"\nerror: {exc}", file=sys.stderr)
            return 1
        if args.plot:
            save_plot(args.plot, result, lo, hi, ref=ref, verdict=verdict,
                      title=f"simulated sweep {lo:+g}..{hi:+g} deg, step {step:g}, "
                            f"{args.sim_beam} beam",
                      true_beam=(dish.peak_deg, dish.true_db))
        return 0 if ref is not None else 1

    # --- hardware ----------------------------------------------------------
    if not args.port:
        ap.error("--port is required (or --simulate)")

    # Imported here so --simulate needs neither pyserial nor an Arduino.
    from link import ActuatorError, ActuatorLink
    errors = errors + (ActuatorError,)

    try:
        if args.check:
            ref, settings = load_reference(args.ref)
        else:
            settings = sdr_settings(args)
            validate_range(*args.sweep)
        # Opens nothing yet: the dongle is opened on the first reading, inside
        # the try below whose finally closes it. preflight_sdr opens and
        # closes its own.
        metric = metric_from(settings)
        preflight_sdr(settings)
    except errors as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    link = ActuatorLink(args.port, args.baud, verbose=args.verbose)
    try:
        link.open()
    except Exception as exc:
        print(f"could not open {args.port}: {exc}", file=sys.stderr)
        return 2

    try:
        st = link.status()
        if st["unit"] != "deg":
            raise PeakRunError(
                "the sketch on this port is not actuator_v2 (unit is not deg). "
                "actuator_v1 reads 'g' as reed counts, so nothing is sent.")
        if st["fault"]:
            raise PeakRunError(
                f"the controller is faulted ({st['fault']}). Find out why, "
                "clear it with 'k' in the serial console, then run this again.")

        mover = Actuator(link)
        if args.check:
            verdict = run_check(mover, metric, ref, args.settle, args.dwell)
            return 0 if not verdict.needs_repeak and verdict.status != "no-signal" else 1

        lo, hi, step = args.sweep
        ref, result = run_sweep(mover, metric, lo, hi, step, args.settle,
                                args.dwell, args.ref_dwell, describe_metric(metric),
                                refine=args.refine)
        if args.plot:
            save_plot(args.plot, result, lo, hi, ref=ref,
                      title=f"{describe_metric(metric)}, {lo:+g}..{hi:+g} deg, "
                            f"step {step:g}")
        if ref is None:
            return 1
        if args.no_save:
            print("  not saved (--no-save)")
        else:
            save_reference(args.ref, ref, settings, result, lo, hi, step)
        return 0

    except errors as exc:
        print(f"\nerror: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted -- stopping the motor")
        return 130
    finally:
        try:
            link.close()      # sends the bare-newline stop on the way out
        finally:
            # The metric holds the dongle open from its first reading. It is
            # single-client, so a handle left open blocks the next run.
            metric.close()


if __name__ == "__main__":
    sys.exit(main())
