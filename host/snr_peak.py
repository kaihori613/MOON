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
import json
import math
import random
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from iq_metric import GOES18_HRIT_HZ, IqMetricError, SdrSnrMetric
from peak import PointingReference, check_pointing, sweep

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

class SimDish:
    """
    Stands in for both the actuator and the SDR.

    The beam follows the law the search assumes, loss = 12*(err/bw)^2 dB.
    Every reading carries Gaussian noise, every move lands somewhere inside
    the PID's deadband, and below SIM_FLOOR_DB it refuses the way snr_m2m4
    refuses on pure noise.
    """

    SIM_FLOOR_DB = 1.0

    def __init__(self, peak_deg, peak_snr_db=12.0, beamwidth_deg=BEAMWIDTH_DEG,
                 noise_db=1.0, readings_per_s=10.0, seed=None):
        self.peak_deg = peak_deg
        self.peak_snr_db = peak_snr_db
        self.beamwidth_deg = beamwidth_deg
        self.noise_db = noise_db
        self.readings_per_s = readings_per_s
        self.pos = 0.0
        self.rng = random.Random(seed)

    def position(self) -> float:
        return self.pos

    def move_to(self, angle: float) -> float:
        if abs(angle) > SOFT_LIMIT_DEG:
            raise PeakRunError(f"(sim) {angle:+.2f} deg is past the soft limit")
        self.pos = angle + self.rng.uniform(-DEADBAND_DEG, DEADBAND_DEG)
        return self.pos

    def sample(self, dwell: float, settle: float = 0.0):
        err = self.pos - self.peak_deg
        true_db = self.peak_snr_db - 12.0 * (err / self.beamwidth_deg) ** 2
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


def run_sweep(mover, metric, lo, hi, step, settle, dwell, ref_dwell, metric_name):
    """
    Returns (PointingReference or None, PeakResult or None).

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

    try:
        result = sweep(mover.move_to, lambda: metric.sample(dwell), lo, hi, step,
                       settle=settle, dwell=dwell, lower_is_better=False,
                       on_point=on_point, return_on_failure=False)
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
    sdr.add_argument("--gain", default="auto")
    sdr.add_argument("--device-index", type=int, default=0)
    sdr.add_argument("--signal-bw", type=float, default=None,
                     help="occupied bandwidth, Hz; enables snr_psd (needs numpy)")
    sdr.add_argument("--estimator", choices=("auto", "psd", "m2m4"), default="auto")

    sim = ap.add_argument_group("simulation")
    sim.add_argument("--simulate", action="store_true",
                     help="fake dish and SDR: a sweep, then a check, nothing saved")
    sim.add_argument("--sim-peak", type=float, default=2.7,
                     help="where the fake beam peaks, deg")
    sim.add_argument("--sim-seed", type=int, default=None)
    args = ap.parse_args(argv)

    errors = (PeakRunError, IqMetricError)
    if args.plot and args.check:
        ap.error("--plot draws a sweep; use it with --sweep")

    # --- simulation --------------------------------------------------------
    if args.simulate:
        if args.check:
            ap.error("--simulate runs a sweep followed by a check; use --sweep")
        dish = SimDish(args.sim_peak, seed=args.sim_seed)
        lo, hi, step = args.sweep
        try:
            ref, result = run_sweep(dish, dish, lo, hi, step, settle=0.0,
                                    dwell=args.dwell, ref_dwell=args.ref_dwell,
                                    metric_name="simulated")
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
                      title=f"simulated sweep {lo:+g}..{hi:+g} deg, step {step:g}",
                      true_beam=(dish.peak_deg, dish.peak_snr_db,
                                 dish.beamwidth_deg))
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
                                args.dwell, args.ref_dwell, describe_metric(metric))
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
        link.close()      # sends the bare-newline stop on the way out


if __name__ == "__main__":
    sys.exit(main())
