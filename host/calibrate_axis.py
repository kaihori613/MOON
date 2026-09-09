#!/usr/bin/env python3
"""
calibrate_axis.py
-----------------
Turn a gravity sweep into the mount's rotation axis and its counts->degrees
curve, and write the result into config.json.

    # from a captured console log (no hardware needed)
    python calibrate_axis.py --from-log sweep.txt
    python calibrate_axis.py --from-log sweep.txt --save

    # drive the sketch and capture the sweep live
    python calibrate_axis.py --port /dev/ttyUSB0 --save

WHAT THIS REPLACES

Homing into the retract cam, as the way an absolute origin gets established.
The cam is still there and still cuts current at the ends -- it is the
end-of-travel protection and nothing removes it -- but it stops being the
thing that defines zero. After this calibration, 'm' plus a fit gives you an
absolute position from gravity alone, at boot, with the dish wherever the
wind left it.

WHAT IT DOES NOT REPLACE

The tie between this mount's frame and the sky. The fit knows where the dish
is pointing relative to itself, not where true north is. That anchor comes
from peaking on GOES-18 once and storing the trim -- which is a better
absolute reference than any compass, and is already what host/ does.

READ THE RESIDUALS. `circle fit RMS` is whether the data is trustworthy at
all. `linear worst error` is how far the linkage departs from a straight line,
which is the question host/README.md has so far been answering on faith.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from axis_fit import fit_axis, parse_sweep


def sweep_from_log(path: Path):
    with open(path, "r", errors="ignore") as fh:
        return parse_sweep(fh)


def sweep_from_port(port: str, baud: int, step: int | None, verbose: bool):
    """Drive the sketch's 'w' command and capture the rows it prints.

    Untested against hardware -- there is no Arduino on the machine this was
    written on. If it misbehaves, run 'w' in a serial monitor, save the output,
    and use --from-log, which is the same code path from the parse onward.
    """
    try:
        import serial  # noqa: F401
    except ImportError:
        sys.exit("pyserial is not installed: pip install -r requirements.txt")
    from link import ActuatorLink

    lines = []
    with ActuatorLink(port, baud=baud, verbose=verbose) as link:
        cmd = "w" if step is None else f"w {step}"
        print(f"  running '{cmd}' -- this is step/stop/read, so it is slow")
        for line in link.stream_command(cmd, done_marker="sweep done"):
            lines.append(line)
            if line.startswith("SWEEP,"):
                print(f"    {line}")
    return parse_sweep(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--from-log", metavar="FILE",
                     help="a saved console log containing SWEEP rows")
    src.add_argument("--port", help="serial port, to run the sweep now")

    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--step", type=int, default=None,
                    help="counts between samples (default: the sketch's)")
    ap.add_argument("--max-spread", type=float, default=0.05,
                    help="drop samples whose batch spread exceeded this, in g")
    ap.add_argument("--save", action="store_true",
                    help="write the fit into config.json")
    ap.add_argument("--config", default=None, help="path to config.json")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    if args.from_log:
        samples = sweep_from_log(Path(args.from_log))
    else:
        samples = sweep_from_port(args.port, args.baud, args.step, args.verbose)

    if not samples:
        sys.exit("no SWEEP rows found -- was the sweep actually run?")

    print(f"\n  read {len(samples)} sweep rows "
          f"({samples[0].counts} .. {samples[-1].counts} counts)")

    try:
        fit = fit_axis(samples, max_spread_g=args.max_spread)
    except ValueError as exc:
        sys.exit(f"\nfit failed: {exc}")

    print("\n--- axis fit " + "-" * 45)
    print(fit.report())
    print("-" * 58)

    # The two numbers that decide whether to believe any of this.
    if fit.circle_rms_deg > 0.5:
        print("\n  WARNING: the samples do not lie on a circle to better than "
              f"{fit.circle_rms_deg:.2f} deg.")
        print("  Something moved between readings, or the mount is flexing.")
        print("  Re-run on a calm day before trusting this.")
    if not fit.usable:
        print(f"\n  UNUSABLE: conditioning is {fit.conditioning:.1f}, and this "
              "method needs 25.")
        print("  Over a short arc a circle is nearly a straight line, and a")
        print("  straight line lies in many planes -- so the axis, and with it")
        print("  the angle SCALE, is not determined. The residuals will not warn")
        print("  you: the wrong circle still passes through every point.")
        print("  No error bar is quoted because below this the error stops being")
        print("  predictable -- measured cases have exceeded 2000%.")
        print("  Sweep MORE TRAVEL. Sagitta grows as the square of the arc, so")
        print("  doubling the sweep beats quartering the noise fourfold.")

    if fit.error_amplification > 3.0:
        print(f"\n  WARNING: tilt error is amplified {fit.error_amplification:.1f}x "
              "into yaw.")
        print("  The axis is closer to vertical than this method likes.")

    print(f"\n  For an 8 ft dish at 1694 MHz the beam is 5.08 deg wide, so")
    print(f"  1 deg of pointing error costs about 0.46 dB. This fit's circle")
    print(f"  residual is {fit.circle_rms_deg:.3f} deg.")

    print(f"\n  Linear model worst-case error over the swept arc: "
          f"{fit.linear_max_error_deg:.4f} deg")
    if fit.linear_max_error_deg < 0.25:
        print("  The straight-line linkage model is good enough. Keep 'linear'.")
    else:
        print("  The linkage is measurably curved over this arc -- prefer the")
        print("  measured table, or switch linkage.model to 'triangle'.")

    if args.save:
        path = Path(args.config) if args.config else Path(__file__).with_name("config.json")
        data = json.loads(path.read_text()) if path.exists() else {}
        data.setdefault("linkage", {})["gravity"] = {
            "_comment": "Written by calibrate_axis.py. axis and centre are in "
                        "the IMU's own frame; table is [counts, degrees] "
                        "measured, and is the authority. Re-run after moving "
                        "the sensor or remounting the dish.",
            "axis": list(fit.axis),
            "centre": list(fit.centre),
            "basis_u": list(fit.basis_u),
            "basis_v": list(fit.basis_v),
            "radius": fit.radius,
            "cone_half_angle_deg": fit.cone_half_angle_deg,
            "error_amplification": fit.error_amplification,
            "deg_per_count": fit.deg_per_count,
            "theta0_deg": fit.theta0_deg,
            "circle_rms_deg": fit.circle_rms_deg,
            "linear_max_error_deg": fit.linear_max_error_deg,
            "samples": fit.n,
            "table": [[c, round(t, 6)] for c, t in fit.table],
        }
        path.write_text(json.dumps(data, indent=2) + "\n")
        print(f"\n  saved to {path}")
    else:
        print("\n  (not saved -- add --save to write it into config.json)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
