#!/usr/bin/env python3
"""
make_test_sweep.py
------------------
Emit a synthetic sweep log, so calibrate_axis.py can be exercised with no
Arduino, no IMU and no dish.

host/README.md notes that the simulator went away with actuator_system/, and
that the serial side can no longer be tested without hardware. This does not
bring that back, but it does cover the new path end to end: the console text
below is shaped exactly like what the sketch's 'w' command prints, banners
and telemetry included, so the parser and the fit get the same input they
will get on the day.

    python make_test_sweep.py > sweep.txt
    python calibrate_axis.py --from-log sweep.txt

Defaults model the real mount: a TVRO polar axis at Davis sits about 51.5 deg
off vertical, which is what makes gravity a good instrument here, swept across
the full ~90 deg of travel a jack gives on such a mount.

Sweep length is the thing that matters most. To see what a too-short sweep
does -- a confident-looking fit whose angle scale is quietly wrong -- try

    python make_test_sweep.py --deg-per-count 0.012 | python calibrate_axis.py --from-log /dev/stdin
"""

from __future__ import annotations

import argparse
import math
import random
import sys

from axis_fit import _cross, _dot, _scale, _sub, _unit


def rotate(v, axis, ang):
    c, s = math.cos(ang), math.sin(ang)
    cr = _cross(axis, v)
    d = _dot(axis, v)
    return tuple(v[i] * c + cr[i] * s + axis[i] * d * (1 - c) for i in range(3))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cone-deg", type=float, default=51.5,
                    help="angle between the rotation axis and gravity "
                         "(90 - latitude for a polar mount; default Davis)")
    ap.add_argument("--deg-per-count", type=float, default=0.056)
    ap.add_argument("--curvature", type=float, default=1e-6,
                    help="quadratic term, deg per count squared -- the linkage "
                         "is a triangle, so the curve is not quite a line")
    ap.add_argument("--from-count", type=int, default=-800)
    ap.add_argument("--to-count", type=int, default=800)
    ap.add_argument("--step", type=int, default=40)
    ap.add_argument("--noise-g", type=float, default=0.001,
                    help="per-axis gaussian noise, in g (1 mg is realistic)")
    ap.add_argument("--wind-at", type=int, default=None,
                    help="counts near which to inject a wind-spoiled sample")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args(argv)

    rng = random.Random(args.seed)

    # An arbitrary sensor mounting, so the fit has to actually find the axis
    # rather than being handed a tidy one.
    axis = _unit((0.31, -0.42, 0.85))
    seed_i = min(((abs(axis[i]), i) for i in range(3)))[1]
    seed_v = tuple(1.0 if i == seed_i else 0.0 for i in range(3))
    perp = _unit(_sub(seed_v, _scale(axis, _dot(seed_v, axis))))
    g0 = _unit(tuple(axis[i] * math.cos(math.radians(args.cone_deg))
                     + perp[i] * math.sin(math.radians(args.cone_deg))
                     for i in range(3)))

    out = sys.stdout
    out.write("=== Actuator System Bench Console ===\n")
    out.write("v1, driver = L298N (ENA D9, IN1 D6, IN2 D5)\n")
    out.write("IMU: found. 'm' reads it, 'w' sweeps for calibration.\n")
    out.write(f"  sweep {args.from_count} .. {args.to_count} "
              f"step {args.step}. Any key aborts.\n")
    out.write("SWEEP,counts,gx,gy,gz,temp_c,spread_g\n")

    for counts in range(args.from_count, args.to_count + 1, args.step):
        deg = args.deg_per_count * counts + args.curvature * counts * counts
        g = rotate(g0, axis, math.radians(deg))
        g = tuple(c + rng.gauss(0.0, args.noise_g) for c in g)
        g = _unit(g)

        # Temperature drifts across the sweep the way it does outdoors, which
        # is worth carrying through so the column is exercised.
        temp = 18.0 + 6.0 * (counts - args.from_count) / max(
            1, args.to_count - args.from_count)

        spread = abs(rng.gauss(0.0, 0.0008))
        if args.wind_at is not None and abs(counts - args.wind_at) < args.step:
            spread = 0.31          # a gust: this row must be thrown away

        out.write(f"SWEEP,{counts},{g[0]:.6f},{g[1]:.6f},{g[2]:.6f},"
                  f"{temp:.2f},{spread:.5f}\n")
        if counts % (args.step * 5) == 0:
            out.write(f"pos={counts} state=IDLE fault=NONE\n")

    out.write("  sweep done: 41 good, 0 rejected. "
              "Feed the SWEEP rows to host/axis_fit.py.\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
