"""
axis_fit.py
-----------
Turn a sweep of gravity vectors into the mount's rotation axis, and into a
map between reed counts and yaw degrees.

THE IDEA

The yaw axis is tilted. Rotate the dish about it and, in the sensor's own
frame, the gravity vector does not stay put -- it traces a CIRCLE. The plane
of that circle is perpendicular to the rotation axis, so:

    fit a plane to the measured gravity vectors
      -> its normal IS the rotation axis, in sensor coordinates
      -> the angle around that circle IS the yaw angle, 1:1

Nothing about the mount has to be measured with a tape. The sensor can be
bolted on at any angle; the fit discovers where it ended up. Sweep once and
you get the axis, the sensor's orientation, and the real counts->degrees
curve, all from the same data.

WHAT SETS THE PRECISION

The circle's radius is sin(gamma), where gamma is the angle between the
rotation axis and gravity. Tilt noise projects onto yaw as

    sigma_yaw = sigma_tilt / sin(gamma)

so a nearly-vertical axis (small gamma, small circle) is a bad instrument and
a strongly tilted one is a good one. `error_amplification` reports 1/sin(gamma)
so the number is visible rather than implied. A TVRO polar mount at Davis puts
gamma near 51 deg, giving about 1.3 -- almost free.

WHAT THE RESIDUALS TELL YOU

`circle_rms_deg` is how well the samples actually lie on a circle. It folds in
sensor noise, mount flex, and anything that moved between samples. If it is
large the fit is not trustworthy no matter how good the linear fit looks.

`linear_max_error_deg` is different and more interesting: it is how far the
counts->degrees relationship departs from a straight line. The linkage is a
triangle, so some curvature is expected and real. That number is the direct
answer to the question host/README.md has been asking on faith -- whether the
`linear` linkage model is good enough, or whether `triangle` is needed.

No numpy on purpose. The build machine has no Python at all, and host/
currently depends on pyserial and nothing else; a 3x3 eigenproblem does not
justify changing that.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Sequence


# ===========================================================================
#  Small vector helpers
# ===========================================================================

def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    n = _norm(a)
    if n == 0.0:
        raise ValueError("cannot normalise a zero vector")
    return _scale(a, 1.0 / n)


# ===========================================================================
#  Symmetric 3x3 eigenproblem, by cyclic Jacobi
# ===========================================================================
#  Enough for a covariance matrix and nothing more. Jacobi is chosen over a
#  closed form because it stays well behaved when two eigenvalues are close,
#  which is exactly the case for a circle traced over a short arc.

def _jacobi_eigen(m, sweeps: int = 24):
    a = [list(row) for row in m]
    v = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]

    for _ in range(sweeps):
        off = abs(a[0][1]) + abs(a[0][2]) + abs(a[1][2])
        if off < 1e-18:
            break
        for p, q in ((0, 1), (0, 2), (1, 2)):
            if abs(a[p][q]) < 1e-20:
                continue
            theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
            t = (1.0 if theta >= 0 else -1.0) / (abs(theta) + math.sqrt(theta * theta + 1.0))
            c = 1.0 / math.sqrt(t * t + 1.0)
            s = t * c
            for k in range(3):
                akp, akq = a[k][p], a[k][q]
                a[k][p] = c * akp - s * akq
                a[k][q] = s * akp + c * akq
            for k in range(3):
                apk, aqk = a[p][k], a[q][k]
                a[p][k] = c * apk - s * aqk
                a[q][k] = s * apk + c * aqk
            for k in range(3):
                vkp, vkq = v[k][p], v[k][q]
                v[k][p] = c * vkp - s * vkq
                v[k][q] = s * vkp + c * vkq

    vals = [a[i][i] for i in range(3)]
    vecs = [(v[0][i], v[1][i], v[2][i]) for i in range(3)]
    order = sorted(range(3), key=lambda i: vals[i])
    return [vals[i] for i in order], [vecs[i] for i in order]


# ===========================================================================
#  Samples
# ===========================================================================

@dataclass
class SweepSample:
    """One stationary reading: where the carriage was, and where gravity was."""
    counts: int
    g: tuple                # gravity unit vector, sensor frame
    temp_c: float = float("nan")
    spread_g: float = 0.0


def parse_sweep(lines: Iterable[str]) -> list:
    """Read the SWEEP rows the sketch prints, ignoring everything else.

    Row format, emitted by the 'w' command:
        SWEEP,<counts>,<gx>,<gy>,<gz>,<temp_c>,<spread_g>

    Anything that is not a well-formed SWEEP row is skipped rather than
    raising, because this has to survive being handed a whole console log
    with banners, telemetry and typos mixed in.
    """
    out = []
    for line in lines:
        parts = line.strip().split(",")
        if len(parts) < 5 or parts[0].strip().upper() != "SWEEP":
            continue
        try:
            counts = int(float(parts[1]))
            g = (float(parts[2]), float(parts[3]), float(parts[4]))
            temp = float(parts[5]) if len(parts) > 5 else float("nan")
            spread = float(parts[6]) if len(parts) > 6 else 0.0
        except ValueError:
            continue
        if _norm(g) <= 0.0:
            continue
        out.append(SweepSample(counts, _unit(g), temp, spread))
    return out


# ===========================================================================
#  The fit
# ===========================================================================

@dataclass
class AxisFit:
    axis: tuple                   # rotation axis, unit, sensor frame
    centre: tuple                 # circle centre, sensor frame
    basis_u: tuple                # in-plane reference direction (theta = 0)
    basis_v: tuple                # in-plane, completes a right-hand set
    radius: float                 # = sin(angle between axis and gravity)
    cone_half_angle_deg: float
    error_amplification: float    # 1 / radius: tilt error -> yaw error
    circle_rms_deg: float
    residual_rms: float           # same, in unit-vector length
    arc_deg: float                # total angle swept -- the conditioning driver
    sagitta: float                # how far the arc bows off its own chord
    conditioning: float           # sagitta / residual_rms; below ~25 is weak
    scale_uncertainty_pct: float  # bound on deg_per_count; NaN if unusable
    deg_per_count: float          # slope of the straight-line fit
    theta0_deg: float             # its intercept, at counts = 0
    linear_rms_deg: float
    linear_max_error_deg: float
    n: int
    table: list = field(default_factory=list)   # [(counts, theta_deg)], sorted

    @property
    def usable(self) -> bool:
        """Whether the angle SCALE can be trusted. A fit can be unusable and
        still look immaculate: see the conditioning note in fit_axis()."""
        return self.conditioning >= MIN_CONDITIONING

    # -- using the fit ----------------------------------------------------

    def angle_of(self, g: Sequence[float]) -> float:
        """Yaw angle, in degrees, of one gravity vector. The absolute read."""
        p = _sub(_unit(g), self.centre)
        return math.degrees(math.atan2(_dot(p, self.basis_v), _dot(p, self.basis_u)))

    def counts_to_deg(self, counts: float) -> float:
        """Interpolate the measured curve, extrapolating linearly outside it."""
        return _interp(self.table, counts, self.deg_per_count)

    def deg_to_counts(self, deg: float) -> float:
        flipped = [(t, c) for (c, t) in self.table]
        flipped.sort()
        if self.deg_per_count == 0.0:
            raise ValueError("degenerate fit: no degrees per count")
        return _interp(flipped, deg, 1.0 / self.deg_per_count)

    def linear_deg(self, counts: float) -> float:
        """The straight-line model, for comparison against the real curve."""
        return self.theta0_deg + self.deg_per_count * counts

    def report(self) -> str:
        lines = [
            f"  samples              {self.n}",
            f"  axis (sensor frame)  [{self.axis[0]:+.5f} {self.axis[1]:+.5f} {self.axis[2]:+.5f}]",
            f"  cone half-angle      {self.cone_half_angle_deg:7.3f} deg",
            f"  circle radius        {self.radius:7.5f}",
            f"  error amplification  {self.error_amplification:7.3f}x  "
            f"(tilt error -> yaw error)",
            f"  arc swept            {self.arc_deg:7.2f} deg",
            f"  circle fit RMS       {self.circle_rms_deg:7.4f} deg",
            f"  conditioning         {self.conditioning:7.1f}   "
            f"(sagitta / noise; >25 wanted)",
            (f"  scale uncertainty    <{self.scale_uncertainty_pct:6.2f} %"
             if self.usable else
             "  scale uncertainty       UNUSABLE -- sweep more travel"),
            f"  deg per count        {self.deg_per_count:+.6f}",
            f"  linear fit RMS       {self.linear_rms_deg:7.4f} deg",
            f"  linear worst error   {self.linear_max_error_deg:7.4f} deg",
        ]
        return "\n".join(lines)


def _interp(table, x, slope_outside):
    """Linear interpolation over a sorted [(x, y)] table, linear outside it."""
    if not table:
        raise ValueError("empty table")
    if len(table) == 1:
        return table[0][1] + (x - table[0][0]) * slope_outside
    if x <= table[0][0]:
        x0, y0 = table[0]
        x1, y1 = table[1]
        if x1 == x0:
            return y0
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    if x >= table[-1][0]:
        x0, y0 = table[-2]
        x1, y1 = table[-1]
        if x1 == x0:
            return y1
        return y1 + (x - x1) * (y1 - y0) / (x1 - x0)
    lo, hi = 0, len(table) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if table[mid][0] <= x:
            lo = mid
        else:
            hi = mid
    x0, y0 = table[lo]
    x1, y1 = table[hi]
    if x1 == x0:
        return y0
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0)


def _unwrap(angles):
    """Remove 2-pi jumps so the sequence is continuous along the sweep."""
    out = [angles[0]]
    for a in angles[1:]:
        prev = out[-1]
        while a - prev > math.pi:
            a -= 2.0 * math.pi
        while a - prev < -math.pi:
            a += 2.0 * math.pi
        out.append(a)
    return out


def _sagitta_from_chord(points):
    """How far the arc bows off the straight line joining its two ends.

    Purely geometric, no circle involved. For an arc of angle A on a circle of
    radius r this is r(1 - cos(A/2)) -- it grows as the square of the arc, which
    is exactly why a short sweep is so much worse than an merely-slightly
    shorter one.
    """
    a, b = points[0], points[-1]
    chord = _sub(b, a)
    length = _norm(chord)
    if length < 1e-12:
        # Ends coincide: either nothing moved, or the sweep went all the way
        # round. Fall back to spread about the centroid, which tells the two
        # apart on its own.
        n = float(len(points))
        c = tuple(sum(p[i] for p in points) / n for i in range(3))
        return max(_norm(_sub(p, c)) for p in points)
    d = _scale(chord, 1.0 / length)
    worst = 0.0
    for p in points:
        v = _sub(p, a)
        perp = _sub(v, _scale(d, _dot(v, d)))
        worst = max(worst, _norm(perp))
    return worst


def _noise_from_second_differences(points):
    """Per-axis noise, estimated without knowing the true path.

    A second difference p[i-1] - 2p[i] + p[i+1] cancels anything linear and
    leaves curvature plus noise. Along a gently curving arc sampled at many
    points the curvature term is orders of magnitude below the noise, so what
    is left is the noise. For independent per-axis noise of sigma, each
    component of the second difference has variance 6*sigma^2, and the vector
    has three components -- hence the 18.
    """
    if len(points) < 3:
        return 0.0
    total = 0.0
    for i in range(1, len(points) - 1):
        for k in range(3):
            d = points[i - 1][k] - 2.0 * points[i][k] + points[i + 1][k]
            total += d * d
    return math.sqrt(total / (len(points) - 2) / 18.0)


# A sweep that moves gravity less than this has not moved it at all: the
# axis is vertical, or the carriage never left the spot.
MIN_ARC_SPREAD = 1e-6

# Below this the circle is so small that tilt noise is amplified more than
# 1000x into yaw, which is not an instrument.
MIN_CIRCLE_RADIUS = 1e-3

# Below this the axis -- and with it the angle scale -- is not determined well
# enough to use, and the error stops being predictable rather than merely
# getting larger. See the note in fit_axis().
MIN_CONDITIONING = 25.0

# Factor relating conditioning to scale error, measured over the envelope
# described in fit_axis(). Past the 99th percentile everywhere measured;
# median behaviour is more than ten times better than this implies.
SCALE_BOUND_FACTOR = 200.0


def fit_axis(samples: Sequence[SweepSample], max_spread_g: float = 0.05) -> AxisFit:
    """Fit the rotation axis and the counts->degrees curve to one sweep.

    Samples whose batch spread exceeded `max_spread_g` are dropped: the sketch
    already flags them, and a reading taken while the dish was moving in the
    wind is not a measurement of anything.
    """
    pts = [s for s in samples if s.spread_g <= max_spread_g]
    if len(pts) < 5:
        raise ValueError(
            f"need at least 5 usable samples to fit a circle, got {len(pts)}"
        )
    pts = sorted(pts, key=lambda s: s.counts)

    n = float(len(pts))
    cx = sum(s.g[0] for s in pts) / n
    cy = sum(s.g[1] for s in pts) / n
    cz = sum(s.g[2] for s in pts) / n
    centroid = (cx, cy, cz)

    # Covariance of the points about their centroid. The eigenvector of the
    # SMALLEST eigenvalue is the direction of least spread -- normal to the
    # best-fit plane, which is the rotation axis.
    cov = [[0.0] * 3 for _ in range(3)]
    for s in pts:
        d = _sub(s.g, centroid)
        for i in range(3):
            for j in range(3):
                cov[i][j] += d[i] * d[j]
    for i in range(3):
        for j in range(3):
            cov[i][j] /= n

    vals, vecs = _jacobi_eigen(cov)
    axis = _unit(vecs[0])

    # Before trusting the eigenvectors at all: did gravity move? When the
    # points are all in one place the covariance is zero and the solver
    # returns an arbitrary frame that will happily fit a confident circle to
    # nothing. The largest eigenvalue is the spread along the widest
    # direction, so it is the honest test.
    if math.sqrt(max(0.0, vals[2])) < MIN_ARC_SPREAD:
        raise ValueError(
            "gravity did not move across the sweep. Either the carriage never "
            "moved, or the rotation axis is parallel to gravity -- a vertical "
            "axis tilts nothing, so gravity cannot measure it at all and a "
            "magnetometer is the only option left"
        )

    # The circle's centre is the point on the axis that the samples ring.
    # Gravity is a unit vector, so every sample lies on the unit sphere and
    # the circle's plane cuts it at height dot(axis, g).
    height = sum(_dot(s.g, axis) for s in pts) / n
    centre = _scale(axis, height)

    # Radius measured from the data rather than inferred from `height`. The
    # two agree for a real circle, but only the measured one falls to zero
    # when there is no circle, which is the case worth catching.
    radius = sum(_norm(_sub(s.g, centre)) for s in pts) / n
    if radius < MIN_CIRCLE_RADIUS:
        raise ValueError(
            "the samples do not trace a circle of usable size: the rotation "
            "axis is nearly parallel to gravity, so yaw error would be "
            f"amplified by {1.0 / max(radius, 1e-12):.0f}x"
        )

    # An orthonormal pair spanning the circle's plane. The seed is whichever
    # world axis is least aligned with the rotation axis, so the cross product
    # never collapses.
    seed = min(((abs(axis[i]), i) for i in range(3)))[1]
    seed_vec = tuple(1.0 if i == seed else 0.0 for i in range(3))
    u = _unit(_sub(seed_vec, _scale(axis, _dot(seed_vec, axis))))
    v = _cross(axis, u)

    raw = []
    resid = []
    for s in pts:
        p = _sub(s.g, centre)
        raw.append(math.atan2(_dot(p, v), _dot(p, u)))
        # Distance from the fitted circle, expressed as an angle at the
        # circle's radius so it is directly comparable to a pointing error.
        resid.append(_norm(p) - radius)

    theta = _unwrap(raw)

    # Sign convention: extending (increasing counts) must increase the angle,
    # otherwise the axis direction from the eigensolver is arbitrary and the
    # readout would run backwards half the time.
    if theta[-1] < theta[0]:
        axis = _scale(axis, -1.0)
        v = _scale(v, -1.0)
        theta = [-t for t in theta]
        centre = _scale(axis, -height)
        height = -height

    theta_deg = [math.degrees(t) for t in theta]
    counts = [float(s.counts) for s in pts]

    # Ordinary least squares of degrees against counts.
    mean_c = sum(counts) / n
    mean_t = sum(theta_deg) / n
    sxx = sum((c - mean_c) ** 2 for c in counts)
    if sxx <= 0.0:
        raise ValueError("every sample is at the same count -- nothing to fit")
    sxy = sum((c - mean_c) * (t - mean_t) for c, t in zip(counts, theta_deg))
    slope = sxy / sxx
    intercept = mean_t - slope * mean_c

    errs = [t - (intercept + slope * c) for c, t in zip(counts, theta_deg)]
    lin_rms = math.sqrt(sum(e * e for e in errs) / n)
    lin_max = max(abs(e) for e in errs)

    residual_rms = math.sqrt(sum(r * r for r in resid) / n)
    circle_rms_deg = (math.degrees(residual_rms / radius)
                      if radius > 0.0 else float("inf"))

    # CONDITIONING. This is the number that decides whether the fit means
    # anything, and it is not obvious from the residuals.
    #
    # Fitting a circle to a SHORT arc is famously ill-posed: over a few
    # degrees an arc is nearly a straight line, and a straight line lies in
    # many planes. The plane normal is the rotation axis, so a short sweep
    # gives a badly determined axis -- and since the yaw angle is arc length
    # over radius, a badly determined radius mis-scales every angle derived
    # from it. The LINEAR residual stays small throughout -- the wrong circle
    # still passes through the points -- so `residual_rms` will not catch it.
    # (`circle_rms_deg` usually does inflate, because it divides by a radius
    # that has collapsed, but that is a side effect rather than a test.)
    #
    # What separates a real arc from a line is the sagitta -- how far the arc
    # bows away from its own chord. Compare that against the measurement noise
    # and you have the signal-to-noise of the whole fit. Both are taken from
    # the raw points, so this stays honest when the fit is not.
    #
    # Calibrated over ~2700 synthetic sweeps spanning 10-120 degrees of arc,
    # 0.1-2 mg of noise, 21-81 samples and cone angles from 30 to 70 degrees.
    # Above a conditioning of 25, (scale error in percent) x conditioning had
    # a median near 15-34 depending on the corner, a 99th percentile of 103 to
    # 148, and a worst single case of 262.
    #
    # The quoted figure uses 200: comfortably past the 99th percentile of
    # every corner measured, but NOT a guarantee -- the worst trial seen would
    # have needed 262. Read it as a strong bound with a long thin tail, and
    # note the typical error is more than an order of magnitude smaller than
    # the number printed.
    #
    # BELOW 25 no number is quoted, deliberately. The relationship is not just
    # worse there, it diverges -- at a conditioning of 3.4 the measured scale
    # error was over 2000%, where extrapolating the formula would have
    # promised 7%. An error bar that is optimistic precisely where the method
    # collapses is worse than no error bar, so the field goes to NaN and the
    # caller is told the fit is unusable instead.
    #
    # The fix for a low number is mechanical -- sweep MORE TRAVEL. Sagitta
    # grows with the square of the arc while noise does not, so doubling the
    # swept arc is worth about four times as much as quartering the noise.
    # Both terms are measured from the raw points, NOT from the fit -- a
    # conditioning number derived from the fit it is meant to judge would be
    # worthless exactly when it matters.
    sagitta = _sagitta_from_chord([s.g for s in pts])
    noise_est = _noise_from_second_differences([s.g for s in pts])
    conditioning = (sagitta / noise_est) if noise_est > 0.0 else float("inf")
    scale_uncertainty_pct = (SCALE_BOUND_FACTOR / conditioning
                             if conditioning >= MIN_CONDITIONING
                             else float("nan"))
    arc_rad = abs(theta[-1] - theta[0])

    return AxisFit(
        axis=axis,
        centre=centre,
        basis_u=u,
        basis_v=v,
        radius=radius,
        cone_half_angle_deg=math.degrees(math.acos(max(-1.0, min(1.0, abs(height))))),
        error_amplification=1.0 / radius,
        circle_rms_deg=circle_rms_deg,
        residual_rms=residual_rms,
        arc_deg=math.degrees(arc_rad),
        sagitta=sagitta,
        conditioning=conditioning,
        scale_uncertainty_pct=scale_uncertainty_pct,
        deg_per_count=slope,
        theta0_deg=intercept,
        linear_rms_deg=lin_rms,
        linear_max_error_deg=lin_max,
        n=len(pts),
        table=sorted(zip((int(c) for c in counts), theta_deg)),
    )
