"""
peak.py
-------
The calibration run: find the angle with the best signal, and stop there.

WHAT THIS IS, AND WHAT IT IS NOT

It is a CALIBRATION, not a startup step. GOES-18 is geostationary, so from a
fixed site the look angle never changes and there is nothing to track. Run
this once, store what it finds, and afterwards point from geometry plus that
stored offset with the receiver contributing nothing.

Running it on every power-on would be actively harmful. A sweep is, by
definition, deliberately mispointing a working dish in order to rediscover a
number you already had -- minutes of wear, for no new information, with a real
chance of walking off a good aim point if the signal happens to be poor that
day for reasons that have nothing to do with pointing.

So: SatDump running ARMS this. A person or a degraded-signal condition
TRIGGERS it.

WHY STOP-MEASURE-MOVE RATHER THAN A CONTINUOUS SWEEP

An SNR estimate needs seconds of averaging. A reading taken while the dish is
moving therefore belongs to where the dish WAS, not where it is, and that lag
biases the located peak in the direction of travel -- you find it consistently
late. Stopping to measure removes the bias outright, which is cheaper than
sweeping both ways to cancel it.

FAILURE MUST BE NON-DESTRUCTIVE

If the run cannot find a peak -- no lock anywhere, rain, a receiver problem --
it returns the dish to where it started. A failed calibration has to leave the
system no worse than it found it, because the most likely reason for failing
is that something other than pointing is wrong, and in that case the old aim
was the good one.
"""

from __future__ import annotations

import time

__all__ = ["sweep", "parabolic_peak", "parabolic_fit", "PeakResult", "PeakError"]


class PeakError(RuntimeError):
    pass


class PeakResult:
    def __init__(self, best_angle, best_score, samples, interpolated, reason=""):
        self.best_angle = best_angle
        self.best_score = best_score
        self.samples = samples          # [(angle, score, stdev, n), ...]
        self.interpolated = interpolated
        self.reason = reason

    @property
    def ok(self):
        return self.best_angle is not None

    def __repr__(self):
        if not self.ok:
            return f"<PeakResult FAILED {self.reason!r} after {len(self.samples)} points>"
        return (f"<PeakResult {self.best_angle:+.3f} deg score={self.best_score:.3f} "
                f"from {len(self.samples)} points"
                f"{' interpolated' if self.interpolated else ''}>")


def parabolic_peak(x1, y1, x2, y2, x3, y3):
    """
    Vertex of the parabola through three equally spaced points, y2 the largest.

    Buys sub-step resolution for free: the sweep can use a coarse step, which
    is what costs wall-clock time, and still land between the samples. Returns
    None when the three points do not describe a maximum -- a flat top or a
    numerical edge -- in which case the caller should keep the raw best point
    rather than invent one.
    """
    denom = y1 - 2.0 * y2 + y3
    if denom >= 0.0:                     # not a maximum
        return None
    h = x2 - x1
    if h <= 0.0 or abs(x3 - x2 - h) > 1e-9 * max(1.0, h):
        return None                      # not equally spaced
    offset = 0.5 * h * (y1 - y3) / denom
    if abs(offset) > h:                  # vertex outside the bracket
        return None
    return x2 + offset


def parabolic_fit(points):
    """
    Least-squares parabola through many points; returns the vertex x, or None.

    THIS IS WHY A THREE-POINT FIT IS NOT ENOUGH, and it is a property of the
    dish rather than of the code. Pointing loss goes as 12*(theta/beamwidth)^2
    dB, so on a 12 deg beam being one degree off costs 0.083 dB -- far below
    the noise on any realistic SNR estimate. The three samples nearest the peak
    are therefore all within noise of each other, and a vertex fitted through
    them is fitting the noise.

    The information about where the peak is lives on the FLANKS, where the
    curve actually has gradient. Fitting many points uses that, and averages
    the noise down by roughly the square root of their number.

    points: [(x, y), ...], at least 3, not necessarily equally spaced.
    """
    n = len(points)
    if n < 3:
        return None

    # Normal equations for y = a x^2 + b x + c. Centre x first: it costs one
    # subtraction and keeps the 3x3 well conditioned when the angles are all
    # near each other, which is exactly the case here.
    x0 = sum(x for x, _ in points) / n
    s1 = s2 = s3 = s4 = t0 = t1 = t2 = 0.0
    for x, y in points:
        xc = x - x0
        x2 = xc * xc
        s1 += xc
        s2 += x2
        s3 += x2 * xc
        s4 += x2 * x2
        t0 += y
        t1 += xc * y
        t2 += x2 * y

    m = [[s4, s3, s2, t2],
         [s3, s2, s1, t1],
         [s2, s1, float(n), t0]]

    # Gaussian elimination with partial pivoting.
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            return None
        m[col], m[piv] = m[piv], m[col]
        for r in range(col + 1, 3):
            f = m[r][col] / m[col][col]
            for c in range(col, 4):
                m[r][c] -= f * m[col][c]
    coef = [0.0, 0.0, 0.0]
    for r in (2, 1, 0):
        acc = m[r][3] - sum(m[r][c] * coef[c] for c in range(r + 1, 3))
        coef[r] = acc / m[r][r]

    a, b = coef[0], coef[1]
    if a >= 0.0:                 # not a maximum
        return None
    vertex = -b / (2.0 * a) + x0
    lo = min(x for x, _ in points)
    hi = max(x for x, _ in points)
    if not (lo <= vertex <= hi):  # extrapolating past the data is not a result
        return None
    return vertex


def sweep(move_to, read_metric, lo, hi, step,
          settle=1.0, dwell=3.0, lower_is_better=False,
          on_point=None, return_on_failure=True, fit_points=None):
    """
    Stop-measure-move across [lo, hi], then park on the best angle found.

    move_to(angle)      -> drive there and block until it has settled
    read_metric()       -> (mean, stdev, n), or raise to mean "no reading"

    lower_is_better flips the comparison for metrics like Viterbi corrected
    errors, which fall as the signal improves. Internally everything becomes a
    SCORE where higher is better, so the interpolation below stays a maximum
    in both cases -- the alternative is a sign error waiting to happen in the
    one place it would be hardest to notice.

    fit_points defaults to None, meaning fit ALL the points that produced a
    reading. Do not lower it without measuring. Against a simulated 12 deg beam
    with 0.4 dB of noise, restricting the fit to the best few points is worse
    than useless -- five points gave a larger 90th-percentile error than three
    did -- because the samples nearest the peak are the ones carrying least
    information about where it is. Accuracy improves monotonically with the
    number of points fitted and saturates around nine; using all of them was no
    worse than the best fixed count at either beamwidth tested.

    That result holds while the sweep stays inside the main lobe, which it does
    over a +/-9 deg range on a beam this wide. A sweep wide enough to reach
    sidelobes would need the fit narrowed, because a parabola stops describing
    the pattern out there.
    """
    if hi <= lo:
        raise PeakError(f"empty range: lo={lo} hi={hi}")
    if step <= 0:
        raise PeakError("step must be positive")

    start_angle = None
    samples = []

    angle = lo
    while angle <= hi + 1e-9:
        move_to(angle)
        if start_angle is None:
            start_angle = angle
        if settle > 0:
            time.sleep(settle)
        try:
            mean, stdev, n = read_metric()
            score = -mean if lower_is_better else mean
            samples.append((angle, score, stdev, n))
        except Exception as exc:
            # No reading here is information, not an error: it means no lock at
            # this angle, which is exactly what the edges of a sweep look like.
            samples.append((angle, None, None, 0))
            if on_point:
                on_point(angle, None, str(exc))
            angle += step
            continue

        if on_point:
            on_point(angle, score, None)
        angle += step

    measured = [(a, s) for (a, s, _sd, _n) in samples if s is not None]
    if not measured:
        if return_on_failure and start_angle is not None:
            move_to(start_angle)
        return PeakResult(None, None, samples, False,
                          "no usable reading anywhere in the range")

    best_i = max(range(len(measured)), key=lambda i: measured[i][1])
    best_angle, best_score = measured[best_i]

    # A peak at the very edge means the real one is probably outside the range
    # that was swept, so the answer is "widen the sweep", not this number.
    if best_i == 0 or best_i == len(measured) - 1:
        if return_on_failure and start_angle is not None:
            move_to(start_angle)
        return PeakResult(None, None, samples, False,
                          f"best reading is at the edge of the sweep "
                          f"({best_angle:+.2f} deg) -- widen the range")

    # Fit every reading rather than just the three around the maximum -- see
    # parabolic_fit for why three is not enough on a wide beam.
    interpolated = False
    if fit_points is None:
        chosen = list(measured)
    else:
        ranked = sorted(measured, key=lambda p: p[1], reverse=True)
        chosen = sorted(ranked[:max(3, fit_points)], key=lambda p: p[0])
    refined = parabolic_fit(chosen)
    if refined is None:
        # Fall back to the classic three-point vertex, then to the raw best.
        refined = parabolic_peak(measured[best_i - 1][0], measured[best_i - 1][1],
                                 measured[best_i][0],     measured[best_i][1],
                                 measured[best_i + 1][0], measured[best_i + 1][1])
    if refined is not None:
        best_angle = refined
        interpolated = True

    move_to(best_angle)
    return PeakResult(best_angle, best_score, samples, interpolated)


# ---------------------------------------------------------------------------
#  Pointing health -- the reference switch, done in software
# ---------------------------------------------------------------------------
#  An earlier revision planned a mid-travel reference switch, on the argument
#  that the encoder's zero could creep with no symptom. The first half of that
#  is true and the second half is not: the satellite IS a reference, and a
#  continuously available, extremely sensitive one. If the zero drifts, the
#  metric at the stored angle falls.
#
#  Better still, the drift needs no separate repair. A calibration run does
#  not care what "zero" means -- it finds the encoder reading with the best
#  signal and stores that -- so a shifted zero is absorbed into the next run
#  automatically.
#
#  So what a health check has to do is narrower than a switch: notice that
#  the signal at the aimed angle is no longer what calibration recorded, and
#  say so. It catches a shifted mount, a slipped magnet, a wet LNA and a
#  failing feed all at once, which is more useful than knowing specifically
#  that the magnet moved.

class PointingReference:
    """
    What a calibration run established, and what the health check compares
    against. Persist it next to the trim; the caller owns storage.
    """

    def __init__(self, angle, metric_mean, metric_stdev, metric_name="",
                 lower_is_better=False, when=None, beamwidth_deg=None):
        self.angle = angle
        self.metric_mean = metric_mean
        self.metric_stdev = metric_stdev
        self.metric_name = metric_name
        self.lower_is_better = lower_is_better
        self.when = when
        self.beamwidth_deg = beamwidth_deg

    @property
    def score(self):
        return -self.metric_mean if self.lower_is_better else self.metric_mean

    def to_dict(self):
        return {"angle_deg": self.angle, "metric_mean": self.metric_mean,
                "metric_stdev": self.metric_stdev, "metric_name": self.metric_name,
                "lower_is_better": self.lower_is_better, "when": self.when,
                "beamwidth_deg": self.beamwidth_deg}

    @classmethod
    def from_dict(cls, d):
        return cls(d["angle_deg"], d["metric_mean"], d.get("metric_stdev", 0.0),
                   d.get("metric_name", ""), d.get("lower_is_better", False),
                   d.get("when"), d.get("beamwidth_deg"))


class HealthVerdict:
    OK        = "ok"
    IMPROVED  = "improved"
    DEGRADED  = "degraded"
    FAILED    = "failed"
    NO_SIGNAL = "no-signal"

    def __init__(self, status, drop=None, threshold=None, now=None,
                 implied_error_deg=None, message=""):
        self.status = status
        self.drop = drop                      # score units, positive = worse
        self.threshold = threshold            # what it had to beat to count
        self.now = now
        self.implied_error_deg = implied_error_deg
        self.message = message

    @property
    def needs_repeak(self):
        return self.status in (HealthVerdict.DEGRADED, HealthVerdict.FAILED)

    def __repr__(self):
        return f"<HealthVerdict {self.status}: {self.message}>"


def check_pointing(reference, read_metric, warn=2.0, fail=4.0, sigmas=3.0):
    """
    Compare the metric now against what calibration recorded.

    warn and fail are in the metric's own units (dB for an SNR). The actual
    threshold is the LARGER of that and `sigmas` times the combined noise of
    the two readings, for the same reason the reed had a debounce and the
    moment estimator has a significance gate: a system that reacts to its own
    measurement noise will re-peak a perfectly good dish on a windy afternoon
    and store whatever it happens to find.

    Never moves anything. It reports; the caller decides.
    """
    try:
        mean, stdev, _n = read_metric()
    except Exception as exc:
        return HealthVerdict(
            HealthVerdict.NO_SIGNAL, message=f"no reading at all ({exc})")

    now_score = -mean if reference.lower_is_better else mean
    drop = reference.score - now_score          # positive means worse

    noise = ((reference.metric_stdev ** 2 + stdev ** 2) ** 0.5) * sigmas
    warn_at = max(warn, noise)
    fail_at = max(fail, noise)

    # A drop only means a pointing error if pointing is what changed, but the
    # implied angle is still the useful sanity check: it says whether
    # re-peaking could plausibly recover this, or whether the number is far too
    # large for any pointing error the mount can even reach.
    implied = None
    if reference.beamwidth_deg and drop > 0:
        implied = reference.beamwidth_deg * (drop / 12.0) ** 0.5

    if drop >= fail_at:
        msg = f"down {drop:.2f} against a {fail_at:.2f} threshold"
        if implied is not None:
            msg += f"; a pointing error of about {implied:.1f} deg would explain it"
        return HealthVerdict(HealthVerdict.FAILED, drop, fail_at, now_score,
                             implied, msg)

    if drop >= warn_at:
        msg = f"down {drop:.2f} against a {warn_at:.2f} threshold"
        if implied is not None:
            msg += f"; about {implied:.1f} deg of pointing error would explain it"
        return HealthVerdict(HealthVerdict.DEGRADED, drop, warn_at, now_score,
                             implied, msg)

    if -drop >= warn_at:
        # Worth saying rather than swallowing: it usually means the reference
        # was taken in poor conditions and is not the standard to hold to.
        return HealthVerdict(
            HealthVerdict.IMPROVED, drop, warn_at, now_score, None,
            f"up {-drop:.2f} on the reference -- calibration may have been "
            f"taken in worse conditions than these")

    return HealthVerdict(HealthVerdict.OK, drop, warn_at, now_score, None,
                         f"within {warn_at:.2f} of the reference")
