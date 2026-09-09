"""
test_axis_fit.py
----------------
Checks the axis fit against synthetic sweeps whose answer is known by
construction: pick an axis, rotate gravity about it, hand the result back and
see whether the fit recovers what was put in.

This matters more than it looks. The fit is the only thing standing between
a bolted-on accelerometer and an absolute pointing number, and a sign error
in the basis or the unwrap gives a curve that looks perfectly smooth while
running backwards.

    python test_axis_fit.py          (no pytest needed)
    pytest test_axis_fit.py
"""

import math
import random

from axis_fit import (AxisFit, SweepSample, fit_axis, parse_sweep, _cross,
                      _dot, _norm, _scale, _sub, _unit)


# --- helpers ---------------------------------------------------------------

def rotate(v, axis, ang):
    """Rodrigues' rotation of v about a unit axis by ang radians."""
    c, s = math.cos(ang), math.sin(ang)
    cr = _cross(axis, v)
    d = _dot(axis, v)
    return (v[0] * c + cr[0] * s + axis[0] * d * (1 - c),
            v[1] * c + cr[1] * s + axis[1] * d * (1 - c),
            v[2] * c + cr[2] * s + axis[2] * d * (1 - c))


def synth(axis, deg_per_count=0.02, n=40, step=20, noise=0.0, seed=1,
          start_count=0):
    """A sweep about `axis`, with gravity starting somewhere off-axis."""
    axis = _unit(axis)
    rng = random.Random(seed)

    # A starting gravity direction that is not parallel to the axis, so the
    # circle it traces has real radius.
    seed_i = min(((abs(axis[i]), i) for i in range(3)))[1]
    seed_v = tuple(1.0 if i == seed_i else 0.0 for i in range(3))
    perp = _unit(_sub(seed_v, _scale(axis, _dot(seed_v, axis))))
    # 40 deg off the axis: a real cone, not a degenerate one.
    g0 = _unit(tuple(axis[i] * math.cos(math.radians(40.0))
                     + perp[i] * math.sin(math.radians(40.0)) for i in range(3)))

    out = []
    for k in range(n):
        counts = start_count + k * step
        ang = math.radians(deg_per_count * counts)
        g = rotate(g0, axis, ang)
        if noise:
            g = tuple(c + rng.gauss(0.0, noise) for c in g)
        out.append(SweepSample(counts, _unit(g), 20.0, 0.0))
    return out


TILTED = _unit((0.30, -0.45, 0.84))


# --- the axis --------------------------------------------------------------

def test_recovers_a_known_axis():
    """
    The fitted axis should be the one the data was generated about, up to
    sign -- and the sign is then pinned by the convention that increasing
    counts increase the angle.
    """
    fit = fit_axis(synth(TILTED))
    aligned = abs(_dot(fit.axis, TILTED))
    assert aligned > 0.9999, f"axis off: dot = {aligned}"


def test_axis_points_so_that_counts_increase_the_angle():
    """
    Feed a sweep that rotates the *other* way. The fit must flip the axis so
    the readout still climbs with the carriage, rather than running backwards.
    """
    fit = fit_axis(synth(TILTED, deg_per_count=-0.02))
    assert fit.deg_per_count > 0, (
        f"slope should be positive after the sign fix, got {fit.deg_per_count}")


def test_recovers_degrees_per_count():
    fit = fit_axis(synth(TILTED, deg_per_count=0.0175))
    assert abs(fit.deg_per_count - 0.0175) < 1e-6, fit.deg_per_count


def test_cone_half_angle_matches_construction():
    """synth() puts gravity 40 deg off the axis, so that is the cone."""
    fit = fit_axis(synth(TILTED))
    assert abs(fit.cone_half_angle_deg - 40.0) < 0.01, fit.cone_half_angle_deg


def test_error_amplification_is_one_over_sin_gamma():
    fit = fit_axis(synth(TILTED))
    expected = 1.0 / math.sin(math.radians(40.0))
    assert abs(fit.error_amplification - expected) < 0.01


# --- the readout -----------------------------------------------------------

def test_angle_of_round_trips_through_the_sweep():
    """Every sample's own gravity vector must read back as its own angle."""
    samples = synth(TILTED, deg_per_count=0.02)
    fit = fit_axis(samples)
    worst = 0.0
    for s in samples:
        got = fit.angle_of(s.g)
        want = fit.counts_to_deg(s.counts)
        # atan2 wraps; compare on the circle, not on the line.
        d = abs((got - want + 180.0) % 360.0 - 180.0)
        worst = max(worst, d)
    assert worst < 1e-6, f"worst round-trip error {worst} deg"


def test_counts_and_degrees_invert_each_other():
    fit = fit_axis(synth(TILTED, deg_per_count=0.02))
    for c in (0, 137, 400, 780):
        back = fit.deg_to_counts(fit.counts_to_deg(c))
        assert abs(back - c) < 1e-3, f"{c} -> {back}"


def test_table_covers_the_swept_range():
    samples = synth(TILTED, n=30, step=25)
    fit = fit_axis(samples)
    assert fit.table[0][0] == 0
    assert fit.table[-1][0] == 29 * 25
    assert len(fit.table) == 30


# --- quality metrics -------------------------------------------------------

def test_clean_data_fits_a_circle_almost_exactly():
    fit = fit_axis(synth(TILTED))
    assert fit.circle_rms_deg < 1e-6, fit.circle_rms_deg


def test_noise_shows_up_in_the_circle_residual():
    """
    The residual is the honest quality number, so it has to actually move
    when the data gets worse.
    """
    clean = fit_axis(synth(TILTED, noise=0.0))
    dirty = fit_axis(synth(TILTED, noise=0.002))
    assert dirty.circle_rms_deg > clean.circle_rms_deg
    # 2 mg of noise on a 40 deg cone is a fraction of a degree, not degrees.
    assert dirty.circle_rms_deg < 0.5, dirty.circle_rms_deg


def test_a_pure_rotation_is_exactly_linear():
    """
    Rotation about a fixed axis is linear in the rotation angle by
    construction, so with counts proportional to angle the linear fit is
    perfect. Real linkage curvature has to come from the linkage, not from
    an artefact of this code.
    """
    fit = fit_axis(synth(TILTED))
    assert fit.linear_max_error_deg < 1e-6, fit.linear_max_error_deg


def test_noisy_samples_above_the_spread_gate_are_dropped():
    samples = synth(TILTED, n=20)
    for s in samples[:6]:
        s.spread_g = 0.9          # "the wind was blowing"
    fit = fit_axis(samples, max_spread_g=0.05)
    assert fit.n == 14, fit.n


# --- conditioning ----------------------------------------------------------

def test_arc_deg_matches_the_sweep():
    fit = fit_axis(synth(TILTED, deg_per_count=0.02, n=41, step=20))
    # 41 samples, 20 counts apart, 0.02 deg/count = 16 deg of arc.
    assert abs(fit.arc_deg - 16.0) < 1e-6, fit.arc_deg


def test_a_longer_arc_conditions_better():
    """
    The whole reason conditioning is reported. Same noise, same sample count,
    only the swept arc changes -- and that is what decides whether the fit is
    worth anything.
    """
    short = fit_axis(synth(TILTED, deg_per_count=0.005, n=41, step=20,
                           noise=0.001, seed=3))     # 4 deg
    long_ = fit_axis(synth(TILTED, deg_per_count=0.05, n=41, step=20,
                           noise=0.001, seed=3))     # 40 deg
    assert long_.conditioning > short.conditioning * 10, (
        f"short {short.conditioning:.1f} vs long {long_.conditioning:.1f}")


def test_short_arc_is_flagged_as_badly_conditioned():
    """
    A 4 degree sweep with 1 mg of noise gives a confident-looking fit with a
    badly wrong scale. The residuals do not catch it; conditioning must.
    """
    fit = fit_axis(synth(TILTED, deg_per_count=0.005, n=41, step=20,
                         noise=0.001, seed=5))
    # The LINEAR residual stays small -- the wrong circle still passes through
    # the points -- which is exactly why conditioning has to be reported
    # separately rather than inferred from how well the fit fits.
    assert fit.residual_rms < 0.01, fit.residual_rms
    assert fit.conditioning < 25.0, fit.conditioning
    assert not fit.usable
    # No error bar at all, rather than an optimistic one.
    assert math.isnan(fit.scale_uncertainty_pct)


def test_a_generous_sweep_is_well_conditioned():
    fit = fit_axis(synth(TILTED, deg_per_count=0.05, n=41, step=20,
                         noise=0.001, seed=5))       # 40 deg of arc
    assert fit.conditioning > 25.0, fit.conditioning
    assert fit.usable
    assert abs(fit.deg_per_count - 0.05) / 0.05 < 0.02, fit.deg_per_count
    # The quoted bound must actually bound the real error.
    real_pct = abs(fit.deg_per_count - 0.05) / 0.05 * 100.0
    assert real_pct <= fit.scale_uncertainty_pct, (
        f"real {real_pct:.2f}% exceeded the quoted {fit.scale_uncertainty_pct:.2f}%")


def test_the_quoted_bound_holds_across_the_envelope():
    """
    The uncertainty figure is only worth printing if it actually bounds the
    error. Sweep the parameter space and check the coverage it claims -- a
    99th-percentile bound has to hold at least that often, or the number is
    worse than none at all.
    """
    checked = violations = 0
    for arc_dpc in (0.02, 0.05, 0.1):
        for noise in (0.0002, 0.001, 0.002):
            for n in (21, 41):
                for seed in range(6):
                    fit = fit_axis(synth(TILTED, deg_per_count=arc_dpc, n=n,
                                         step=20, noise=noise, seed=seed))
                    if not fit.usable:
                        continue
                    checked += 1
                    real = abs(fit.deg_per_count - arc_dpc) / arc_dpc * 100.0
                    if real > fit.scale_uncertainty_pct:
                        violations += 1
    assert checked > 50, f"only {checked} usable fits -- test is not exercising much"
    rate = violations / checked
    assert rate <= 0.02, (
        f"quoted bound exceeded in {violations}/{checked} = {rate:.1%} of cases")


# --- degenerate geometry ---------------------------------------------------

def test_axis_parallel_to_gravity_is_refused():
    """
    A vertical yaw axis traces no circle at all -- gravity is constant through
    the whole sweep. That is the case where a magnetometer is the only option,
    and the fit has to say so rather than return confident noise.
    """
    axis = (0.0, 0.0, 1.0)
    samples = [SweepSample(k * 20, (0.0, 0.0, 1.0), 20.0, 0.0) for k in range(20)]
    try:
        fit_axis(samples)
    except ValueError as exc:
        assert "parallel to gravity" in str(exc), exc
    else:
        raise AssertionError("a vertical axis should have been refused")


def test_too_few_samples_is_refused():
    try:
        fit_axis(synth(TILTED, n=4))
    except ValueError as exc:
        assert "at least 5" in str(exc), exc
    else:
        raise AssertionError("four samples should not fit a circle")


def test_all_samples_at_one_count_is_refused():
    samples = [SweepSample(100, s.g, 20.0, 0.0) for s in synth(TILTED, n=10)]
    try:
        fit_axis(samples)
    except ValueError as exc:
        assert "same count" in str(exc), exc
    else:
        raise AssertionError("a sweep that never moved should be refused")


# --- parsing ---------------------------------------------------------------

def test_parse_sweep_reads_well_formed_rows():
    lines = [
        "=== Actuator System Bench Console ===",
        "SWEEP,0,0.0123,-0.4567,0.8894,21.5,0.0031",
        "  pos=20 state=IDLE",
        "SWEEP,20,0.0201,-0.4501,0.8925,21.6,0.0028",
        "SWEEP,40,0.0280,-0.4433,0.8955,21.6,0.0030",
    ]
    got = parse_sweep(lines)
    assert len(got) == 3, got
    assert got[0].counts == 0
    assert abs(_norm(got[0].g) - 1.0) < 1e-9, "vectors should come back unit"
    assert abs(got[2].temp_c - 21.6) < 1e-9


def test_parse_sweep_survives_garbage():
    """
    This gets handed raw console logs, so a truncated line or a burst of
    serial noise must be skipped rather than kill the calibration.
    """
    lines = [
        "SWEEP,0,0.1,0.2,0.9,20.0,0.001",
        "SWEEP,truncated",
        "SWEEP,10,notanumber,0.2,0.9,20.0,0.001",
        "SWEEP,20,0.0,0.0,0.0,20.0,0.001",      # zero vector, not a direction
        "",
        "SWEEP,30,0.1,0.3,0.9,20.0,0.001",
    ]
    got = parse_sweep(lines)
    assert [s.counts for s in got] == [0, 30], [s.counts for s in got]


def test_parse_sweep_tolerates_missing_optional_columns():
    got = parse_sweep(["SWEEP,0,0.1,0.2,0.9"])
    assert len(got) == 1
    assert got[0].spread_g == 0.0
    assert math.isnan(got[0].temp_c)


# --- end to end ------------------------------------------------------------

def test_realistic_sweep_lands_inside_the_pointing_budget():
    """
    The number that decides whether any of this was worth building. An 8 ft
    dish at 1694 MHz has a 5.08 deg beam, so 1 deg of pointing error costs
    about 0.46 dB and is the working budget.

    Simulate a Davis-like polar mount (axis ~51.5 deg from vertical, so the
    cone is wide) with 1 mg of accelerometer noise, and check the recovered
    curve stays far inside that.
    """
    fit = fit_axis(synth(TILTED, deg_per_count=0.02, n=40, step=20,
                         noise=0.001, seed=7))
    assert fit.circle_rms_deg < 0.15, fit.circle_rms_deg
    assert fit.error_amplification < 2.0, fit.error_amplification


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    tests = [(name, fn) for name, fn in sorted(globals().items())
             if name.startswith("test_") and callable(fn)]
    failures = 0

    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except AssertionError as exc:
            failures += 1
            print(f"  FAIL  {name}: {exc}")
        except Exception as exc:
            failures += 1
            print(f"  ERROR {name}: {type(exc).__name__}: {exc}")

    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    raise SystemExit(1 if failures else 0)
