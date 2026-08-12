"""
test_geometry.py
----------------
Checks the pointing math against cases whose answers are known independently
of the code. No hardware, no serial port, no config.

This exists because the geometry is the one part of the system you cannot
sanity-check by eye. A wrong sign in the ENU rotation puts the dish 180
degrees out and looks perfectly plausible on screen.

    python test_geometry.py          (no pytest needed)
    pytest test_geometry.py
"""

import math

from geometry import (GEO_RADIUS_M, GOES18_LON_DEG, WGS84_A, LinearLinkage,
                      LinkageError, TriangleLinkage, degrees_per_count,
                      look_angles, make_linkage)


# --- look angles -----------------------------------------------------------

def test_subsatellite_point_is_straight_up():
    """
    Standing on the equator directly under the satellite: elevation 90, and
    the range is exactly the orbit radius minus the equatorial radius. Both
    the sphere and the WGS84 ellipsoid agree here, so this one is exact.
    """
    az, el, rng = look_angles(0.0, GOES18_LON_DEG, 0.0, GOES18_LON_DEG)
    assert abs(el - 90.0) < 1e-6, el
    assert abs(rng - (GEO_RADIUS_M - WGS84_A)) < 1.0, rng


def test_northern_site_looks_south():
    """Same meridian, north of the equator -> due south, azimuth 180."""
    az, el, _ = look_angles(40.0, GOES18_LON_DEG, 0.0, GOES18_LON_DEG)
    assert abs(az - 180.0) < 1e-6, az
    assert 40.0 < el < 50.0, el          # closed form gives 43.7 deg


def test_southern_site_looks_north():
    az, _, _ = look_angles(-40.0, GOES18_LON_DEG, 0.0, GOES18_LON_DEG)
    assert az % 360.0 < 1e-6 or abs(az - 360.0) < 1e-6, az


def test_elevation_is_symmetric_about_the_equator():
    """The ellipsoid is symmetric north-south, so these must match exactly."""
    _, north, _ = look_angles(35.0, GOES18_LON_DEG, 0.0, GOES18_LON_DEG)
    _, south, _ = look_angles(-35.0, GOES18_LON_DEG, 0.0, GOES18_LON_DEG)
    assert abs(north - south) < 1e-9, (north, south)


def test_site_east_of_satellite_looks_west():
    """GOES-18 is at 137 W. From 100 W the bird is to the west, az > 180."""
    az, el, _ = look_angles(40.0, -100.0, 0.0, GOES18_LON_DEG)
    assert 180.0 < az < 270.0, az
    assert el > 0.0, el


def test_site_west_of_satellite_looks_east():
    az, el, _ = look_angles(40.0, -170.0, 0.0, GOES18_LON_DEG)
    assert 90.0 < az < 180.0, az
    assert el > 0.0, el


def test_far_side_of_the_earth_is_below_the_horizon():
    """From the antipodal longitude the satellite is not visible at all."""
    _, el, _ = look_angles(0.0, GOES18_LON_DEG + 180.0, 0.0, GOES18_LON_DEG)
    assert el < 0.0, el


def test_elevation_matches_the_closed_form():
    """
    Cross-check against the standard spherical-Earth formula. They should agree
    to within a few tenths of a degree; a larger gap means a real bug rather
    than the sphere/ellipsoid difference.
    """
    lat, lon = 37.5, -122.0
    _, el, _ = look_angles(lat, lon, 0.0, GOES18_LON_DEG)

    d_lon = math.radians(GOES18_LON_DEG - lon)
    phi = math.radians(lat)
    ratio = WGS84_A / GEO_RADIUS_M
    cos_gamma = math.cos(d_lon) * math.cos(phi)
    closed_form = math.degrees(math.atan2(cos_gamma - ratio,
                                          math.sqrt(1.0 - cos_gamma ** 2)))
    assert abs(el - closed_form) < 0.35, (el, closed_form)


# --- linear linkage --------------------------------------------------------

def test_linear_round_trips():
    link = LinearLinkage(angle_a_deg=180.0, counts_a=0.0,
                         angle_b_deg=200.0, counts_b=400.0)
    for counts in (0.0, 137.0, 400.0, 512.0):
        assert abs(link.counts_for_angle(link.angle_for_counts(counts)) - counts) < 1e-9


def test_linear_hits_its_calibration_points():
    link = LinearLinkage(180.0, 0.0, 200.0, 400.0)
    assert abs(link.counts_for_angle(180.0) - 0.0) < 1e-9
    assert abs(link.counts_for_angle(200.0) - 400.0) < 1e-9
    assert abs(link.counts_for_angle(190.0) - 200.0) < 1e-9     # midpoint


def test_linear_rejects_degenerate_calibration():
    for bad in [(180.0, 0.0, 180.0, 400.0), (180.0, 100.0, 200.0, 100.0)]:
        try:
            LinearLinkage(*bad)
        except LinkageError:
            continue
        raise AssertionError(f"should have rejected {bad}")


# --- triangle linkage ------------------------------------------------------

def _triangle():
    return TriangleLinkage(pivot_to_base_mm=400.0,
                           pivot_to_carriage_mm=350.0,
                           retracted_length_mm=300.0,
                           mm_per_count=0.5,
                           angle_at_retract_deg=180.0,
                           direction=1)


def test_triangle_is_the_identity_at_home():
    link = _triangle()
    assert abs(link.angle_for_counts(0.0) - 180.0) < 1e-9
    assert abs(link.counts_for_angle(180.0) - 0.0) < 1e-9


def test_triangle_round_trips():
    link = _triangle()
    for counts in (0.0, 50.0, 200.0, 400.0):
        angle = link.angle_for_counts(counts)
        assert abs(link.counts_for_angle(angle) - counts) < 1e-6, counts


def test_triangle_extension_opens_the_angle():
    """Longer actuator -> wider included angle -> larger heading at dir=+1."""
    link = _triangle()
    assert link.angle_for_counts(200.0) > link.angle_for_counts(0.0)


def test_triangle_direction_flips_the_sense():
    a = _triangle()
    b = TriangleLinkage(400.0, 350.0, 300.0, 0.5, 180.0, direction=-1)
    da = a.angle_for_counts(200.0) - 180.0
    db = b.angle_for_counts(200.0) - 180.0
    assert abs(da + db) < 1e-9, (da, db)


def test_triangle_is_measurably_nonlinear():
    """
    The whole reason the triangle model exists. If equal count steps gave equal
    angle steps, the linear model would be exact everywhere and this would be
    dead code.
    """
    link = _triangle()
    first = link.angle_for_counts(100.0) - link.angle_for_counts(0.0)
    later = link.angle_for_counts(500.0) - link.angle_for_counts(400.0)
    assert abs(first - later) > 0.1, (first, later)


def test_triangle_rejects_an_unreachable_heading():
    link = _triangle()
    try:
        link.counts_for_angle(0.0)          # would need a negative side length
    except LinkageError:
        return
    raise AssertionError("should have rejected an unreachable heading")


def test_triangle_rejects_impossible_geometry():
    try:
        TriangleLinkage(400.0, 350.0, 5000.0, 0.5, 180.0, 1)
    except LinkageError:
        return
    raise AssertionError("a 5000 mm side cannot close a 400/350 triangle")


# --- resolution ------------------------------------------------------------

def test_degrees_per_count_is_positive_and_small():
    """
    Sanity on the number that bounds yaw accuracy. For plausible TVRO geometry
    one reed count should be a small fraction of a degree -- if this ever came
    back near a whole degree, the linkage needs a longer moment arm.
    """
    link = _triangle()
    res = degrees_per_count(link, 200.0)
    assert 0.0 < res < 1.0, res


def test_factory_builds_both_models():
    assert isinstance(make_linkage({"model": "linear", "angle_a_deg": 180.0,
                                    "counts_a": 0, "angle_b_deg": 200.0,
                                    "counts_b": 400}), LinearLinkage)
    assert isinstance(make_linkage({"model": "triangle", "pivot_to_base_mm": 400,
                                    "pivot_to_carriage_mm": 350,
                                    "retracted_length_mm": 300,
                                    "mm_per_count": 0.5,
                                    "angle_at_retract_deg": 180}), TriangleLinkage)


def test_factory_rejects_an_unknown_model():
    try:
        make_linkage({"model": "quadratic"})
    except LinkageError:
        return
    raise AssertionError("should have rejected an unknown model")


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
