"""
geometry.py
-----------
Two independent pieces of math, deliberately kept apart:

  1. Where GOES-18 is       -- look angles from a ground site to a
                               geostationary satellite. Depends only on where
                               you are standing.
  2. Where the actuator has -- yaw angle <-> reed counts. Depends only on how
     to be to point there      the jack is bolted to the mount.

Neither knows about serial ports, and everything here is a pure function, so
it can be checked against known values with no hardware attached. Run
test_geometry.py.

What is NOT modelled: atmospheric refraction. It lifts *apparent* elevation by
roughly 0.1 deg at 30 deg elevation, and much more near the horizon. Refraction
is purely vertical, so it does not touch yaw at all -- it would only matter if
pitch is ever automated.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# --- WGS84 ellipsoid -------------------------------------------------------
WGS84_A = 6378137.0                      # equatorial radius, m
WGS84_F = 1.0 / 298.257223563            # flattening
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)     # first eccentricity squared

# Nominal geostationary orbit radius, measured from the centre of the Earth.
GEO_RADIUS_M = 42164169.0

# GOES-18 took over as GOES-West at 137.0 W in January 2023. It is station-kept
# within a fraction of a degree of that, which is far below the pointing
# resolution of a jack-driven dish, so a fixed longitude is plenty.
GOES18_LON_DEG = -137.0


# ===========================================================================
#  1. LOOK ANGLES
# ===========================================================================

def site_ecef(lat_deg: float, lon_deg: float, alt_m: float = 0.0):
    """Geodetic site position -> Earth-centred, Earth-fixed metres."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)

    # Radius of curvature in the prime vertical.
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)

    x = (n + alt_m) * cos_lat * math.cos(lon)
    y = (n + alt_m) * cos_lat * math.sin(lon)
    z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def geo_satellite_ecef(sat_lon_deg: float):
    """A geostationary satellite sits on the equatorial plane, so z is zero."""
    lon = math.radians(sat_lon_deg)
    return GEO_RADIUS_M * math.cos(lon), GEO_RADIUS_M * math.sin(lon), 0.0


def _ecef_to_enu(v, lat_deg: float, lon_deg: float):
    """Rotate an ECEF vector into the site's East / North / Up frame."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    vx, vy, vz = v

    east = -sin_lon * vx + cos_lon * vy
    north = -sin_lat * cos_lon * vx - sin_lat * sin_lon * vy + cos_lat * vz
    up = cos_lat * cos_lon * vx + cos_lat * sin_lon * vy + sin_lat * vz
    return east, north, up


def look_angles(site_lat_deg: float,
                site_lon_deg: float,
                site_alt_m: float = 0.0,
                sat_lon_deg: float = GOES18_LON_DEG):
    """
    Return (azimuth_true_deg, elevation_deg, slant_range_m).

    Azimuth is TRUE north, clockwise. If you are going to check it against a
    handheld compass you have to add your local magnetic declination -- see
    Config.to_true() for that conversion.

    Elevation below zero means the satellite is under your horizon and no
    amount of pointing will help.

    Longitudes are East-positive: 137.0 W is -137.0.
    """
    sx, sy, sz = geo_satellite_ecef(sat_lon_deg)
    gx, gy, gz = site_ecef(site_lat_deg, site_lon_deg, site_alt_m)

    v = (sx - gx, sy - gy, sz - gz)
    east, north, up = _ecef_to_enu(v, site_lat_deg, site_lon_deg)

    azimuth = math.degrees(math.atan2(east, north)) % 360.0
    horizontal = math.hypot(east, north)
    elevation = math.degrees(math.atan2(up, horizontal))
    slant_range = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)

    return azimuth, elevation, slant_range


# ===========================================================================
#  2. LINKAGE:  yaw angle <-> reed counts
# ===========================================================================
#  A linear actuator does not move the mount through equal angles for equal
#  extensions -- the relationship is a triangle, so it is nonlinear. How much
#  that matters depends on how wide an arc you sweep.
#
#  Two models are provided. Both must implement the same pair of methods so
#  the rest of the program does not care which one is configured.


class LinkageError(ValueError):
    """Raised when a requested angle cannot be reached by this linkage."""


@dataclass
class LinearLinkage:
    """
    Straight line fit through two measured points.

    Calibration, which needs nothing but a compass:
      1. Home the actuator ('h'). That is counts = 0.
      2. Sight the boom and write down the heading -> point A.
      3. Drive out to some counts well along the stroke.
      4. Sight it again -> point B.

    Good to a fraction of a degree over a modest arc, and it drifts from the
    truth as the arc widens. If the two calibration points bracket the arc you
    actually use, the error in between is small.
    """
    angle_a_deg: float
    counts_a: float
    angle_b_deg: float
    counts_b: float

    def __post_init__(self):
        if self.angle_a_deg == self.angle_b_deg:
            raise LinkageError("linear linkage: the two calibration angles are identical")
        if self.counts_a == self.counts_b:
            raise LinkageError("linear linkage: the two calibration counts are identical")

    @property
    def _counts_per_deg(self) -> float:
        return (self.counts_b - self.counts_a) / (self.angle_b_deg - self.angle_a_deg)

    def counts_for_angle(self, angle_deg: float) -> float:
        return self.counts_a + (angle_deg - self.angle_a_deg) * self._counts_per_deg

    def angle_for_counts(self, counts: float) -> float:
        return self.angle_a_deg + (counts - self.counts_a) / self._counts_per_deg

    def describe(self) -> str:
        return (f"linear: {self.angle_a_deg:.2f} deg @ {self.counts_a:.0f} cts, "
                f"{self.angle_b_deg:.2f} deg @ {self.counts_b:.0f} cts "
                f"({self._counts_per_deg:.2f} cts/deg)")


@dataclass
class TriangleLinkage:
    """
    The physically correct model: pivot, base mount and carriage mount form a
    triangle whose third side is the actuator itself.

                    carriage mount
                       /\\
        pivot_to_    /    \\  actuator length L
        carriage   /        \\
                 /  theta     \\
            pivot ------------ base mount
                 pivot_to_base

    Law of cosines gives the included angle at the pivot:

        theta = acos( (a^2 + b^2 - L^2) / (2ab) )

    and the mount's heading changes one-for-one with theta.

    Calibration needs a tape measure for a and b, plus mm_per_count from the
    bench test, plus one compass heading taken at the homed position.
    """
    pivot_to_base_mm: float          # a
    pivot_to_carriage_mm: float      # b
    retracted_length_mm: float       # L at counts = 0 (i.e. homed)
    mm_per_count: float              # from the sensor bench test
    angle_at_retract_deg: float      # measured heading with the actuator homed
    direction: int = 1               # +1 if extending increases heading, else -1

    def __post_init__(self):
        if self.direction not in (1, -1):
            raise LinkageError("triangle linkage: direction must be +1 or -1")
        for name in ("pivot_to_base_mm", "pivot_to_carriage_mm",
                     "retracted_length_mm", "mm_per_count"):
            if getattr(self, name) <= 0:
                raise LinkageError(f"triangle linkage: {name} must be positive")
        self._theta0 = self._theta_for_length(self.retracted_length_mm)

    def _theta_for_length(self, length_mm: float) -> float:
        a, b = self.pivot_to_base_mm, self.pivot_to_carriage_mm
        cos_theta = (a * a + b * b - length_mm * length_mm) / (2.0 * a * b)
        if not -1.0 <= cos_theta <= 1.0:
            raise LinkageError(
                f"actuator length {length_mm:.1f} mm cannot close a triangle with "
                f"sides {a:.1f} and {b:.1f} mm -- check the measured geometry")
        return math.acos(cos_theta)

    def _length_for_theta(self, theta_rad: float) -> float:
        a, b = self.pivot_to_base_mm, self.pivot_to_carriage_mm
        return math.sqrt(a * a + b * b - 2.0 * a * b * math.cos(theta_rad))

    def counts_for_angle(self, angle_deg: float) -> float:
        delta = math.radians((angle_deg - self.angle_at_retract_deg) / self.direction)
        theta = self._theta0 + delta
        if not 0.0 < theta < math.pi:
            raise LinkageError(
                f"heading {angle_deg:.2f} deg is outside the arc this linkage can reach")
        length = self._length_for_theta(theta)
        return (length - self.retracted_length_mm) / self.mm_per_count

    def angle_for_counts(self, counts: float) -> float:
        length = self.retracted_length_mm + counts * self.mm_per_count
        theta = self._theta_for_length(length)
        return self.angle_at_retract_deg + self.direction * math.degrees(theta - self._theta0)

    def describe(self) -> str:
        return (f"triangle: a={self.pivot_to_base_mm:.0f} mm, "
                f"b={self.pivot_to_carriage_mm:.0f} mm, "
                f"L0={self.retracted_length_mm:.0f} mm, "
                f"{self.mm_per_count:.3f} mm/count, "
                f"{self.angle_at_retract_deg:.2f} deg at home, dir={self.direction:+d}")


def make_linkage(spec: dict):
    """Build whichever linkage model the config asked for."""
    model = spec.get("model", "linear").lower()

    if model == "linear":
        return LinearLinkage(
            angle_a_deg=float(spec["angle_a_deg"]),
            counts_a=float(spec["counts_a"]),
            angle_b_deg=float(spec["angle_b_deg"]),
            counts_b=float(spec["counts_b"]),
        )

    if model == "triangle":
        return TriangleLinkage(
            pivot_to_base_mm=float(spec["pivot_to_base_mm"]),
            pivot_to_carriage_mm=float(spec["pivot_to_carriage_mm"]),
            retracted_length_mm=float(spec["retracted_length_mm"]),
            mm_per_count=float(spec["mm_per_count"]),
            angle_at_retract_deg=float(spec["angle_at_retract_deg"]),
            direction=int(spec.get("direction", 1)),
        )

    raise LinkageError(f"unknown linkage model {model!r} -- use 'linear' or 'triangle'")


def degrees_per_count(linkage, counts: float, span: float = 1.0) -> float:
    """
    Local angular resolution: how much heading one reed count buys you at this
    point in the stroke. This is the hard floor on yaw accuracy -- the control
    loop cannot hold tighter than one count no matter how it is tuned.
    """
    try:
        return abs(linkage.angle_for_counts(counts + span) -
                   linkage.angle_for_counts(counts)) / span
    except LinkageError:
        return float("nan")
