"""
config.py
---------
Site, linkage and trim, stored as JSON next to the script.

The trim lives here rather than in the sketch's EEPROM on purpose. It is a
property of *this site and this mount*, not of the Arduino -- swap the board
and the trim should follow the dish, not the microcontroller.

There are no fallback defaults for the site coordinates. A dish pointed
confidently at the wrong patch of sky because a placeholder latitude went
unnoticed is worse than a script that refuses to start.
"""

from __future__ import annotations

import json
from pathlib import Path

from geometry import GOES18_LON_DEG, make_linkage

DEFAULT_PATH = Path(__file__).with_name("config.json")


TEMPLATE = {
    "site": {
        "_comment": "Your ground station. Longitude is East-positive: 122.3 W is -122.3.",
        "latitude_deg": None,
        "longitude_deg": None,
        "altitude_m": 0.0,
    },
    "satellite": {
        "name": "GOES-18",
        "longitude_deg": GOES18_LON_DEG,
    },
    "headings": {
        "_comment": "Set magnetic=true if the linkage headings below were taken "
                    "with a compass rather than derived from true north. "
                    "Declination is positive east.",
        "magnetic": False,
        "declination_deg": 0.0,
    },
    "linkage": {
        "_comment": "model is 'linear' (two measured points, no tape measure "
                    "needed) or 'triangle' (physically correct, needs the mount "
                    "geometry). See host/README.md.",
        "model": "linear",
        "angle_a_deg": None,
        "counts_a": 0,
        "angle_b_deg": None,
        "counts_b": None,
    },
    "tuning": {
        "_comment": "trim_counts is added to every computed target. It is what "
                    "the manual keyboard tuning writes back.",
        "trim_counts": 0,
        "nudge_counts": 1,
    },
}


class ConfigError(RuntimeError):
    pass


class Config:
    def __init__(self, data: dict, path: Path):
        self.data = data
        self.path = path

    # --- io ----------------------------------------------------------------

    @classmethod
    def load(cls, path: Path = DEFAULT_PATH) -> "Config":
        path = Path(path)
        if not path.exists():
            path.write_text(json.dumps(TEMPLATE, indent=2), encoding="utf-8")
            raise ConfigError(
                f"wrote a blank config to {path}\n"
                "Fill in the site coordinates and the linkage calibration, "
                "then run again. host/README.md walks through both.")
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise ConfigError(f"{path} is not valid JSON: {exc}") from exc
        return cls(data, path)

    def save(self):
        self.path.write_text(json.dumps(self.data, indent=2), encoding="utf-8")

    # --- accessors ---------------------------------------------------------

    @property
    def site(self):
        site = self.data.get("site", {})
        lat = site.get("latitude_deg")
        lon = site.get("longitude_deg")
        if lat is None or lon is None:
            raise ConfigError(
                f"site latitude/longitude are unset in {self.path}. "
                "Refusing to compute a pointing angle from a placeholder.")
        if not -90.0 <= lat <= 90.0:
            raise ConfigError(f"site latitude {lat} is out of range")
        if not -180.0 <= lon <= 180.0:
            raise ConfigError(f"site longitude {lon} is out of range "
                              "(East-positive, so use -122.3 not 122.3 W)")
        return float(lat), float(lon), float(site.get("altitude_m", 0.0))

    @property
    def satellite_lon(self) -> float:
        return float(self.data.get("satellite", {}).get("longitude_deg", GOES18_LON_DEG))

    @property
    def satellite_name(self) -> str:
        return str(self.data.get("satellite", {}).get("name", "GOES-18"))

    def linkage(self):
        spec = self.data.get("linkage")
        if not spec:
            raise ConfigError(f"no linkage section in {self.path}")
        missing = [k for k, v in spec.items() if v is None and not k.startswith("_")]
        if missing:
            raise ConfigError(
                f"linkage calibration incomplete in {self.path}: {', '.join(missing)}")
        return make_linkage(spec)

    # --- heading frame -----------------------------------------------------

    def to_linkage_frame(self, true_heading_deg: float) -> float:
        """
        look_angles() always returns TRUE azimuth. If the linkage was
        calibrated against compass headings, convert into that frame so the
        two agree.

            magnetic = true - declination_east
        """
        headings = self.data.get("headings", {})
        if not headings.get("magnetic", False):
            return true_heading_deg
        return (true_heading_deg - float(headings.get("declination_deg", 0.0))) % 360.0

    # --- trim --------------------------------------------------------------

    @property
    def trim(self) -> int:
        return int(self.data.get("tuning", {}).get("trim_counts", 0))

    @trim.setter
    def trim(self, value: int):
        self.data.setdefault("tuning", {})["trim_counts"] = int(value)

    @property
    def nudge(self) -> int:
        return max(1, int(self.data.get("tuning", {}).get("nudge_counts", 1)))

    @nudge.setter
    def nudge(self, value: int):
        self.data.setdefault("tuning", {})["nudge_counts"] = max(1, int(value))
