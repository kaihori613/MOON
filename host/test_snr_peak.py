"""
test_snr_peak.py
----------------
The outer loop's safety rules, checked against a fake serial link and a fake
dish. No hardware, no SDR.

Every rule here exists because breaking it moves the dish somewhere wrong
without anything looking wrong: a reading credited to the wrong angle, a
failed sweep that leaves the dish at the first swept angle instead of the old
aim, a 'g' sent to a sketch that reads it as reed counts.

    pytest test_snr_peak.py
"""

import json

import pytest

import link
import snr_peak
from link import ActuatorFault
from peak import PeakResult, PointingReference
from snr_peak import (LAND_TOL_DEG, SOFT_LIMIT_DEG, Actuator, PeakRunError,
                      SimDish, load_reference, run_check, run_sweep,
                      save_reference, validate_range)


# --- fakes -----------------------------------------------------------------

class FakeLink:
    """
    Stands in for link.ActuatorLink. Every 'g' is recorded, and land(angle)
    decides where the PID ends up -- exactly on target unless a test says
    otherwise.
    """

    def __init__(self, pos=0.0, unit="deg", fault=None, encoder_ok=True,
                 land=None, fault_on_move=None):
        self.pos = pos
        self.unit = unit
        self.fault = fault
        self.encoder_ok = encoder_ok
        self.land = land or (lambda a: a)
        self.fault_on_move = fault_on_move
        self.commands = []
        self.opened = False
        self.closed = False

    def open(self):
        self.opened = True

    def close(self):
        self.closed = True

    def status(self):
        return {"pos": self.pos, "unit": self.unit, "fault": self.fault,
                "encoder_ok": self.encoder_ok, "state": "IDLE"}

    def move_to_deg(self, angle):
        self.commands.append(angle)
        self.pos = self.land(angle)

    def wait_idle(self, timeout=180.0):
        if self.fault_on_move is not None and self.fault_on_move(self.commands[-1]):
            raise ActuatorFault(f"stall at pos={self.pos}")
        return self.status()


class HillMetric:
    """Noise-free beam: an exact parabola, so the fitted peak is exact too."""

    def __init__(self, where, peak_deg=2.7, peak_db=12.0, beamwidth=12.0):
        self.where = where              # callable returning the current angle
        self.peak_deg = peak_deg
        self.peak_db = peak_db
        self.beamwidth = beamwidth

    def sample(self, dwell, settle=0.0):
        err = self.where() - self.peak_deg
        return self.peak_db - 12.0 * (err / self.beamwidth) ** 2, 0.5, 30


def _sweep(link_, metric, lo=-8, hi=8, step=1):
    return run_sweep(Actuator(link_), metric, lo, hi, step, settle=0.0,
                     dwell=0.0, ref_dwell=0.0, metric_name="test")


# --- validate_range --------------------------------------------------------

@pytest.mark.parametrize("lo, hi", [(-14, 8), (-8, 13.5), (-20, 20)])
def test_range_past_soft_limit_is_refused(lo, hi):
    """A target past the limit latches fault=range in the sketch."""
    with pytest.raises(PeakRunError, match="soft limit"):
        validate_range(lo, hi, 1)


def test_range_at_soft_limit_is_allowed():
    assert validate_range(-SOFT_LIMIT_DEG, SOFT_LIMIT_DEG, 1) == 27


@pytest.mark.parametrize("lo, hi, step, why", [
    (-8, 8, 0, "STEP"),
    (-8, 8, -1, "STEP"),
    (8, -8, 1, "HI"),
    (-2, 1, 1, "too few"),          # 4 points: the fit needs flanks
])
def test_bad_ranges_are_refused(lo, hi, step, why):
    with pytest.raises(PeakRunError, match=why):
        validate_range(lo, hi, step)


def test_point_count_survives_float_steps():
    """0.1 steps accumulate rounding; 0..1 must still be 11 points, not 10."""
    assert validate_range(0.0, 1.0, 0.1) == 11


# --- Actuator --------------------------------------------------------------

def test_landing_inside_tolerance_is_accepted():
    lk = FakeLink(land=lambda a: a + LAND_TOL_DEG * 0.9)
    assert Actuator(lk).move_to(3.0) == pytest.approx(3.0 + LAND_TOL_DEG * 0.9)


def test_landing_outside_tolerance_is_refused():
    """Refused or pre-empted by a panel button: a reading there is mislabeled."""
    lk = FakeLink(land=lambda a: a + LAND_TOL_DEG * 1.1)
    with pytest.raises(PeakRunError, match="stopped at"):
        Actuator(lk).move_to(3.0)


def test_missing_encoder_is_refused():
    """A 'g' with no encoder is refused without a fault and the dish stays put."""
    lk = FakeLink(encoder_ok=False)
    with pytest.raises(PeakRunError, match="encoder"):
        Actuator(lk).move_to(3.0)
    with pytest.raises(PeakRunError, match="encoder"):
        Actuator(lk).position()


def test_fault_during_move_propagates():
    lk = FakeLink(fault_on_move=lambda a: True)
    with pytest.raises(ActuatorFault):
        Actuator(lk).move_to(3.0)


# --- run_sweep -------------------------------------------------------------

def test_sweep_finds_and_parks_on_the_peak():
    lk = FakeLink(pos=1.5)
    ref, result = _sweep(lk, HillMetric(lambda: lk.pos))
    assert result.ok
    assert result.interpolated
    assert ref.angle == pytest.approx(2.7, abs=1e-6)
    assert lk.commands[-1] == pytest.approx(2.7, abs=1e-6)
    assert ref.metric_n == 30           # recorded, so --check can use it


def test_failed_move_returns_to_start_not_first_swept_angle():
    """peak.sweep() alone would go back to -8; the old aim was +1.5."""
    lk = FakeLink(pos=1.5, land=lambda a: a + 1.0 if abs(a - 2.0) < 1e-6 else a)
    with pytest.raises(PeakRunError):
        _sweep(lk, HillMetric(lambda: lk.pos))
    assert lk.commands[0] == pytest.approx(-8.0)
    assert lk.commands[-1] == 1.5
    assert lk.pos == 1.5


def test_fault_mid_sweep_tries_to_return_to_start():
    lk = FakeLink(pos=1.5, fault_on_move=lambda a: abs(a - 0.0) < 1e-6)
    with pytest.raises(ActuatorFault):
        _sweep(lk, HillMetric(lambda: lk.pos))
    assert lk.commands[-1] == 1.5


def test_peak_at_edge_is_no_answer_and_returns_to_start():
    """Best reading at the last point means the real peak is outside the range."""
    lk = FakeLink(pos=1.5)
    ref, result = _sweep(lk, HillMetric(lambda: lk.pos, peak_deg=9.0),
                         lo=-8, hi=0, step=1)
    assert ref is None
    assert not result.ok
    assert "edge" in result.reason
    assert lk.commands[-1] == 1.5


def test_no_signal_anywhere_returns_to_start():
    class Silent:
        def sample(self, dwell, settle=0.0):
            raise snr_peak.IqMetricError("nothing on the air")

    lk = FakeLink(pos=-0.5)
    ref, result = _sweep(lk, Silent())
    assert ref is None
    assert "no usable reading" in result.reason
    assert lk.commands[-1] == -0.5


def test_sweep_range_is_checked_before_anything_moves():
    lk = FakeLink(pos=1.5)
    with pytest.raises(PeakRunError, match="soft limit"):
        _sweep(lk, HillMetric(lambda: lk.pos), lo=-15, hi=8)
    assert lk.commands == []


@pytest.mark.parametrize("seed", range(20))
def test_simulated_dish_peak_is_found_within_half_a_degree(seed):
    """
    The noisy fake dish from --simulate: 1 dB per reading, moves landing
    anywhere inside the 0.25 deg deadband. Twenty fixed seeds, so a
    regression in the search shows up as the same failure every run.
    """
    dish = SimDish(2.7, seed=seed)
    ref, result = run_sweep(dish, dish, -8, 8, 1, settle=0.0, dwell=3.0,
                            ref_dwell=10.0, metric_name="sim")
    assert result.ok
    assert abs(ref.angle - 2.7) < 0.5


# --- run_check -------------------------------------------------------------

def test_check_moves_to_reference_and_never_re_peaks():
    lk = FakeLink(pos=0.0)
    ref = PointingReference(2.7, 12.0, 1.0, metric_n=100, beamwidth_deg=12.0)

    class Dropped:
        def sample(self, dwell, settle=0.0):
            return 7.0, 1.0, 30         # 5 dB down: FAILED

    verdict = run_check(Actuator(lk), Dropped(), ref, settle=0.0, dwell=0.0)
    assert verdict.status == "failed"
    assert verdict.needs_repeak
    assert lk.commands == [2.7]         # one move, to the reference, nothing else


# --- the saved reference ---------------------------------------------------

def test_reference_round_trips_through_the_file(tmp_path):
    ref = PointingReference(2.566, 11.86, 1.01, metric_name="sdr-snr m2m4",
                            when="2026-10-27T00:00:00+00:00",
                            beamwidth_deg=12.0, metric_n=100)
    result = PeakResult(2.624, 11.9, [(-1.0, 11.1, 1.0, 30), (0.0, None, None, 0)],
                        True)
    settings = {"freq_hz": 1_694_100_000.0, "fs": 2_400_000.0,
                "n_samples": 262144, "gain": "auto", "device_index": 0,
                "signal_bw": None, "estimator": "auto"}
    path = tmp_path / "pointing.json"

    save_reference(path, ref, settings, result, -8, 8, 1)
    back, back_settings = load_reference(path)

    assert back.to_dict() == ref.to_dict()
    assert back_settings == settings
    points = json.loads(path.read_text())["sweep"]["points"]
    assert points == [[-1.0, 11.1], [0.0, None]]


def test_missing_reference_is_a_clear_error(tmp_path):
    with pytest.raises(PeakRunError, match="run --sweep first"):
        load_reference(tmp_path / "nope.json")


# --- main(): refusals before anything moves --------------------------------

@pytest.fixture
def fake_port(monkeypatch):
    """main() with the SDR preflight skipped and the serial port faked."""
    holder = {}

    def make(port, baud, verbose=False):
        return holder["link"]

    monkeypatch.setattr(snr_peak, "preflight_sdr", lambda settings: None)
    monkeypatch.setattr(link, "ActuatorLink", make)

    def use(fake):
        holder["link"] = fake
        return fake
    return use


SWEEP_ARGS = ["--port", "COM9", "--sweep", "-8", "8", "1", "--no-save",
              "--settle", "0", "--dwell", "0", "--ref-dwell", "0"]


def test_main_refuses_actuator_v1(fake_port, capsys):
    """actuator_v1 reads 'g 5' as five reed counts."""
    lk = fake_port(FakeLink(unit="counts"))
    assert snr_peak.main(SWEEP_ARGS) == 1
    assert lk.commands == []
    assert lk.closed
    assert "not actuator_v2" in capsys.readouterr().err


def test_main_refuses_a_faulted_controller(fake_port, capsys):
    lk = fake_port(FakeLink(fault="range"))
    assert snr_peak.main(SWEEP_ARGS) == 1
    assert lk.commands == []
    assert "faulted (range)" in capsys.readouterr().err


def test_main_refuses_bad_range_before_opening_the_port(fake_port, capsys):
    lk = fake_port(FakeLink())
    args = ["--port", "COM9", "--sweep", "-20", "8", "1"]
    assert snr_peak.main(args) == 2
    assert not lk.opened
    assert "soft limit" in capsys.readouterr().err


def test_main_check_without_reference_does_not_open_the_port(fake_port, tmp_path):
    lk = fake_port(FakeLink())
    args = ["--port", "COM9", "--check", "--ref", str(tmp_path / "nope.json")]
    assert snr_peak.main(args) == 2
    assert not lk.opened


def test_main_refuses_when_sdr_preflight_fails(fake_port, monkeypatch, capsys):
    def no_dongle(settings):
        raise PeakRunError("the SDR is not delivering samples: no device")

    monkeypatch.setattr(snr_peak, "preflight_sdr", no_dongle)
    lk = fake_port(FakeLink())
    assert snr_peak.main(SWEEP_ARGS) == 2
    assert not lk.opened
    assert "not delivering samples" in capsys.readouterr().err


def test_simulate_runs_end_to_end():
    args = ["--simulate", "--sweep", "-8", "8", "1", "--sim-seed", "3"]
    assert snr_peak.main(args) == 0
