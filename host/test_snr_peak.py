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
import math
import statistics

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


# --- main(): the dongle, opened once and always released --------------------
#
# The real SdrSnrMetric over conftest.py's FakeRtlSdr. The dongle is
# single-client, so a run that leaves it open blocks the next one.

def on_a_hill(dongle, lk, peak_deg=2.7, peak_db=12.0):
    """Serve blocks whose SNR follows a 12 deg beam around the fake link."""
    dongle.block = lambda: dongle.signal(
        peak_db - 12.0 * ((lk.pos - peak_deg) / 12.0) ** 2)


def _main_args(tmp_path, *mode):
    return ["--port", "COM9", *mode, "--ref", str(tmp_path / "pointing.json"),
            "--settle", "0", "--dwell", "0", "--ref-dwell", "0"]


def test_main_sweep_then_check_open_the_dongle_once_per_run(fake_port,
                                                            fake_rtlsdr, tmp_path):
    lk = fake_port(FakeLink())
    on_a_hill(fake_rtlsdr, lk)

    assert snr_peak.main(_main_args(tmp_path, "--sweep", "-8", "8", "1")) == 0
    [dev] = fake_rtlsdr.devices
    assert dev.closed and lk.closed
    assert dev.settings == [("sample_rate", 2_400_000.0),
                            ("center_freq", snr_peak.GOES18_HRIT_HZ),
                            ("gain", "auto")]
    readings = dev.reads.count(1 << 18)
    assert readings > 30                    # both passes and the reference
    # one settle on opening, then one flush per later dwell
    assert dev.reads.count(16384) == readings

    lk.closed = False
    assert snr_peak.main(_main_args(tmp_path, "--check")) == 0
    assert len(fake_rtlsdr.devices) == 2
    assert fake_rtlsdr.still_open == []
    assert lk.closed


def test_main_numeric_gain_reaches_the_dongle_and_the_reference(fake_port,
                                                                fake_rtlsdr,
                                                                tmp_path):
    """--gain 40 used to reach pyrtlsdr as the string '40': a TypeError."""
    lk = fake_port(FakeLink())
    on_a_hill(fake_rtlsdr, lk)
    args = _main_args(tmp_path, "--sweep", "-8", "8", "1") + ["--gain", "40"]
    assert snr_peak.main(args) == 0
    saved = json.loads((tmp_path / "pointing.json").read_text())
    assert saved["sdr"]["gain"] == 40.0

    assert snr_peak.main(_main_args(tmp_path, "--check")) == 0
    for dev in fake_rtlsdr.devices:
        assert ("gain", 40.0) in dev.settings


def test_main_refuses_a_bad_gain_before_opening_anything(fake_port, fake_rtlsdr,
                                                         tmp_path, capsys):
    lk = fake_port(FakeLink())
    args = _main_args(tmp_path, "--sweep", "-8", "8", "1") + ["--gain", "loud"]
    with pytest.raises(SystemExit) as exit_:
        snr_peak.main(args)
    assert exit_.value.code == 2
    assert fake_rtlsdr.devices == []
    assert not lk.opened


def test_main_releases_the_dongle_when_a_move_faults_mid_sweep(fake_port,
                                                               fake_rtlsdr,
                                                               tmp_path, capsys):
    lk = fake_port(FakeLink(fault_on_move=lambda a: a >= 0))
    on_a_hill(fake_rtlsdr, lk)
    assert snr_peak.main(_main_args(tmp_path, "--sweep", "-8", "8", "1")) == 1
    assert len(fake_rtlsdr.devices) == 1    # it was open: -8..-1 were read
    assert fake_rtlsdr.still_open == []
    assert lk.closed
    assert "stall" in capsys.readouterr().err


def test_main_releases_the_dongle_on_an_unexpected_error(fake_port, fake_rtlsdr,
                                                         tmp_path, capsys):
    def land(angle):
        if angle >= 0:
            raise RuntimeError("not one of the errors main() expects")
        return angle

    lk = fake_port(FakeLink(land=land))
    on_a_hill(fake_rtlsdr, lk)
    with pytest.raises(RuntimeError):
        snr_peak.main(_main_args(tmp_path, "--sweep", "-8", "8", "1"))
    assert len(fake_rtlsdr.devices) == 1
    assert fake_rtlsdr.still_open == []
    assert lk.closed


@pytest.mark.parametrize("mode", ["sweep", "check"])
def test_main_releases_the_dongle_on_ctrl_c(fake_port, fake_rtlsdr, tmp_path,
                                            mode, capsys):
    lk = fake_port(FakeLink())
    on_a_hill(fake_rtlsdr, lk)
    if mode == "check":
        assert snr_peak.main(_main_args(tmp_path, "--sweep", "-8", "8", "1")) == 0
        lk.closed = False
        args = _main_args(tmp_path, "--check")
    else:
        args = _main_args(tmp_path, "--sweep", "-8", "8", "1")
    fake_rtlsdr.raise_on_read[fake_rtlsdr.reads + 2] = KeyboardInterrupt()

    assert snr_peak.main(args) == 130
    assert fake_rtlsdr.devices[-1].reads        # it was open when interrupted
    assert fake_rtlsdr.still_open == []
    assert lk.closed


# --- --plot ----------------------------------------------------------------

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


@pytest.mark.parametrize("sweep, code", [
    (["-8", "8", "1"], 0),          # peak found
    (["-8", "1", "1"], 1),          # peak past the range: still drawn
])
def test_simulate_plot_writes_a_png(tmp_path, sweep, code):
    pytest.importorskip("matplotlib")
    png = tmp_path / "sweep.png"
    args = ["--simulate", "--sweep", *sweep, "--sim-seed", "3", "--plot", str(png)]
    assert snr_peak.main(args) == code
    assert png.read_bytes()[:8] == PNG_MAGIC


def test_plot_draws_missing_readings(tmp_path):
    """Angles with no reading are part of the picture, not dropped from it."""
    pytest.importorskip("matplotlib")
    from sweep_plot import plot_sweep
    result = PeakResult(None, None, [(-2.0, None, None, 0), (-1.0, 5.0, 1.0, 30),
                                     (0.0, 6.0, 1.0, 30), (1.0, None, None, 0)],
                        False, "no usable reading anywhere in the range")
    png = plot_sweep(tmp_path / "s.png", result, -2, 1)
    assert png.read_bytes()[:8] == PNG_MAGIC


def test_plot_with_check_is_refused():
    with pytest.raises(SystemExit):
        snr_peak.main(["--port", "COM9", "--check", "--plot"])


def test_missing_matplotlib_does_not_fail_the_sweep(monkeypatch, tmp_path, capsys):
    """The dish has already parked by then; a picture is not worth a failure."""
    import sweep_plot

    def no_matplotlib(*a, **kw):
        raise RuntimeError("--plot needs matplotlib: pip install matplotlib")

    monkeypatch.setattr(sweep_plot, "plot_sweep", no_matplotlib)
    args = ["--simulate", "--sweep", "-8", "8", "1", "--sim-seed", "3",
            "--plot", str(tmp_path / "s.png")]
    assert snr_peak.main(args) == 0
    assert "no plot" in capsys.readouterr().err


# --- the refining pass -----------------------------------------------------

class SincBeam(HillMetric):
    """
    A uniform-aperture beam, sinc^2 -- deliberately NOT the parabola the fit
    assumes, which is what the refining pass exists to cope with. -3 dB at
    half the beamwidth; anything 20 dB down reads as nothing.
    """

    def sample(self, dwell, settle=0.0):
        x = 1.3916 * 2.0 * (self.where() - self.peak_deg) / self.beamwidth
        g = 1.0 if abs(x) < 1e-9 else (math.sin(x) / x) ** 2
        db = self.peak_db + 10.0 * math.log10(max(g, 1e-12))
        if db < self.peak_db - 20.0:
            raise snr_peak.IqMetricError("nothing above the floor")
        return db, 0.5, 30


def _fine_commands(lk, coarse_n=17):
    """The refining pass's moves: after the coarse pass, before the park."""
    return lk.commands[coarse_n:-1]


def test_refine_sweeps_half_a_beamwidth_each_side_at_half_the_step():
    lk = FakeLink(pos=1.5)
    ref, result = _sweep(lk, HillMetric(lambda: lk.pos))
    fine = _fine_commands(lk)
    assert result.coarse.best_angle == pytest.approx(2.7, abs=1e-6)
    assert fine[0] == pytest.approx(2.7 - 6.0, abs=1e-6)
    assert fine[-1] == pytest.approx(2.7 + 6.0, abs=1e-6)
    assert len(fine) == 25
    assert fine[1] - fine[0] == pytest.approx(0.5)
    assert lk.commands[-1] == pytest.approx(result.best_angle)
    # one fit over both passes
    assert len(result.samples) == 17 + 25


def test_refine_may_pass_the_typed_range_but_never_the_soft_limit():
    lk = FakeLink(pos=0.0)
    _sweep(lk, HillMetric(lambda: lk.pos, peak_deg=9.0), lo=-4, hi=12, step=1)
    fine = _fine_commands(lk, coarse_n=17)
    assert max(fine) == pytest.approx(SOFT_LIMIT_DEG)      # 9 + 6 clipped to 13
    assert max(lk.commands) <= SOFT_LIMIT_DEG


def test_no_refine_is_one_pass():
    lk = FakeLink(pos=1.5)
    ref, result = run_sweep(Actuator(lk), HillMetric(lambda: lk.pos), -8, 8, 1,
                            settle=0.0, dwell=0.0, ref_dwell=0.0,
                            metric_name="test", refine=False)
    assert result.coarse is None
    assert len(lk.commands) == 17 + 1         # the sweep and the park


def test_refine_removes_the_bias_of_a_one_sided_fit():
    """
    Peak near the end of the coarse range on a beam that is not a parabola:
    the coarse points all sit on one flank and the fit leans toward it. No
    noise here, so all of the error is that bias.
    """
    lk = FakeLink(pos=0.0)
    ref, result = _sweep(lk, SincBeam(lambda: lk.pos, peak_deg=5.5))
    coarse_err = abs(result.coarse.best_angle - 5.5)
    refined_err = abs(result.best_angle - 5.5)
    assert coarse_err > 0.2
    assert refined_err < coarse_err / 3


def test_coarse_failure_skips_the_refining_pass():
    lk = FakeLink(pos=1.5)
    ref, result = _sweep(lk, HillMetric(lambda: lk.pos, peak_deg=9.0),
                         lo=-8, hi=0, step=1)
    assert ref is None
    assert len(lk.commands) == 9 + 1          # coarse pass, then back to start
    assert lk.commands[-1] == 1.5


def test_refined_reference_records_the_coarse_peak(tmp_path):
    lk = FakeLink(pos=1.5)
    ref, result = _sweep(lk, HillMetric(lambda: lk.pos))
    path = tmp_path / "pointing.json"
    save_reference(path, ref, {"freq_hz": 1.0}, result, -8, 8, 1)
    saved = json.loads(path.read_text())["sweep"]
    assert saved["coarse_peak"] == pytest.approx(2.7, abs=1e-6)
    assert saved["coarse_points"] == 17
    assert len(saved["points"]) == 42


def test_simulate_without_refine_still_runs():
    args = ["--simulate", "--sweep", "-8", "8", "1", "--sim-seed", "3", "--no-refine"]
    assert snr_peak.main(args) == 0


# --- peak.sweep / fit_peak -------------------------------------------------

def test_readings_are_credited_to_where_the_move_landed():
    from peak import sweep
    landed = []

    def move_to(a):
        landed.append(a + 0.2)
        return a + 0.2

    result = sweep(move_to, lambda: (1.0, 0.1, 10), 0, 4, 1, settle=0,
                   park=False, return_on_failure=False)
    assert [s[0] for s in result.samples] == pytest.approx(landed)


def test_a_mover_that_returns_nothing_is_credited_the_commanded_angle():
    from peak import sweep
    result = sweep(lambda a: None, lambda: (1.0, 0.1, 10), 0, 4, 1, settle=0,
                   park=False, return_on_failure=False)
    assert [s[0] for s in result.samples] == [0, 1, 2, 3, 4]


def test_sweep_never_steps_past_hi():
    """Accumulating 0.5 twenty-five times overshoots 13 by 2e-15."""
    from peak import sweep
    sent = []
    sweep(lambda a: sent.append(a), lambda: (1.0, 0.1, 10), 1.0, 13.0, 0.5,
          settle=0, park=False, return_on_failure=False)
    assert len(sent) == 25
    assert max(sent) == 13.0


def test_fit_peak_sorts_samples_from_two_passes():
    from peak import fit_peak
    pts = [(a, 10 - (a - 1.0) ** 2, 0.1, 10) for a in (-2, 0, 2, 4)]
    pts += [(a, 10 - (a - 1.0) ** 2, 0.1, 10) for a in (-1, 1, 3)]
    result = fit_peak(pts)
    assert [s[0] for s in result.samples] == [-2, -1, 0, 1, 2, 3, 4]
    assert result.best_angle == pytest.approx(1.0)


# --- the realistic beam, and Monte Carlo trials ----------------------------

from snr_peak import beam_loss_db, bessel_j1, run_trials  # noqa: E402


@pytest.mark.parametrize("x, j1", [
    (0.0, 0.0),
    (1.0, 0.4400505857449335),          # tabulated
    (-1.0, -0.4400505857449335),        # odd function
    (3.8317059702075125, 0.0),          # first zero
    (5.0, -0.3275791375914652),
])
def test_bessel_j1_matches_tables(x, j1):
    assert bessel_j1(x) == pytest.approx(j1, abs=1e-10)


@pytest.mark.parametrize("beam", ["parabola", "airy"])
def test_both_beams_are_3db_down_at_half_the_beamwidth(beam):
    assert beam_loss_db(0.0, 12.0, beam) == 0.0
    assert beam_loss_db(6.0, 12.0, beam) == pytest.approx(3.0, abs=0.02)
    assert beam_loss_db(-6.0, 12.0, beam) == pytest.approx(3.0, abs=0.02)


def test_airy_falls_faster_than_the_parabola_down_the_flanks():
    """The difference the fit cannot model, and the refining pass corrects."""
    assert beam_loss_db(9.0, 12.0, "airy") > beam_loss_db(9.0, 12.0, "parabola") + 0.5
    assert beam_loss_db(12.0, 12.0, "airy") > beam_loss_db(12.0, 12.0, "parabola") + 4.0


def test_unknown_beam_is_refused():
    with pytest.raises(ValueError):
        SimDish(0.0, beam="horn")


def test_trials_are_repeatable_with_a_seed(capsys):
    a = run_trials(5, 2.7, "parabola", -8, 8, 1, 3.0, 10.0, True, seed=42)
    b = run_trials(5, 2.7, "parabola", -8, 8, 1, 3.0, 10.0, True, seed=42)
    assert a["fitted"] == b["fitted"]
    assert a["found"] == 5
    assert "5 simulated sweeps" in capsys.readouterr().out


def test_trials_show_the_one_pass_bias_on_a_real_beam_and_refine_removes_it():
    """
    The case --trials exists for: an edge peak on an Airy beam. The one-pass
    fit leans the same way every time -- a mean far from zero -- and the
    two-pass fit does not.
    """
    one = run_trials(40, 5.5, "airy", -8, 8, 1, 3.0, 10.0, False, seed=7)
    two = run_trials(40, 5.5, "airy", -8, 8, 1, 3.0, 10.0, True, seed=7)
    one_bias = statistics.fmean(one["fitted"])
    two_bias = statistics.fmean(two["fitted"])
    assert one_bias < -0.15
    assert abs(two_bias) < 0.05


def test_trials_need_simulate():
    with pytest.raises(SystemExit):
        snr_peak.main(["--port", "COM9", "--sweep", "-8", "8", "1", "--trials", "5"])


def test_trials_must_be_positive():
    with pytest.raises(SystemExit):
        snr_peak.main(["--simulate", "--sweep", "-8", "8", "1", "--trials", "0"])


def test_simulate_refuses_a_bad_range_up_front(capsys):
    assert snr_peak.main(["--simulate", "--sweep", "-20", "8", "1"]) == 2
    assert "soft limit" in capsys.readouterr().err


def test_trials_plot_writes_a_histogram(tmp_path):
    pytest.importorskip("matplotlib")
    png = tmp_path / "mc.png"
    args = ["--simulate", "--sweep", "-8", "8", "1", "--sim-beam", "airy",
            "--trials", "5", "--sim-seed", "1", "--plot", str(png)]
    assert snr_peak.main(args) == 0
    assert png.read_bytes()[:8] == PNG_MAGIC


def test_airy_single_run_plot(tmp_path):
    pytest.importorskip("matplotlib")
    png = tmp_path / "airy.png"
    args = ["--simulate", "--sweep", "-8", "8", "1", "--sim-beam", "airy",
            "--sim-seed", "1", "--plot", str(png)]
    assert snr_peak.main(args) == 0
    assert png.read_bytes()[:8] == PNG_MAGIC
