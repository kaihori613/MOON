"""
test_check_pointing.py
----------------------
peak.check_pointing(): when a drop in SNR is real enough to say so.

The statistical tests replay the simulation behind the table in README.md:
1 dB of scatter per reading, a 30-reading reference, a 10-reading check. They
use a fixed seed, so a change to the threshold shows up as the same failure
every run rather than as a flaky one.

    pytest test_check_pointing.py
"""

import math
import random
import statistics

import pytest

from peak import HealthVerdict, PointingReference, check_pointing

TRIALS = 2000
NOISE_DB = 1.0
REF_N, CHECK_N = 30, 10


def _reading(rng, true_db, n):
    values = [true_db + rng.gauss(0.0, NOISE_DB) for _ in range(n)]
    return statistics.fmean(values), statistics.stdev(values), n


def _flagged_fraction(drop_db, record_n=True, seed=1):
    """How often a check `drop_db` below the reference says re-peak."""
    rng = random.Random(seed)
    flagged = 0
    for _ in range(TRIALS):
        mean, sd, n = _reading(rng, 12.0, REF_N)
        ref = PointingReference(0.0, mean, sd, metric_n=n if record_n else None)
        now = _reading(rng, 12.0 - drop_db, CHECK_N)
        if check_pointing(ref, lambda: now).needs_repeak:
            flagged += 1
    return flagged / TRIALS


# --- the README table, as tests -------------------------------------------

def test_healthy_link_never_raises_a_false_alarm():
    assert _flagged_fraction(0.0) == 0.0


def test_three_db_drop_is_caught_nearly_always():
    """3 dB is about 6 deg off on a 12 deg beam. README: 99.8%."""
    assert _flagged_fraction(3.0) >= 0.99


def test_four_db_drop_is_always_caught():
    assert _flagged_fraction(4.0) == 1.0


def test_reference_without_n_falls_back_to_raw_scatter():
    """
    An old pointing.json has no metric_n, so the REFERENCE side of the noise
    uses its raw scatter; the check side, which knows its n, still uses the
    standard error. That puts the threshold near 3.2 dB here: a 3 dB drop is
    caught less often than with n recorded -- the safe direction -- and there
    are still no false alarms. (README's 3% is the old code, which used raw
    scatter on both sides.)
    """
    with_n = _flagged_fraction(3.0)
    without_n = _flagged_fraction(3.0, record_n=False)
    assert without_n < with_n
    assert _flagged_fraction(0.0, record_n=False) == 0.0


def test_fallback_threshold_is_raw_scatter_on_the_reference_only():
    ref = PointingReference(0.0, 12.0, 1.0, metric_n=None)
    verdict = check_pointing(ref, lambda: (11.0, 1.0, 10))
    assert verdict.threshold == pytest.approx(3.0 * math.hypot(1.0, 1.0 / math.sqrt(10)))


# --- verdicts, one at a time -----------------------------------------------

def _ref(mean=12.0, stdev=0.0, n=100, beamwidth=12.0):
    return PointingReference(2.7, mean, stdev, metric_n=n, beamwidth_deg=beamwidth)


@pytest.mark.parametrize("now_db, status", [
    (12.0, HealthVerdict.OK),
    (10.5, HealthVerdict.OK),            # 1.5 dB: inside the 2 dB floor
    (9.5, HealthVerdict.DEGRADED),       # 2.5 dB
    (7.5, HealthVerdict.FAILED),         # 4.5 dB
    (14.5, HealthVerdict.IMPROVED),
])
def test_verdict_against_the_fixed_floor(now_db, status):
    verdict = check_pointing(_ref(), lambda: (now_db, 0.0, 10))
    assert verdict.status == status
    assert verdict.needs_repeak == (status in ("degraded", "failed"))


def test_three_db_drop_implies_six_degrees_on_a_twelve_degree_beam():
    verdict = check_pointing(_ref(), lambda: (9.0, 0.0, 10))
    assert verdict.status == HealthVerdict.DEGRADED
    assert verdict.implied_error_deg == pytest.approx(6.0)


def test_noisy_readings_raise_the_threshold_above_the_floor():
    """With 3 dB per reading and only 2 of each, a 3 dB drop is not significant."""
    ref = _ref(stdev=3.0, n=2)
    verdict = check_pointing(ref, lambda: (9.0, 3.0, 2))
    expected = 3.0 * math.hypot(3.0 / math.sqrt(2), 3.0 / math.sqrt(2))
    assert verdict.threshold == pytest.approx(expected)
    assert verdict.status == HealthVerdict.OK


def test_no_reading_is_no_signal_not_a_drop():
    def silent():
        raise RuntimeError("nothing on the air")

    verdict = check_pointing(_ref(), silent)
    assert verdict.status == HealthVerdict.NO_SIGNAL
    assert not verdict.needs_repeak     # rain is not a reason to re-peak


def test_lower_is_better_metrics_are_flipped():
    """Viterbi-style counts fall as pointing improves: a rise is the drop."""
    ref = PointingReference(0.0, 100.0, 0.0, lower_is_better=True, metric_n=10)
    assert check_pointing(ref, lambda: (105.0, 0.0, 10)).status == HealthVerdict.FAILED
    assert check_pointing(ref, lambda: (95.0, 0.0, 10)).status == HealthVerdict.IMPROVED


# --- PointingReference -----------------------------------------------------

def test_reference_round_trips_through_a_dict():
    ref = PointingReference(2.5, 11.0, 0.9, "sdr-snr", False, "2026-10-27",
                            12.0, 100)
    assert PointingReference.from_dict(ref.to_dict()).to_dict() == ref.to_dict()


def test_old_reference_without_n_still_loads():
    old = {"angle_deg": 2.5, "metric_mean": 11.0, "metric_stdev": 0.9}
    ref = PointingReference.from_dict(old)
    assert ref.metric_n is None
    assert ref.lower_is_better is False
