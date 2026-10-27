"""
test_iq_metric.py
-----------------
The SNR estimators and SdrSnrMetric, against synthetic IQ of known SNR. The
RTL-SDR capture is replaced, so this says nothing about the real dongle --
only that, given the samples, the number that comes out is the right one.

The rule that matters most: on pure noise an estimator must REFUSE, not
return a number. A search treats -24 dB as better than -25 dB and walks the
dish uphill on noise.

    pytest test_iq_metric.py
"""

import cmath
import math
import random

import pytest

import iq_metric
from iq_metric import IqMetricError, SdrSnrMetric
from iq_snr import SnrError, snr_m2m4, snr_psd

np = pytest.importorskip("numpy")

FS = 2_400_000.0
BW = 600_000.0


def qpsk_in_noise(snr_db, n, seed):
    """Constant-modulus QPSK plus complex Gaussian noise of unit power."""
    rng = random.Random(seed)
    amp = math.sqrt(10 ** (snr_db / 10))
    out = []
    for _ in range(n):
        sym = amp * cmath.exp(1j * (math.pi / 4 + math.pi / 2 * rng.randrange(4)))
        noise = complex(rng.gauss(0, 1), rng.gauss(0, 1)) / math.sqrt(2)
        out.append(sym + noise)
    return out


def band_limited_in_noise(snr_db, n, seed):
    """
    A signal flat across +/-BW/2 plus white noise of unit power across FS.
    In-band noise power is BW/FS, so SNR in the signal band is S/(BW/FS).
    """
    rng = np.random.default_rng(seed)
    freqs = np.fft.fftfreq(n, d=1 / FS)
    spec = np.zeros(n, dtype=complex)
    band = np.abs(freqs) < BW / 2
    spec[band] = np.exp(2j * np.pi * rng.random(band.sum()))
    sig = np.fft.ifft(spec)
    s_power = 10 ** (snr_db / 10) * (BW / FS)
    sig *= math.sqrt(s_power / np.mean(np.abs(sig) ** 2))
    noise = (rng.standard_normal(n) + 1j * rng.standard_normal(n)) / math.sqrt(2)
    return sig + noise


def pure_noise(n, seed):
    rng = np.random.default_rng(seed)
    return (rng.standard_normal(n) + 1j * rng.standard_normal(n)) / math.sqrt(2)


# --- snr_m2m4 --------------------------------------------------------------

@pytest.mark.parametrize("snr_db", [3.0, 10.0, 20.0])
def test_m2m4_reads_known_snr(snr_db):
    got = snr_m2m4(qpsk_in_noise(snr_db, 1 << 15, seed=int(snr_db)))
    assert got == pytest.approx(snr_db, abs=0.5)


@pytest.mark.parametrize("seed", range(20))
def test_m2m4_refuses_pure_noise(seed):
    with pytest.raises(SnrError, match="no signal"):
        snr_m2m4(pure_noise(16384, seed))


def test_m2m4_needs_samples():
    with pytest.raises(SnrError):
        snr_m2m4([1 + 0j])


# --- snr_psd ---------------------------------------------------------------

@pytest.mark.parametrize("snr_db", [0.0, 10.0, 20.0])
def test_psd_reads_known_snr(snr_db):
    got = snr_psd(band_limited_in_noise(snr_db, 1 << 18, seed=int(snr_db)), FS, BW)
    assert got == pytest.approx(snr_db, abs=1.0)


@pytest.mark.xfail(strict=True, reason=(
    "KNOWN BUG: snr_psd has no significance gate. On pure noise the in-band "
    "density lands above the out-of-band one about half the time, and it "
    "returns a number between roughly -45 and -19 dB instead of refusing. "
    "snr_m2m4 guards against exactly this; snr_psd needs the same."))
def test_psd_refuses_pure_noise():
    numbers = []
    for seed in range(20):
        try:
            numbers.append(snr_psd(pure_noise(1 << 18, seed), FS, BW))
        except SnrError:
            pass
    assert numbers == [], f"returned {len(numbers)}/20 numbers on pure noise"


def test_psd_needs_noise_bins_outside_the_band():
    with pytest.raises(SnrError, match="outside the signal band"):
        snr_psd(pure_noise(1 << 14, 0), FS, signal_bw=FS)


# --- SdrSnrMetric ----------------------------------------------------------

@pytest.fixture
def fake_capture(monkeypatch):
    """Replace the RTL-SDR with a list of blocks, served in turn."""
    def use(blocks):
        it = iter(blocks * 1000)
        calls = []

        def capture(freq_hz, fs, n_samples, gain, device_index):
            calls.append(freq_hz)
            return next(it)
        monkeypatch.setattr(iq_metric, "capture_rtlsdr", capture)
        return calls
    return use


def test_zero_dwell_still_takes_one_reading(fake_capture):
    calls = fake_capture([qpsk_in_noise(10.0, 8192, seed=1)])
    mean, stdev, n = SdrSnrMetric(1.6941e9).sample(0.0)
    assert n == 1 and len(calls) == 1
    assert stdev == 0.0
    assert mean == pytest.approx(10.0, abs=1.0)


def test_dwell_averages_several_blocks(fake_capture):
    blocks = [qpsk_in_noise(10.0, 8192, seed=s) for s in range(4)]
    fake_capture(blocks)
    mean, stdev, n = SdrSnrMetric(1.6941e9).sample(0.3)
    assert n > 1
    assert stdev > 0.0
    assert mean == pytest.approx(10.0, abs=1.0)


def test_noise_only_dwell_raises_instead_of_averaging_nothing(fake_capture):
    fake_capture([list(pure_noise(8192, s)) for s in range(4)])
    with pytest.raises(IqMetricError, match="no usable SNR reading"):
        SdrSnrMetric(1.6941e9).sample(0.1)


def test_one_good_block_among_noise_is_still_a_reading(fake_capture):
    """Blocks that refuse are skipped, not averaged in as zeros."""
    blocks = [list(pure_noise(8192, 0)), qpsk_in_noise(10.0, 8192, seed=1)]
    calls = fake_capture(blocks)
    mean, _sd, n = SdrSnrMetric(1.6941e9).sample(0.2)
    assert len(calls) >= 2
    assert n == len(calls) // 2         # only the QPSK blocks counted
    assert mean == pytest.approx(10.0, abs=1.0)


@pytest.mark.parametrize("estimator, signal_bw, psd", [
    ("auto", None, False),       # no bandwidth: never guess one
    ("auto", BW, True),
    ("m2m4", BW, False),
    ("psd", BW, True),
])
def test_estimator_choice(estimator, signal_bw, psd):
    metric = SdrSnrMetric(1.6941e9, signal_bw=signal_bw, estimator=estimator)
    assert metric._use_psd() is psd


def test_unknown_estimator_is_refused():
    with pytest.raises(ValueError):
        SdrSnrMetric(1.6941e9, estimator="magic")
