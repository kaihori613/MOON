"""
iq_snr.py
---------
SNR estimated straight from IQ samples, with no decoder in the path.

WHY THIS EXISTS ALONGSIDE metric.py

metric.py and satdump.py read a number the decoder already computed. That is
the better number when you can get it -- a demodulator knows things about the
signal that a spectrum does not -- but it has two limits that matter for
pointing:

  * it only exists once the decoder has LOCK, and lock is exactly what you do
    not have when the dish is mispointed. A decoder metric cannot help you
    find the bird, only refine an aim you already roughly have.
  * it only exists while the decoder is running.

Estimating from IQ has neither limit. It works far below lock threshold, so
it can drive a coarse search, and it needs nothing but samples.

BER IS NOT AVAILABLE THIS WAY, AND CANNOT BE

Bit error rate is a decoder output. Measuring it means knowing what the
transmitted bits should have been, which means demodulating, synchronising
and running the FEC -- at which point you have written a decoder and should
have used SatDump. What you can get from raw IQ is SNR or C/N0. If you want
BER or Viterbi corrections, take them from SatDump (satdump.py). There is no
shortcut, and anything claiming to be "BER from the spectrum" is estimating
SNR and converting through an assumed modulation and code rate.

THE SDR IS A SINGLE-CLIENT DEVICE

An RTL-SDR can be opened by one process at a time. While SatDump holds it,
nothing here can read samples, so these two metric paths are alternatives in
time, not in parallel:

    before SatDump starts   ->  iq_snr, to acquire and coarse-peak
    while SatDump runs      ->  satdump.py, to refine on the decoder metric

which happens to fit a scheduled recording exactly: peak while the SDR is
free, then hand it over -- which means closing it first. RtlSdrCapture holds
the device open across many reads, so whatever holds one must close it.

TWO ESTIMATORS

snr_m2m4()  -- second and fourth moments. Pure Python, no dependency at all,
               no FFT, no need to know where the signal sits in the band. It
               assumes a constant-modulus signal (PSK) in Gaussian noise.
snr_psd()   -- in-band power against out-of-band power. Needs numpy. Slower,
               but it does not care about the modulation, it is easy to sanity
               check by eye against a spectrum plot, and it rejects
               interference outside the signal band instead of counting it.

Prefer snr_psd() when numpy is available. Use snr_m2m4() when it is not, or
as a cross-check -- two estimators disagreeing is itself informative.
"""

from __future__ import annotations

import math

__all__ = ["snr_m2m4", "snr_psd", "SnrError", "to_db", "RtlSdrCapture",
           "capture_rtlsdr"]


class SnrError(RuntimeError):
    pass


def to_db(linear: float) -> float:
    if linear <= 0.0:
        return float("-inf")
    return 10.0 * math.log10(linear)


# ---------------------------------------------------------------------------
#  Moment estimator -- no dependencies
# ---------------------------------------------------------------------------

def snr_m2m4(iq) -> float:
    """
    SNR in dB from the 2nd and 4th moments of complex baseband samples.

    For a constant-modulus signal of power S in complex Gaussian noise of
    power N:

        M2 = S + N
        M4 = S^2 + 4*S*N + 2*N^2

    which solves without knowing either separately:

        S = sqrt(2*M2^2 - M4)
        N = M2 - S

    That is the whole estimator. It needs no FFT, no idea where in the band
    the signal is, and no noise-only reference -- which is why it fits on a
    machine with nothing installed.

    LIMITS, because they are real:
      * It assumes constant modulus. True for PSK at the symbol instants,
        less true once root-raised-cosine shaping has been applied, so
        expect a bias on a filtered signal.
      * It degrades at low SNR, where 2*M2^2 - M4 approaches zero and the
        square root amplifies the estimate's own noise.
      * It counts EVERYTHING in the captured bandwidth as signal, including
        an adjacent carrier or a birdie. Tune tight.
      * It has a hard noise floor that scales as n^-1/4, so a short block
        simply cannot resolve a weak signal. The estimator refuses rather
        than reporting the small negative number it would otherwise produce
        from pure noise; see the gate in the code for why that matters more
        than it sounds.

    Raises SnrError when the moments are inconsistent with the model, which
    is the honest outcome -- it means the input is not a constant-modulus
    signal in Gaussian noise, and returning a number anyway would be a
    fabrication the search would then act on.
    """
    n = 0
    m2 = 0.0
    m4 = 0.0
    for s in iq:
        p = (s.real * s.real + s.imag * s.imag) if isinstance(s, complex) \
            else float(abs(s)) ** 2
        m2 += p
        m4 += p * p
        n += 1

    if n < 2:
        raise SnrError("need at least 2 samples")

    m2 /= n
    m4 /= n

    disc = 2.0 * m2 * m2 - m4

    # On PURE noise the discriminant is zero in expectation, not negative --
    # complex Gaussian noise has kurtosis exactly 2, so M4 = 2*M2^2. With a
    # finite block it lands either side of zero at random, and when it lands
    # positive this estimator will happily report something like -10 dB.
    #
    # That is the dangerous failure for a search, because it is not an error,
    # it is a NUMBER, and a search will cheerfully decide -10 dB beats -11 dB
    # and walk the dish uphill on noise. So require the discriminant to be
    # significantly above what noise alone produces before believing it.
    #
    # For pure noise |r|^2 is exponential, so var(M4_hat) = 20*M2^4/n and the
    # discriminant's own standard deviation is sqrt(20/n)*M2^2. Three of those
    # is the gate.
    noise_sigma = math.sqrt(20.0 / n) * m2 * m2
    if disc <= 3.0 * noise_sigma:
        floor_db = to_db(math.sqrt(3.0 * noise_sigma) / m2) if m2 > 0 else 0.0
        raise SnrError(
            "no signal distinguishable from noise: 2*M2^2 - M4 is within "
            f"three sigma of what {n} samples of pure noise give. This block "
            f"cannot resolve better than about {floor_db:.1f} dB -- capture "
            "more samples to push that down, or accept that there is nothing "
            "here to point at.")

    s_pow = math.sqrt(disc)
    n_pow = m2 - s_pow
    if n_pow <= 0.0:
        raise SnrError(
            "estimated noise power is not positive -- the capture looks "
            "noiseless, which in practice means it saturated or the estimator "
            "is out of its range.")
    return to_db(s_pow / n_pow)


# ---------------------------------------------------------------------------
#  Spectral estimator -- needs numpy
# ---------------------------------------------------------------------------

# snr_psd's noise gate: how many groups the capture is split into in time,
# and how many standard errors the in-band excess must clear.
_GATE_GROUPS = 16
_GATE_SIGMAS = 3.0


def _welch(iq, fs, nfft, overlap=0.5):
    """
    Welch periodograms, returned as (freqs, segments) with DC in the middle:
    one row per windowed segment, in time order. The averaged PSD is
    segments.mean(axis=0); the rows are kept so snr_psd can see how much
    that average would wander.
    """
    import numpy as np

    x = np.asarray(iq, dtype=np.complex128)
    if x.size < nfft:
        raise SnrError(f"need at least nfft={nfft} samples, got {x.size}")

    step = max(1, int(nfft * (1.0 - overlap)))
    win = np.hanning(nfft)
    # Normalise so the estimate is a power spectral density rather than
    # something that changes when the window or the length changes.
    scale = 1.0 / (fs * (win * win).sum())

    starts = range(0, x.size - nfft + 1, step)
    if len(starts) == 0:
        raise SnrError("no complete segments")
    segs = np.array([np.abs(np.fft.fft(x[s:s + nfft] * win)) ** 2 for s in starts])

    segs = np.fft.fftshift(segs, axes=1) * scale
    freqs = np.fft.fftshift(np.fft.fftfreq(nfft, d=1.0 / fs))
    return freqs, segs


def snr_psd(iq, fs, signal_bw, center=0.0, guard=None, nfft=4096,
            return_detail=False):
    """
    SNR in dB from in-band power density against out-of-band power density.

    The signal band carries signal PLUS noise, so the noise density measured
    outside is subtracted before the ratio is taken. Skipping that subtraction
    is the classic error and it compresses the result badly at low SNR:

        SNR = (D_in - D_out) / D_out

    where D is power per hertz. Because both are densities, the answer is the
    SNR in the signal bandwidth regardless of how wide the capture was.

    Arguments:
      fs          sample rate, Hz
      signal_bw   occupied bandwidth of the signal, Hz
      center      where the signal sits relative to baseband DC, Hz
      guard       gap left either side of the signal band before noise is
                  measured, so the pulse-shaping skirts are not counted as
                  noise. Defaults to half the signal bandwidth.

    With return_detail, also returns the densities and the bin counts, which
    is what you want when the number looks wrong -- almost always because the
    noise window landed on another signal.

    Raises SnrError when the in-band excess is not significantly above what
    noise alone produces -- the same rule snr_m2m4 applies, for the same
    reason. See the gate below.
    """
    import numpy as np

    if guard is None:
        guard = signal_bw * 0.5

    freqs, segs = _welch(iq, fs, nfft)
    psd = segs.mean(axis=0)
    off = freqs - center

    in_band = np.abs(off) <= (signal_bw * 0.5)
    out_band = np.abs(off) >= (signal_bw * 0.5 + guard)

    if in_band.sum() < 4:
        raise SnrError(
            f"only {int(in_band.sum())} bins inside the signal band. "
            "Raise nfft, or signal_bw is smaller than the bin spacing "
            f"({fs / nfft:.1f} Hz).")
    if out_band.sum() < 4:
        raise SnrError(
            "not enough bins outside the signal band to estimate noise. "
            "The capture bandwidth needs to be wider than signal_bw + 2*guard.")

    d_in = float(np.mean(psd[in_band]))
    d_out = float(np.mean(psd[out_band]))

    if d_out <= 0.0:
        raise SnrError("measured noise density is zero")

    ratio = (d_in - d_out) / d_out
    if ratio <= 0.0:
        raise SnrError(
            "in-band density is at or below the out-of-band density: there is "
            "no signal here above the noise. For a search this is the correct "
            "answer, not a failure -- the dish is not pointed at it.")

    # On PURE noise the in-band density is above the out-of-band one half the
    # time, by chance, and the line above lets every one of those through as
    # a number around -25 dB. That is snr_m2m4's dangerous failure exactly: a
    # search decides -24 dB beats -25 dB and walks the dish uphill on noise.
    #
    # So ask whether the excess is bigger than its own scatter. Split the
    # segments into consecutive groups in time, measure the excess in each,
    # and require the mean to sit three standard errors above zero. Groups
    # far apart in time have independent noise, which is what makes their
    # scatter honest; measuring it across neighbouring frequency bins instead
    # would not be, because a Hann window correlates adjacent bins.
    n_groups = min(_GATE_GROUPS, len(segs))
    if n_groups < 4:
        raise SnrError(
            f"only {len(segs)} FFT segments -- too few to tell signal from "
            f"noise. Capture at least {4 * nfft} samples.")
    excess = []
    for g in np.array_split(segs, n_groups):
        g_psd = g.mean(axis=0)
        excess.append(float(np.mean(g_psd[in_band]) / np.mean(g_psd[out_band])) - 1.0)
    excess = np.array(excess)
    stderr = float(excess.std(ddof=1)) / math.sqrt(n_groups)
    significance = float(excess.mean()) / stderr if stderr > 0 else math.inf
    if significance <= _GATE_SIGMAS:
        raise SnrError(
            f"no signal distinguishable from noise: the in-band excess is "
            f"{significance:.1f} standard errors above zero, short of "
            f"{_GATE_SIGMAS:g}. Capture more samples to push the floor down, "
            "or accept that there is nothing here to point at.")

    snr_db = to_db(ratio)
    if not return_detail:
        return snr_db
    return snr_db, {
        "density_in": d_in,
        "density_out": d_out,
        "bins_in": int(in_band.sum()),
        "bins_out": int(out_band.sum()),
        "bin_hz": fs / nfft,
        "significance": significance,
        # C/N0 falls out of the same numbers and is the bandwidth-independent
        # way to compare two receivers or two dishes.
        "cn0_dbhz": snr_db + to_db(signal_bw),
    }


# ---------------------------------------------------------------------------
#  Capture
# ---------------------------------------------------------------------------

# Thrown away after tuning, and by flush(): about 7 ms at 2.4 MS/s.
SETTLE_SAMPLES = 16384


class RtlSdrCapture:
    """
    An RTL-SDR opened and tuned once, then read a block at a time.

        with RtlSdrCapture(1694.1e6) as sdr:
            for _ in range(10):
                iq = sdr.read(1 << 18)

    Opening is the expensive part -- USB setup, tuner init, the PLL locking
    on the new frequency -- so a caller reading many blocks should hold one
    of these rather than call capture_rtlsdr() per block.

    NOT RUN. There is no SDR on the machine this was written on, so unlike
    the estimators above -- which are checked against synthetic signals of
    known SNR -- this has never moved a sample. The tests drive it against a
    fake RtlSdr, which checks the call sequence and nothing about the dongle.

    Requires pyrtlsdr. The device is single-client: while one of these is
    open, nothing else -- SatDump, SDR#, the next run of this -- can open the
    dongle. Use it in a `with`, or close() it in a finally.
    """

    def __init__(self, freq_hz, fs=2_400_000, gain="auto", device_index=0):
        try:
            from rtlsdr import RtlSdr
        except ImportError as exc:
            raise SnrError(
                "pyrtlsdr is not installed (pip install pyrtlsdr), and it needs "
                "librtlsdr present as well.") from exc

        self._sdr = RtlSdr(device_index)
        try:
            self._sdr.sample_rate = fs
            self._sdr.center_freq = freq_hz
            self._sdr.gain = gain
            # The first samples after retune are not trustworthy; the tuner
            # PLL and the AGC both need a moment to settle.
            self.flush()
        except BaseException:
            self.close()
            raise

    def read(self, n_samples):
        if self._sdr is None:
            raise ValueError("read from a closed RtlSdrCapture")
        return self._sdr.read_samples(n_samples)

    def flush(self):
        """
        Throw away SETTLE_SAMPLES. Whatever the dongle buffered while nobody
        was reading -- during a dish move, say -- comes out first otherwise.
        """
        self.read(SETTLE_SAMPLES)

    def close(self):
        """Release the dongle. Safe to call twice."""
        if self._sdr is not None:
            sdr, self._sdr = self._sdr, None
            sdr.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def capture_rtlsdr(freq_hz, fs=2_400_000, n_samples=1 << 18, gain="auto",
                   device_index=0):
    """
    Grab one block of IQ from an RTL-SDR: open, tune, read, close.

    For a single block -- a check that samples flow at all. Anything that
    reads block after block should hold an RtlSdrCapture open instead; this
    pays the full open and settle every call. NOT RUN on hardware either,
    for the reason given there.
    """
    with RtlSdrCapture(freq_hz, fs=fs, gain=gain,
                       device_index=device_index) as sdr:
        return sdr.read(n_samples)
