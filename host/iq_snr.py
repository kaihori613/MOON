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
free, then hand it over.

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

__all__ = ["snr_m2m4", "snr_psd", "SnrError", "to_db"]


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

def _welch(iq, fs, nfft, overlap=0.5):
    """Averaged periodogram, returned as (freqs, psd) with DC in the middle."""
    import numpy as np

    x = np.asarray(iq, dtype=np.complex128)
    if x.size < nfft:
        raise SnrError(f"need at least nfft={nfft} samples, got {x.size}")

    step = max(1, int(nfft * (1.0 - overlap)))
    win = np.hanning(nfft)
    # Normalise so the estimate is a power spectral density rather than
    # something that changes when the window or the length changes.
    scale = 1.0 / (fs * (win * win).sum())

    acc = np.zeros(nfft)
    count = 0
    for start in range(0, x.size - nfft + 1, step):
        seg = x[start:start + nfft] * win
        acc += np.abs(np.fft.fft(seg)) ** 2
        count += 1
    if count == 0:
        raise SnrError("no complete segments")

    psd = np.fft.fftshift(acc / count) * scale
    freqs = np.fft.fftshift(np.fft.fftfreq(nfft, d=1.0 / fs))
    return freqs, psd


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
    """
    import numpy as np

    if guard is None:
        guard = signal_bw * 0.5

    freqs, psd = _welch(iq, fs, nfft)
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

    snr_db = to_db(ratio)
    if not return_detail:
        return snr_db
    return snr_db, {
        "density_in": d_in,
        "density_out": d_out,
        "bins_in": int(in_band.sum()),
        "bins_out": int(out_band.sum()),
        "bin_hz": fs / nfft,
        # C/N0 falls out of the same numbers and is the bandwidth-independent
        # way to compare two receivers or two dishes.
        "cn0_dbhz": snr_db + to_db(signal_bw),
    }


# ---------------------------------------------------------------------------
#  Capture
# ---------------------------------------------------------------------------

def capture_rtlsdr(freq_hz, fs=2_400_000, n_samples=1 << 18, gain="auto",
                   device_index=0):
    """
    Grab a block of IQ from an RTL-SDR.

    NOT RUN. There is no SDR on the machine this was written on, so unlike
    the estimators above -- which are checked against synthetic signals of
    known SNR -- this function has never moved a sample. Treat it as a
    sketch of the call sequence.

    Requires pyrtlsdr. Remember that the device is single-client: this will
    fail while SatDump holds it.
    """
    try:
        from rtlsdr import RtlSdr
    except ImportError as exc:
        raise SnrError(
            "pyrtlsdr is not installed (pip install pyrtlsdr), and it needs "
            "librtlsdr present as well.") from exc

    sdr = RtlSdr(device_index)
    try:
        sdr.sample_rate = fs
        sdr.center_freq = freq_hz
        sdr.gain = gain
        # The first samples after retune are not trustworthy; the tuner PLL
        # and the AGC both need a moment to settle.
        sdr.read_samples(16384)
        return sdr.read_samples(n_samples)
    finally:
        sdr.close()
