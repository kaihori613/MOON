"""
sweep_plot.py
-------------
Draw a snr_peak.py sweep as a picture: SNR against angle, the fitted curve,
and where the dish was told to park.

The printed table says the same thing, but a hill is easier to judge by eye
than by column -- in particular whether the flanks actually fall off (the fit
needs them to) and whether one wild point is dragging the vertex.

Needs matplotlib (pip install matplotlib). Imported only when --plot is
asked for, so nothing else in host/ depends on it.
"""

from __future__ import annotations

import math
from pathlib import Path


def plot_sweep(path: Path, result, lo, hi, ref=None, verdict=None,
               title="", true_beam=None) -> Path:
    """
    Write the sweep to `path` as a PNG and return the path.

    result      peak.PeakResult; its samples are (angle, score, stdev, n)
    ref         PointingReference from the sweep, or None if it found no peak
    verdict     HealthVerdict from a --check right after, if there was one
    true_beam   (peak_deg, true_db) where true_db(angle) is the beam's real
                SNR there -- simulation only
    """
    try:
        import matplotlib
        matplotlib.use("Agg")        # write a file; never open a window that blocks
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError as exc:
        raise RuntimeError(
            "--plot needs matplotlib: pip install matplotlib") from exc

    got = [(a, s, sd, n) for (a, s, sd, n) in result.samples if s is not None]
    missed = [a for (a, s, _sd, _n) in result.samples if s is None]
    coarse = set(result.coarse.samples) if result.coarse is not None else set()

    # The refining pass may run past LO..HI, so the axis follows the readings.
    angles = [s[0] for s in result.samples] + [lo, hi]
    x_lo, x_hi = min(angles), max(angles)

    fig, ax = plt.subplots(figsize=(9, 5.5), dpi=110)

    if true_beam is not None:
        peak_deg, true_db = true_beam
        xs = np.linspace(x_lo, x_hi, 300)
        ys = np.array([true_db(x) for x in xs])
        # An Airy beam's nulls go to -100 dB; draw only the part near the data.
        floor = min((g[1] for g in got), default=ys.max() - 20.0) - 3.0
        ys[ys < floor] = np.nan
        ax.plot(xs, ys, ls=":", color="0.55", lw=1.5,
                label="true beam (simulation)")
        ax.axvline(peak_deg, ls=":", color="0.55", lw=1.2)

    if got:
        a = np.array([g[0] for g in got])
        s = np.array([g[1] for g in got])
        # Bars are the standard error of each average -- what the fit and the
        # health check actually have to contend with -- not the raw scatter.
        se = np.array([(sd or 0.0) / math.sqrt(n) if n and n > 1 else 0.0
                       for (_a, _s, sd, n) in got])
        first = np.array([g in coarse for g in got], dtype=bool)
        if first.any():
            ax.errorbar(a[first], s[first], yerr=se[first], fmt="o", mfc="none",
                        color="C0", ms=6, capsize=3,
                        label="coarse pass (±1 standard error)")
        if (~first).any():
            ax.errorbar(a[~first], s[~first], yerr=se[~first], fmt="o",
                        color="C0", ms=4, capsize=2,
                        label="refining pass" if first.any()
                        else "sweep reading (±1 standard error)")

        if result.ok and len(got) >= 3:
            coef = np.polyfit(a, s, 2)
            xs = np.linspace(a.min(), a.max(), 300)
            ax.plot(xs, np.polyval(coef, xs), color="C1", lw=2,
                    label="fitted parabola")

    if missed:
        floor = min((g[1] for g in got), default=0.0) - 1.0
        ax.plot(missed, [floor] * len(missed), "x", color="C3", ms=8, mew=2,
                label="no reading")

    if result.coarse is not None and result.coarse.ok:
        ax.axvline(result.coarse.best_angle, color="0.4", ls="-.", lw=1,
                   label=f"coarse peak {result.coarse.best_angle:+.2f}°")
    if result.ok:
        ax.axvline(result.best_angle, color="C1", ls="--", lw=1.2,
                   label=f"fitted peak {result.best_angle:+.2f}°")
    if ref is not None:
        ax.axvline(ref.angle, color="C2", lw=1.5,
                   label=f"parked {ref.angle:+.2f}°")
        se = (ref.metric_stdev / math.sqrt(ref.metric_n)
              if ref.metric_n and ref.metric_n > 1 else 0.0)
        ax.errorbar([ref.angle], [ref.metric_mean], yerr=[se], fmt="*",
                    color="C2", ms=14, capsize=3,
                    label=f"reference {ref.metric_mean:.2f} dB")
    if verdict is not None and verdict.now is not None and ref is not None:
        ax.plot([ref.angle], [verdict.now], "D", color="C4", ms=8,
                label=f"check: {verdict.status} ({verdict.now:.2f} dB)")

    status = "peak found" if result.ok else f"NO PEAK: {result.reason}"
    if true_beam is not None and ref is not None:
        status += f"  —  parked {ref.angle - true_beam[0]:+.2f}° from the true peak"
    ax.set_title(f"{title}\n{status}" if title else status, fontsize=10)
    ax.set_xlabel("actuator angle (deg)")
    ax.set_ylabel("SNR (dB)")
    ax.set_xlim(x_lo - 0.5, x_hi + 0.5)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="lower center")
    fig.tight_layout()

    path = Path(path)
    fig.savefig(path)
    plt.close(fig)
    return path


def plot_trials(path: Path, fitted_err, parked_err, title="") -> Path:
    """
    Write a Monte Carlo run as a PNG: how far the fitted peak and the parked
    dish landed from the true peak, signed, over every trial that found one.

    Signed on purpose. Noise spreads the histogram symmetrically around zero;
    a model error -- fitting a parabola to one flank of a beam that is not
    one -- SHIFTS it, and the shift is what --trials exists to show.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError as exc:
        raise RuntimeError(
            "--plot needs matplotlib: pip install matplotlib") from exc

    fitted = np.asarray(fitted_err, dtype=float)
    parked = np.asarray(parked_err, dtype=float)
    span = max(0.3, float(np.max(np.abs(np.concatenate([fitted, parked])))) * 1.05)
    bins = np.linspace(-span, span, 41)

    fig, ax = plt.subplots(figsize=(9, 5.5), dpi=110)
    ax.hist(parked, bins=bins, color="C2", alpha=0.45,
            label=f"parked (median |error| {np.median(np.abs(parked)):.3f}°)")
    ax.hist(fitted, bins=bins, color="C1", alpha=0.75,
            label=f"fitted peak (median |error| {np.median(np.abs(fitted)):.3f}°)")
    ax.axvline(0.0, color="k", lw=1)
    ax.axvline(float(fitted.mean()), color="C1", ls="--", lw=1.5,
               label=f"fitted mean {fitted.mean():+.3f}° (the bias)")
    ax.set_xlabel("error from the true peak (deg)  —  signed: + means too far +")
    ax.set_ylabel("trials")
    ax.set_title(title, fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()

    path = Path(path)
    fig.savefig(path)
    plt.close(fig)
    return path
