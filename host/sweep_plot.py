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
    true_beam   (peak_deg, peak_db, beamwidth_deg) -- simulation only
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

    fig, ax = plt.subplots(figsize=(9, 5.5), dpi=110)

    if true_beam is not None:
        peak_deg, peak_db, bw = true_beam
        xs = np.linspace(lo, hi, 300)
        ax.plot(xs, peak_db - 12.0 * ((xs - peak_deg) / bw) ** 2, ls=":",
                color="0.55", lw=1.5, label="true beam (simulation)")
        ax.axvline(peak_deg, ls=":", color="0.55", lw=1.2)

    if got:
        a = np.array([g[0] for g in got])
        s = np.array([g[1] for g in got])
        # Bars are the standard error of each average -- what the fit and the
        # health check actually have to contend with -- not the raw scatter.
        se = np.array([(sd or 0.0) / math.sqrt(n) if n and n > 1 else 0.0
                       for (_a, _s, sd, n) in got])
        ax.errorbar(a, s, yerr=se, fmt="o", color="C0", ms=5, capsize=3,
                    label="sweep reading (±1 standard error)")

        if result.ok and len(got) >= 3:
            coef = np.polyfit(a, s, 2)
            xs = np.linspace(a.min(), a.max(), 300)
            ax.plot(xs, np.polyval(coef, xs), color="C1", lw=2,
                    label="fitted parabola")

    if missed:
        floor = min((g[1] for g in got), default=0.0) - 1.0
        ax.plot(missed, [floor] * len(missed), "x", color="C3", ms=8, mew=2,
                label="no reading")

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
    ax.set_xlim(lo - 0.5, hi + 0.5)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="lower center")
    fig.tight_layout()

    path = Path(path)
    fig.savefig(path)
    plt.close(fig)
    return path
