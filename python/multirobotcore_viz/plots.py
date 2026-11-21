"""Static benchmark plots and markdown tables in the shared style."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.axes import Axes

from multirobotcore_viz import style

MARKERS = ["o", "s", "^", "D", "v", "P", "X", "h"]


def line_chart(
    series: dict[str, tuple[Sequence[float], Sequence[float]]],
    xlabel: str,
    ylabel: str,
    title: str,
    out: Path,
    logx: bool = False,
    logy: bool = False,
    width_px: int = 760,
    height_px: int = 440,
) -> Path:
    """Writes a multi-series line chart. Series colours follow the fixed
    categorical order; the legend sits in one row under the title and every
    series is also direct-labelled at its last point (labels are spread
    vertically so they never overlap), so identity never depends on colour
    alone."""
    fig = style.new_figure(width_px, height_px)
    ax: Axes = fig.add_axes((0.1, 0.13, 0.66, 0.66))
    ends = []
    for k, (label, (xs, ys)) in enumerate(series.items()):
        color = style.CATEGORICAL[k % len(style.CATEGORICAL)]
        ax.plot(
            xs,
            ys,
            color=color,
            linewidth=2.0,
            marker=MARKERS[k % len(MARKERS)],
            markersize=6,
            markeredgecolor=style.SURFACE,
            markeredgewidth=1.0,
            label=label,
        )
        if len(xs):
            ends.append((label, xs[-1], ys[-1]))
    if logx:
        ax.set_xscale("log")
    if logy:
        ax.set_yscale("log")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, which="major", axis="y")
    fig.text(0.1, 0.94, title, fontsize=12, weight="bold", color=style.TEXT_PRIMARY)
    ax.legend(
        loc="lower left",
        bbox_to_anchor=(0.0, 1.02),
        ncol=min(len(series), 4),
        frameon=False,
        borderaxespad=0.0,
        handlelength=1.6,
    )
    # Direct labels at the right edge of the axes, spread to a minimum gap.
    fig.canvas.draw()
    to_px = ax.transData.transform
    placed = sorted(((to_px((x, y))[1], label) for label, x, y in ends), reverse=True)
    min_gap = 13.0
    adjusted: list[tuple[float, str]] = []
    for y_raw, label in placed:
        y_px = min(y_raw, adjusted[-1][0] - min_gap) if adjusted else y_raw
        adjusted.append((y_px, label))
    x_px = ax.transAxes.transform((1.0, 0.0))[0] + 8
    for y_px, label in adjusted:
        fx, fy = fig.transFigure.inverted().transform((x_px, y_px))
        fig.text(fx, fy, label, fontsize=8.5, color=style.TEXT_SECONDARY, va="center")
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=style.DPI)
    plt.close(fig)
    return out


def markdown_table(headers: Sequence[str], rows: Sequence[Sequence[object]]) -> str:
    """GitHub-flavoured markdown table."""
    lines = ["| " + " | ".join(headers) + " |", "|" + "|".join("---" for _ in headers) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(str(c) for c in row) + " |")
    return "\n".join(lines)


def fmt(value: float, digits: int = 3) -> str:
    """Compact number formatting for tables."""
    if value is None:
        return "-"
    if abs(value) >= 1000:
        return f"{value:,.0f}"
    return f"{value:.{digits}g}" if abs(value) < 1 else f"{value:.{digits}f}".rstrip("0").rstrip(".")
