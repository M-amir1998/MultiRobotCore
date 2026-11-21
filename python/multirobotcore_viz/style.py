"""Shared visual style for every animation and plot.

Colours follow a validated categorical palette (fixed order, colour-vision
deficiency checked) on a light scientific surface. Colour encodes a role or
state (leader, follower, failed, conflict); robot identity is carried by
labels when teams are larger than a handful of robots.
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")  # headless, no GPU or display required

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure

# Surfaces and ink
SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e8e7e3"
FRAME = "#c9c8c2"
LINK = "#b4b3ad"  # communication links: recessive
OBSTACLE_FILL = "#dcdbd6"
OBSTACLE_EDGE = "#a3a29c"
FREE_CELL = "#f4f3ef"
UNKNOWN = "#cfcec9"

# Categorical slots in fixed order (never cycled).
BLUE = "#2a78d6"
ORANGE = "#eb6834"
AQUA = "#1baf7a"
YELLOW = "#eda100"
MAGENTA = "#e87ba4"
GREEN = "#008300"
VIOLET = "#6250d6"
RED = "#e34948"
CATEGORICAL = [BLUE, ORANGE, AQUA, YELLOW, MAGENTA, GREEN, VIOLET, RED]

# Single-hue sequential ramp (blue, light -> dark) for magnitudes.
SEQUENTIAL_BLUE = [
    "#cde2fb",
    "#b7d3f6",
    "#9ec5f4",
    "#86b6ef",
    "#6da7ec",
    "#5598e7",
    "#3987e5",
    "#2a78d6",
    "#256abf",
    "#1c5cab",
    "#184f95",
    "#104281",
    "#0d366b",
]

# Status colours are reserved for states and always paired with a label.
STATUS_GOOD = "#0ca30c"
STATUS_WARNING = "#fab219"
STATUS_SERIOUS = "#ec835a"
STATUS_CRITICAL = "#d03b3b"

# Roles
ROBOT = BLUE
LEADER = ORANGE
FAILED = "#a3a29c"

DPI = 100


def apply_style() -> None:
    """Configures matplotlib globally for the MultiRobotCore look."""
    plt.rcParams.update(
        {
            "figure.facecolor": SURFACE,
            "axes.facecolor": SURFACE,
            "savefig.facecolor": SURFACE,
            "axes.edgecolor": FRAME,
            "axes.labelcolor": TEXT_SECONDARY,
            "axes.titlecolor": TEXT_PRIMARY,
            "axes.titlesize": 12,
            "axes.titleweight": "bold",
            "axes.labelsize": 10,
            "axes.linewidth": 0.8,
            "axes.grid": False,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "grid.color": GRID,
            "grid.linewidth": 0.6,
            "xtick.color": TEXT_SECONDARY,
            "ytick.color": TEXT_SECONDARY,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "legend.frameon": False,
            "legend.fontsize": 9,
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "text.color": TEXT_PRIMARY,
            "lines.linewidth": 2.0,
            "lines.solid_capstyle": "round",
        }
    )


def new_figure(width_px: int = 960, height_px: int = 540) -> Figure:
    """Creates a figure of an exact pixel size."""
    apply_style()
    return plt.figure(figsize=(width_px / DPI, height_px / DPI), dpi=DPI)


def world_axes(
    fig: Figure,
    rect: tuple[float, float, float, float],
    xlim: tuple[float, float],
    ylim: tuple[float, float],
    title: str | None = None,
    frame: bool = True,
) -> Axes:
    """Equal-aspect axes for a planar world, without tick clutter."""
    ax = fig.add_axes(rect)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_visible(frame)
        ax.spines[side].set_color(FRAME)
    if title:
        ax.set_title(title, loc="left", pad=6)
    return ax


def figure_title(fig: Figure, title: str, subtitle: str | None = None) -> None:
    """Top-left title block shared by all animations."""
    fig.text(0.02, 0.955, title, ha="left", va="center", fontsize=14, weight="bold", color=TEXT_PRIMARY)
    if subtitle:
        fig.text(0.02, 0.91, subtitle, ha="left", va="center", fontsize=10, color=TEXT_SECONDARY)


def sequential_color(value: float, vmin: float, vmax: float) -> str:
    """Maps a value onto the sequential blue ramp (clamped)."""
    if vmax <= vmin:
        return SEQUENTIAL_BLUE[len(SEQUENTIAL_BLUE) // 2]
    t = min(max((value - vmin) / (vmax - vmin), 0.0), 1.0)
    # Skip the lightest steps so that markers stay visible on the surface.
    lo = 3
    index = lo + round(t * (len(SEQUENTIAL_BLUE) - 1 - lo))
    return SEQUENTIAL_BLUE[index]


# Colours that must survive GIF quantisation exactly.
PALETTE_SEED = [
    SURFACE,
    TEXT_PRIMARY,
    TEXT_SECONDARY,
    GRID,
    FRAME,
    LINK,
    OBSTACLE_FILL,
    OBSTACLE_EDGE,
    FREE_CELL,
    UNKNOWN,
    *CATEGORICAL,
    *SEQUENTIAL_BLUE,
    STATUS_GOOD,
    STATUS_WARNING,
    STATUS_SERIOUS,
    STATUS_CRITICAL,
    FAILED,
    "#ffffff",
]
