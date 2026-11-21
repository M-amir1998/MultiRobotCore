"""Grid-world helpers for MAPF scenes.

Grid conventions match the C++ library: a cell is [x, y] with y = 0 at the
bottom; map rows in result files are listed top row first. Cell (x, y)
occupies the unit square [x, x + 1] x [y, y + 1] in plot coordinates.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from matplotlib.axes import Axes
from matplotlib.colors import to_rgb

from multirobotcore_viz import style

BLOCKED_CHARS = frozenset("@OTW#")


def blocked_mask(rows: Sequence[str]) -> np.ndarray:
    """Boolean (height, width) array indexed [y, x] with y = 0 at the bottom."""
    mask = np.array([[c in BLOCKED_CHARS for c in row] for row in rows], dtype=bool)
    return mask[::-1]


def draw_grid(
    ax: Axes,
    rows: Sequence[str],
    free_color: str = style.FREE_CELL,
    blocked_color: str = style.OBSTACLE_FILL,
    grid_lines: bool = True,
    highlight: dict[str, Sequence[Sequence[int]]] | None = None,
) -> tuple[int, int]:
    """Draws the occupancy grid; `highlight` maps a colour to cells to tint.

    Returns (width, height)."""
    mask = blocked_mask(rows)
    height, width = mask.shape
    image = np.empty((height, width, 3))
    image[:] = to_rgb(free_color)
    if highlight:
        for color, cells in highlight.items():
            for x, y in cells:
                image[y, x] = to_rgb(color)
    image[mask] = to_rgb(blocked_color)
    ax.imshow(image, origin="lower", extent=(0, width, 0, height), interpolation="nearest", zorder=0)
    if grid_lines:
        ax.vlines(np.arange(width + 1), 0, height, colors=style.SURFACE, linewidth=0.6, zorder=0.5)
        ax.hlines(np.arange(height + 1), 0, width, colors=style.SURFACE, linewidth=0.6, zorder=0.5)
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    return width, height


def path_arrays(paths: Sequence[Sequence[Sequence[int]]]) -> list[np.ndarray]:
    """Converts JSON paths to arrays of cell centres, shape (T_i, 2)."""
    return [np.asarray(p, dtype=float) + 0.5 for p in paths]


def positions_at(paths: Sequence[np.ndarray], t: float) -> np.ndarray:
    """Positions at continuous time t (timesteps), linearly interpolated
    between cells; agents stay at their last cell after their path ends."""
    out = np.empty((len(paths), 2))
    k = int(np.floor(t))
    frac = t - k
    for i, p in enumerate(paths):
        a = p[min(k, len(p) - 1)]
        b = p[min(k + 1, len(p) - 1)]
        out[i] = a + (b - a) * frac
    return out


def cell_at(path: np.ndarray, step: int) -> np.ndarray:
    """Cell centre occupied at an integer timestep (with parking)."""
    return path[min(step, len(path) - 1)]


def remaining_route(path: np.ndarray, t: float) -> np.ndarray:
    """Polyline from the interpolated position at time t to the end of the path."""
    k = int(np.floor(t))
    if k >= len(path) - 1:
        return path[-1:]
    current = positions_at([path], t)[0]
    return np.vstack([current, path[k + 1 :]])
