"""Reusable, update-in-place drawing layers for planar multi-robot scenes.

Each layer creates its matplotlib artists once and updates them per frame,
which keeps rendering fast on a CPU-only machine.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from matplotlib.axes import Axes
from matplotlib.collections import EllipseCollection, LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle

from multirobotcore_viz import style


class RobotLayer:
    """Disc-shaped robots with optional heading ticks and ID labels."""

    def __init__(
        self,
        ax: Axes,
        positions: np.ndarray,
        radius: float | Sequence[float],
        colors: str | Sequence[str] = style.ROBOT,
        labels: Sequence[str] | None = None,
        show_heading: bool = False,
        zorder: float = 5.0,
        label_size: float = 7.0,
    ) -> None:
        positions = np.asarray(positions, dtype=float)
        n = len(positions)
        radii = np.broadcast_to(np.asarray(radius, dtype=float), (n,))
        self.ax = ax
        self.discs = EllipseCollection(
            widths=2 * radii,
            heights=2 * radii,
            angles=np.zeros(n),
            units="xy",
            offsets=positions,
            offset_transform=ax.transData,
            facecolors=colors,
            edgecolors=style.SURFACE,
            linewidths=1.2,
            zorder=zorder,
        )
        ax.add_collection(self.discs)
        self.radii = radii
        self.heading = None
        if show_heading:
            self.heading = LineCollection([], colors=style.SURFACE, linewidths=1.4, zorder=zorder + 0.1)
            ax.add_collection(self.heading)
        self.labels = []
        if labels is not None:
            for p, text in zip(positions, labels, strict=True):
                self.labels.append(
                    ax.text(
                        p[0],
                        p[1],
                        text,
                        ha="center",
                        va="center",
                        fontsize=label_size,
                        color=style.SURFACE,
                        weight="bold",
                        zorder=zorder + 0.2,
                    )
                )

    def update(
        self,
        positions: np.ndarray,
        colors: str | Sequence[str] | None = None,
        headings: np.ndarray | None = None,
        visible: Sequence[bool] | None = None,
    ) -> None:
        positions = np.asarray(positions, dtype=float)
        self.discs.set_offsets(positions)
        if colors is not None:
            self.discs.set_facecolors(colors)
        if visible is not None:
            alpha = np.where(np.asarray(visible), 1.0, 0.0)
            face = self.discs.get_facecolors()
            if len(face) == len(alpha):
                face[:, 3] = alpha
                self.discs.set_facecolors(face)
        if self.heading is not None and headings is not None:
            tips = positions + np.stack([np.cos(headings), np.sin(headings)], axis=1) * self.radii[:, None]
            self.heading.set_segments(np.stack([positions, tips], axis=1))
        for label, p in zip(self.labels, positions, strict=False):
            label.set_position((p[0], p[1]))


class EdgeLayer:
    """Undirected communication or formation links."""

    def __init__(
        self,
        ax: Axes,
        color: str = style.LINK,
        width: float = 1.0,
        alpha: float = 0.9,
        zorder: float = 2.0,
        linestyle: str = "-",
    ) -> None:
        self.lines = LineCollection(
            [], colors=color, linewidths=width, alpha=alpha, zorder=zorder, linestyles=linestyle
        )
        ax.add_collection(self.lines)

    def update(self, positions: np.ndarray, edges: Sequence[Sequence[int]]) -> None:
        positions = np.asarray(positions, dtype=float)
        segments = [(positions[int(e[0])], positions[int(e[1])]) for e in edges]
        self.lines.set_segments(segments)


class TrailLayer:
    """Fading recent-history trails, one polyline per robot."""

    def __init__(
        self,
        ax: Axes,
        num_robots: int,
        colors: str | Sequence[str] = style.ROBOT,
        width: float = 1.4,
        alpha: float = 0.45,
        length: int = 60,
        zorder: float = 3.0,
    ) -> None:
        self.length = length
        self.lines = LineCollection([], colors=colors, linewidths=width, alpha=alpha, zorder=zorder)
        self.num_robots = num_robots
        ax.add_collection(self.lines)

    def update(self, history: np.ndarray) -> None:
        """`history` has shape (T, N, 2): positions up to the current frame."""
        recent = history[-self.length :]
        self.lines.set_segments([recent[:, i, :] for i in range(self.num_robots)])


def draw_obstacles(ax: Axes, obstacles: dict, zorder: float = 1.0) -> None:
    """Draws obstacles from the C++ JSON format {"circles": [...], "boxes": [...]}."""
    for c in obstacles.get("circles", []):
        ax.add_patch(
            Circle(
                c["center"],
                c["radius"],
                facecolor=style.OBSTACLE_FILL,
                edgecolor=style.OBSTACLE_EDGE,
                linewidth=0.8,
                zorder=zorder,
            )
        )
    for b in obstacles.get("boxes", []):
        (x0, y0), (x1, y1) = b["min"], b["max"]
        ax.add_patch(
            Rectangle(
                (x0, y0),
                x1 - x0,
                y1 - y0,
                facecolor=style.OBSTACLE_FILL,
                edgecolor=style.OBSTACLE_EDGE,
                linewidth=0.8,
                zorder=zorder,
            )
        )


def draw_goals(
    ax: Axes, goals: np.ndarray, color: str = style.TEXT_SECONDARY, size: float = 46.0, zorder: float = 2.5
) -> None:
    """Hollow square markers for goal positions."""
    goals = np.asarray(goals, dtype=float)
    ax.scatter(
        goals[:, 0],
        goals[:, 1],
        marker="s",
        s=size,
        facecolors="none",
        edgecolors=color,
        linewidths=1.1,
        zorder=zorder,
    )


def legend_handles(entries: Sequence[tuple[str, str, str]]) -> list:
    """Legend proxies from (label, color, kind) with kind in {disc, line, square}."""
    handles = []
    for label, color, kind in entries:
        if kind == "disc":
            handles.append(
                Line2D(
                    [],
                    [],
                    marker="o",
                    linestyle="",
                    markersize=8,
                    markerfacecolor=color,
                    markeredgecolor=style.SURFACE,
                    label=label,
                )
            )
        elif kind == "square":
            handles.append(
                Line2D(
                    [],
                    [],
                    marker="s",
                    linestyle="",
                    markersize=7,
                    markerfacecolor="none",
                    markeredgecolor=color,
                    label=label,
                )
            )
        else:
            handles.append(Line2D([], [], color=color, linewidth=2.0, label=label))
    return handles
