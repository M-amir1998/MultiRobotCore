"""Renders the bottleneck scenarios from mrc_demo_avoidance_bottleneck output.

Output: docs/assets/avoidance_bottleneck.gif - ORCA through a doorway (one
way), a doorway with counterflow, and a single-file corridor swap. Arrival and
stall information comes from the C++ run, not from the renderer.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from multirobotcore_viz import draw, pipeline, style
from multirobotcore_viz.animation import GifRecorder, frame_indices

CONFIG = "configs/avoidance/bottleneck.json"
TITLES = {
    "doorway": "Doorway, one way",
    "doorway_counterflow": "Doorway, counterflow",
    "corridor_swap": "Single-file corridor swap",
}


def view_window(data: dict, margin: float = 1.0) -> tuple[float, float, float, float]:
    """Region actually used by robots and goals (plus a margin), inside the bounds."""
    pts = np.concatenate(
        [np.asarray(data["trajectory"]["positions"]).reshape(-1, 2), np.asarray(data["goals"])]
    )
    lo = np.maximum(pts.min(axis=0) - margin, data["bounds"]["min"])
    hi = np.minimum(pts.max(axis=0) + margin, data["bounds"]["max"])
    return float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])


class Panel:
    def __init__(self, fig, rect, data: dict) -> None:
        self.data = data
        self.positions = np.asarray(data["trajectory"]["positions"])
        self.times = np.asarray(data["trajectory"]["t"])
        self.arrivals = np.array([np.inf if a is None or a < 0 else a for a in data["arrival_times"]])
        x0, y0, x1, y1 = view_window(data)
        self.ax = style.world_axes(
            fig, rect, (x0, x1), (y0, y1), title=TITLES.get(data["name"], data["name"])
        )
        draw.draw_obstacles(self.ax, data["obstacles"])
        draw.draw_goals(self.ax, np.asarray(data["goals"]), size=18)
        n = self.positions.shape[1]
        self.trails = draw.TrailLayer(self.ax, n, length=30, alpha=0.3, width=1.1)
        self.robots = draw.RobotLayer(self.ax, self.positions[0], radius=data["robot_radius"])
        self.status = self.ax.text(
            x0 + 0.3, y0 + 0.3, "", fontsize=8.5, color=style.TEXT_SECONDARY, va="bottom"
        )
        self.alert = self.ax.text(
            x0 + 0.3, y1 - 0.3, "", fontsize=8.5, color=style.STATUS_CRITICAL, va="top", weight="bold"
        )

    def update(self, t: float) -> None:
        f = int(np.clip(np.searchsorted(self.times, t, side="right") - 1, 0, len(self.times) - 1))
        now = self.times[f]
        arrived = self.arrivals <= now + 1e-9
        colors = [style.STATUS_GOOD if a else style.ROBOT for a in arrived]
        self.trails.update(self.positions[: f + 1])
        self.robots.update(self.positions[f], colors=colors)
        n = len(arrived)
        self.status.set_text(f"t = {now:4.1f} s   arrived {int(arrived.sum())}/{n}")
        m = self.data["metrics"]
        ended = f == len(self.times) - 1
        if ended and m["stalled"]:
            self.alert.set_text(f"stall detected at {m['stall_time']:.1f} s")
        elif ended and m["reached"] < n:
            self.alert.set_text("congested: goals not reached")
        else:
            self.alert.set_text("")


def render(result: dict, out: Path) -> Path:
    scenarios = result["scenarios"]
    widths = []
    for s in scenarios:
        x0, y0, x1, y1 = view_window(s)
        widths.append((x1 - x0) / (y1 - y0))
    # Panels share one height; the figure height follows from their aspect ratios.
    panel_px = 1200 * 0.95 / sum(widths)
    height_px = int(panel_px + 125)
    fig = style.new_figure(1200, height_px)
    # Title block in pixel units (the figure is short).
    fig.text(
        0.02,
        1 - 20 / height_px,
        "Local avoidance at bottlenecks (ORCA)",
        ha="left",
        va="center",
        fontsize=14,
        weight="bold",
        color=style.TEXT_PRIMARY,
    )
    fig.text(
        0.02,
        1 - 44 / height_px,
        "door waypoints give a minimal route; congestion and deadlock in two-way traffic are "
        "limits of purely local methods",
        ha="left",
        va="center",
        fontsize=10,
        color=style.TEXT_SECONDARY,
    )
    scale = 0.95 / sum(widths)
    panels = []
    left = 0.02
    for s, w in zip(scenarios, widths, strict=True):
        panels.append(Panel(fig, (left, 0.03, w * scale - 0.012, panel_px / height_px), s))
        left += w * scale + 0.004
    fig.legend(
        handles=draw.legend_handles(
            [
                ("moving", style.ROBOT, "disc"),
                ("arrived", style.STATUS_GOOD, "disc"),
                ("goal", style.TEXT_SECONDARY, "square"),
            ]
        ),
        loc="upper right",
        ncol=3,
        bbox_to_anchor=(0.99, 0.99),
    )
    t_end = max(float(p.times[-1]) for p in panels)
    rec = GifRecorder(fig, fps=15, max_colors=64)
    for k in frame_indices(400, 160):
        t = t_end * k / 399
        for p in panels:
            p.update(t)
        rec.capture(repeat=6 if k == 0 else 1)
    path = rec.save(out, hold_last_seconds=3.0)
    pipeline.report(path, rec.num_frames)
    return path


def main() -> None:
    args = pipeline.render_arguments(__doc__).parse_args()
    result, _ = pipeline.obtain_result(args, "avoidance_bottleneck", CONFIG)
    out = args.output or pipeline.ASSETS_DIR / "avoidance_bottleneck.gif"
    path = render(result, out)
    pipeline.update_manifest(
        path,
        "doorway one-way/counterflow, corridor swap",
        "ORCA",
        result["config"]["seed"],
        "scripts/render/render_bottleneck.py",
        CONFIG,
    )


if __name__ == "__main__":
    main()
