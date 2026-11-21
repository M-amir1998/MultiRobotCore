"""Renders the circle-swap comparison from mrc_demo_avoidance_circle_swap output.

Output: docs/assets/circle_swap.gif - ORCA (left) and sampling-based VO (right)
started from identical positions; robot contacts are highlighted.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from matplotlib.collections import EllipseCollection

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from multirobotcore_viz import draw, pipeline, style
from multirobotcore_viz.animation import GifRecorder, frame_indices

CONFIG = "configs/avoidance/circle_swap.json"
PANELS = (("orca", "ORCA (reciprocal half-planes + LP)"), ("vo", "VO (non-reciprocal, sampled)"))


class Panel:
    """One method: robots, trails, goals and contact highlights."""

    def __init__(self, fig, rect, result: dict, method: str, title: str) -> None:
        scenario = result["scenario"]
        data = result["methods"][method]
        self.positions = np.asarray(data["trajectory"]["positions"])
        self.times = np.asarray(data["trajectory"]["t"])
        self.radius = scenario["robot_radius"]
        self.metrics = data["metrics"]
        extent = float(np.abs(np.asarray(scenario["starts"])).max()) + 1.0
        self.ax = style.world_axes(fig, rect, (-extent, extent), (-extent, extent), title=title)
        draw.draw_goals(self.ax, np.asarray(scenario["goals"]), size=30)
        self.trails = draw.TrailLayer(self.ax, self.positions.shape[1], length=40, alpha=0.3)
        self.robots = draw.RobotLayer(self.ax, self.positions[0], radius=self.radius)
        self.contacts = EllipseCollection(
            widths=[],
            heights=[],
            angles=[],
            units="xy",
            offsets=np.empty((0, 2)),
            offset_transform=self.ax.transData,
            facecolors="none",
            edgecolors=style.STATUS_CRITICAL,
            linewidths=2.0,
            zorder=6,
        )
        self.ax.add_collection(self.contacts)
        self.status = self.ax.text(
            -extent + 0.3, -extent + 0.3, "", fontsize=9, color=style.TEXT_SECONDARY, va="bottom"
        )
        # Contact events as detected by the simulator at every step.
        self.events = np.asarray(sorted(e["t"] for e in data["collisions"] if e["other"] is not None))

    def update(self, t: float) -> None:
        f = min(int(np.searchsorted(self.times, t, side="right")) - 1, len(self.times) - 1)
        f = max(f, 0)
        p = self.positions[f]
        self.trails.update(self.positions[: f + 1])
        self.robots.update(p)
        # Overlapping discs (contact) are ringed in the critical status colour.
        d = np.linalg.norm(p[:, None, :] - p[None, :, :], axis=2)
        i, j = np.nonzero(np.triu(d < 2 * self.radius, k=1))
        centers = np.concatenate([p[i], p[j]]) if len(i) else np.empty((0, 2))
        size = np.full(len(centers), 2 * self.radius + 0.25)
        self.contacts.set_offsets(centers)
        self.contacts.set_widths(size)
        self.contacts.set_heights(size)
        self.contacts.set_angles(np.zeros(len(centers)))
        done = f == len(self.times) - 1
        clock = self.metrics["makespan"] if done else self.times[f]
        self.status.set_text(
            f"t = {clock:5.1f} s    contact events: {int(np.sum(self.events <= self.times[f]))}"
            + ("    all goals reached" if done and self.metrics["reached"] == len(p) else "")
        )


def render(result: dict, out: Path) -> Path:
    n = len(result["scenario"]["starts"])
    fig = style.new_figure(1000, 560)
    style.figure_title(
        fig,
        f"Circle swap: {n} robots exchange antipodal positions",
        "identical starts; ORCA guarantees collision-free velocities when every robot's program is feasible",
    )
    panels = [
        Panel(fig, (0.03 + 0.49 * k, 0.02, 0.46, 0.80), result, m, title)
        for k, (m, title) in enumerate(PANELS)
    ]
    t_end = max(float(p.times[-1]) for p in panels)
    rec = GifRecorder(fig, fps=15, max_colors=64)
    for k in frame_indices(400, 150):
        t = t_end * k / 399
        for p in panels:
            p.update(t)
        rec.capture(repeat=8 if k == 0 else 1)
    path = rec.save(out, hold_last_seconds=2.5)
    pipeline.report(path, rec.num_frames)
    return path


def main() -> None:
    args = pipeline.render_arguments(__doc__).parse_args()
    result, _ = pipeline.obtain_result(args, "avoidance_circle_swap", CONFIG)
    out = args.output or pipeline.ASSETS_DIR / "circle_swap.gif"
    path = render(result, out)
    pipeline.update_manifest(
        path,
        f"circle swap, {len(result['scenario']['starts'])} robots",
        "ORCA vs. sampling VO",
        result["config"]["seed"],
        "scripts/render/render_circle_swap.py",
        CONFIG,
    )


if __name__ == "__main__":
    main()
