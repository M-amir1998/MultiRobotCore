"""Renders the formation animations from mrc_demo_formation output.

Outputs (docs/assets/):
  formation.gif              gather -> V -> column through a passage -> V restored
  formation_leader_loss.gif  leader failure without vs. with re-election
  formation_transitions.gif  line -> V -> circle -> triangle reconfiguration
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from multirobotcore_viz import draw, pipeline, style
from multirobotcore_viz.animation import GifRecorder, frame_indices

CONFIG = "configs/formation/formation.json"


def _phase_at(events: list[dict], t: float) -> str:
    label = events[0]["label"] if events else ""
    for e in events:
        if e["time"] <= t + 1e-9:
            label = e["label"]
    return label


class MissionPanel:
    """World view of one formation mission run."""

    def __init__(self, fig, rect, run: dict, title: str | None = None, window: float | None = None) -> None:
        self.run = run
        arena = run["arena"]
        (x0, y0), (x1, y1) = arena["min"], arena["max"]
        self.arena_x = (x0, x1)
        self.window = window
        xlim = (x0, x0 + window) if window else (x0, x1)
        self.ax = style.world_axes(fig, rect, xlim, (y0, y1), title=title)
        draw.draw_obstacles(self.ax, run["obstacles"])
        path = np.asarray(run["leader_path"])
        self.ax.plot(
            path[:, 0],
            path[:, 1],
            color=style.TEXT_SECONDARY,
            linewidth=0.8,
            linestyle=(0, (2, 4)),
            zorder=1.5,
        )
        traj = run["trajectory"]
        self.positions = np.asarray(traj["positions"])
        self.status = traj["status"]
        self.times = np.asarray(traj["t"])
        self.slots = np.asarray(run["slots"])
        n = self.positions.shape[1]
        radius = run["robot_radius"]
        self.links = draw.EdgeLayer(self.ax, width=0.8, alpha=0.7)
        self.trails = draw.TrailLayer(self.ax, n, colors=style.ROBOT, length=40, alpha=0.3)
        self.slot_marks = self.ax.scatter(
            self.slots[0][:, 0],
            self.slots[0][:, 1],
            s=60,
            facecolors="none",
            edgecolors=style.TEXT_SECONDARY,
            linewidths=0.9,
            linestyles=(0, (1.5, 1.5)),
            zorder=4,
        )
        self.robots = draw.RobotLayer(
            self.ax,
            self.positions[0],
            radius=radius * 1.25,
            labels=[str(i) for i in range(n)],
            label_size=6.5,
        )
        self.failed_marks = self.ax.scatter(
            [], [], marker="x", s=46, color=style.STATUS_CRITICAL, linewidths=1.6, zorder=7
        )
        self.text = self.ax.text(
            0.01,
            0.97,
            "",
            fontsize=9.5,
            color=style.TEXT_SECONDARY,
            va="top",
            zorder=8,
            transform=self.ax.transAxes,
        )

    def update(self, k: int) -> None:
        run = self.run
        leader = run["leader"][k]
        status = self.status[k]
        colors = []
        failed = []
        for i, s in enumerate(status):
            if s == "failed":
                colors.append(style.FAILED)
                failed.append(self.positions[k][i])
            elif i == leader:
                colors.append(style.LEADER)
            else:
                colors.append(style.ROBOT)
        self.links.update(self.positions[k], run["edges"][k])
        self.trails.update(self.positions[: k + 1])
        self.slot_marks.set_offsets(self.slots[k])
        self.robots.update(self.positions[k], colors=colors)
        self.failed_marks.set_offsets(np.asarray(failed) if failed else np.empty((0, 2)))
        if self.window:
            # Camera follows the team centroid, clamped to the arena.
            centre = float(np.mean(self.positions[k][:, 0]))
            lo = min(max(centre - 0.45 * self.window, self.arena_x[0]), self.arena_x[1] - self.window)
            self.ax.set_xlim(lo, lo + self.window)
        t = self.times[k]
        phase = _phase_at(run["events"], t)
        self.text.set_text(f"t = {t:5.1f} s   {phase}   slot RMSE {run['rmse'][k]:.2f} m")


def render_mission(result: dict, out: Path) -> Path:
    run = result["mission"]
    fig = style.new_figure(960, 600)
    style.figure_title(
        fig,
        "Formation control through a narrow passage",
        "consensus formation pinned to a leader: gather -> V -> column -> V restored",
    )
    panel = MissionPanel(fig, (0.02, 0.09, 0.96, 0.78), run, window=31.0)
    handles = draw.legend_handles(
        [
            ("leader", style.LEADER, "disc"),
            ("follower", style.ROBOT, "disc"),
            ("desired slot", style.TEXT_SECONDARY, "square"),
            ("communication link", style.LINK, "line"),
        ]
    )
    fig.legend(handles=handles, loc="lower center", ncol=4, bbox_to_anchor=(0.5, 0.0))
    rec = GifRecorder(fig, fps=20)
    indices = frame_indices(len(panel.times), 230)
    for k in indices:
        panel.update(k)
        rec.capture(repeat=10 if k == 0 else 1)
    path = rec.save(out, hold_last_seconds=2.0)
    pipeline.report(path, rec.num_frames)
    return path


def render_leader_loss(result: dict, out: Path) -> Path:
    runs = result["leader_loss"]
    fig = style.new_figure(1000, 760)
    style.figure_title(
        fig,
        "Leader failure: degradation vs. re-election",
        "leader 0 fails at t = 14 s; followers lose their reference",
    )
    top = MissionPanel(
        fig, (0.02, 0.5, 0.96, 0.36), runs["without_reelection"], title="without re-election: the team stalls"
    )
    bottom = MissionPanel(
        fig,
        (0.02, 0.07, 0.96, 0.36),
        runs["with_reelection"],
        title="min-ID flooding election: robot 1 takes over",
    )
    handles = draw.legend_handles(
        [
            ("leader", style.LEADER, "disc"),
            ("follower", style.ROBOT, "disc"),
            ("failed robot", style.FAILED, "disc"),
        ]
    )
    fig.legend(handles=handles, loc="lower center", ncol=3, bbox_to_anchor=(0.5, 0.0))
    times = top.times
    last = int(np.searchsorted(times, 52.0))
    rec = GifRecorder(fig, fps=20)
    for k in frame_indices(last, 200):
        top.update(k)
        bottom.update(k)
        rec.capture()
    path = rec.save(out, hold_last_seconds=2.0)
    pipeline.report(path, rec.num_frames)
    return path


def render_transitions(result: dict, out: Path) -> Path:
    data = result["transitions"]
    positions = np.asarray(data["trajectory"]["positions"])
    times = np.asarray(data["trajectory"]["t"])
    slots = np.asarray(data["slots"])
    ref = data["reference"]
    fig = style.new_figure(640, 560)
    style.figure_title(
        fig, "Formation reconfiguration", "min sum-of-squares slot assignment + quintic blending"
    )
    ax = style.world_axes(
        fig, (0.04, 0.04, 0.92, 0.8), (ref[0] - 4.2, ref[0] + 4.2), (ref[1] - 3.6, ref[1] + 3.6)
    )
    n = positions.shape[1]
    trails = draw.TrailLayer(ax, n, length=25, alpha=0.35)
    slot_marks = ax.scatter(
        slots[0][:, 0],
        slots[0][:, 1],
        s=90,
        facecolors="none",
        edgecolors=style.TEXT_SECONDARY,
        linewidths=0.9,
        zorder=4,
    )
    robots = draw.RobotLayer(
        ax, positions[0], radius=data["robot_radius"] * 1.3, labels=[str(i) for i in range(n)], label_size=7
    )
    label = ax.text(
        ref[0] - 4.0, ref[1] + 3.35, "", fontsize=11, color=style.TEXT_PRIMARY, va="top", weight="bold"
    )
    rec = GifRecorder(fig, fps=15)
    for k in frame_indices(len(times), 160):
        trails.update(positions[: k + 1])
        slot_marks.set_offsets(slots[k])
        robots.update(positions[k])
        label.set_text(data["labels"][k])
        rec.capture()
    path = rec.save(out, hold_last_seconds=1.5)
    pipeline.report(path, rec.num_frames)
    return path


def main() -> None:
    args = pipeline.render_arguments(__doc__).parse_args()
    result, _ = pipeline.obtain_result(args, "formation", CONFIG)
    seed = result["config"]["seed"]
    script = "scripts/render/render_formation.py"
    assets = pipeline.ASSETS_DIR
    outputs = [
        (
            render_mission(result, assets / "formation.gif"),
            "7 robots, passage between two walls",
            "leader-pinned consensus formation + shape transitions",
        ),
        (
            render_leader_loss(result, assets / "formation_leader_loss.gif"),
            "leader failure at t = 14 s",
            "leader-follower formation + min-ID re-election",
        ),
        (
            render_transitions(result, assets / "formation_transitions.gif"),
            "8 robots, in place",
            "assignment-based formation transitions",
        ),
    ]
    for path, scenario, algorithm in outputs:
        pipeline.update_manifest(path, scenario, algorithm, seed, script, CONFIG)


if __name__ == "__main__":
    main()
