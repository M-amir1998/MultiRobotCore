"""Renders the flagship warehouse animation from mrc_demo_warehouse output.

Output: docs/assets/warehouse_hero.gif - task requests, Hungarian allocation,
prioritized/CBS multi-agent plans, ORCA-protected execution and deliveries.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.patches import Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from multirobotcore_viz import draw, pipeline, style
from multirobotcore_viz.animation import GifRecorder, frame_indices

CONFIG = "configs/warehouse/warehouse_mission.json"
STATE_COLORS = {
    "idle": "#9a9994",
    "to_pickup": style.BLUE,
    "picking": style.BLUE,
    "to_dropoff": style.ORANGE,
    "dropping": style.ORANGE,
}
TASK = style.YELLOW
STATION = style.VIOLET


def render(result: dict, out: Path, frames: int = 240) -> Path:
    m = result["mission"]
    rows = m["map"]
    height, width = len(rows), len(rows[0])
    cs = m["cell_size"]
    traj = np.asarray(m["trajectory"]["positions"])
    times = np.asarray(m["trajectory"]["t"])
    names = m["state_names"]
    tasks = {t["id"]: t for t in m["tasks"]}
    summary = m["summary"]

    fig = style.new_figure(1100, 600)
    style.figure_title(
        fig,
        "Multi-robot warehouse: allocation -> multi-agent path finding -> collision avoidance",
        "Hungarian task assignment on grid travel costs | prioritized planning with CBS fallback | "
        "ORCA safety layer | deterministic simulation",
    )
    ax = style.world_axes(fig, (0.015, 0.14, 0.97, 0.72), (0, width * cs), (0, height * cs))
    ax.add_patch(
        Rectangle((0, 0), width * cs, height * cs, facecolor=style.FREE_CELL, edgecolor="none", zorder=0)
    )
    # ASCII rows are top-to-bottom; grid y = 0 is the bottom row.
    for r, line in enumerate(rows):
        y = height - 1 - r
        for x, ch in enumerate(line):
            if ch != ".":
                ax.add_patch(
                    Rectangle(
                        (x * cs + 0.04, y * cs + 0.04),
                        cs - 0.08,
                        cs - 0.08,
                        facecolor=style.OBSTACLE_FILL,
                        edgecolor=style.OBSTACLE_EDGE,
                        linewidth=0.6,
                        zorder=1,
                    )
                )
    for x, y in m["stations"]:
        ax.add_patch(
            Rectangle(
                (x * cs + 0.1, y * cs + 0.1),
                cs - 0.2,
                cs - 0.2,
                facecolor="none",
                edgecolor=STATION,
                linewidth=1.3,
                zorder=1.5,
            )
        )
    for x, y in m["homes"]:
        ax.add_patch(
            Rectangle(
                (x * cs + 0.15, y * cs + 0.15),
                cs - 0.3,
                cs - 0.3,
                facecolor="none",
                edgecolor=style.FRAME,
                linewidth=0.9,
                linestyle=(0, (2, 2)),
                zorder=1.5,
            )
        )
    ax.text(0.5 * cs, height * cs + 0.15, "packing stations", fontsize=8.5, color=STATION, va="bottom")
    ax.text(
        width * cs - 0.5 * cs,
        height * cs + 0.15,
        "robot homes",
        fontsize=8.5,
        color=style.TEXT_SECONDARY,
        va="bottom",
        ha="right",
    )

    open_tasks = ax.scatter(
        [], [], marker="s", s=70, facecolors="none", edgecolors=TASK, linewidths=1.8, zorder=2.5
    )
    claimed_tasks = ax.scatter(
        [], [], marker="s", s=70, color=TASK, edgecolors=style.SURFACE, linewidths=0.8, zorder=2.5
    )
    routes = LineCollection([], linewidths=1.6, alpha=0.6, zorder=3)
    ax.add_collection(routes)
    robots = draw.RobotLayer(
        ax, traj[0], radius=0.33 * cs, labels=[str(i) for i in range(traj.shape[1])], label_size=6.5, zorder=6
    )
    status = fig.text(0.015, 0.085, "", fontsize=10.5, color=style.TEXT_PRIMARY)
    handles = draw.legend_handles(
        [
            ("idle", STATE_COLORS["idle"], "disc"),
            ("to pickup", style.BLUE, "disc"),
            ("carrying", style.ORANGE, "disc"),
            ("task request", TASK, "square"),
            ("planned route", style.TEXT_SECONDARY, "line"),
        ]
    )
    fig.legend(handles=handles, loc="lower left", ncol=5, bbox_to_anchor=(0.01, 0.0))

    delivered_times = sorted(t["delivered"] for t in m["tasks"] if t["delivered"] >= 0)
    rec = GifRecorder(fig, fps=20)
    for k in frame_indices(len(times), frames):
        t = times[k]
        states = [names[s] for s in m["robot_states"][k]]
        colors = [STATE_COLORS[s] for s in states]
        robots.update(traj[k], colors=colors)
        segs = [np.asarray(route) for route in m["routes"][k]]
        routes.set_segments(segs)
        routes.set_color(colors)
        claimed = {tid for tid in m["robot_tasks"][k] if tid is not None}
        visible = m["visible_tasks"][k]
        unclaimed = [tasks[i]["pickup"] for i in visible if i not in claimed]
        picked_up = {i for i in claimed if 0 <= tasks[i]["picked"] <= t}
        claimed_pts = [tasks[i]["pickup"] for i in visible if i in claimed and i not in picked_up]
        to_xy = lambda cells: (np.asarray(cells, dtype=float) + 0.5) * cs if cells else np.empty((0, 2))  # noqa: E731
        open_tasks.set_offsets(to_xy(unclaimed))
        claimed_tasks.set_offsets(to_xy(claimed_pts))
        done = sum(1 for d in delivered_times if d <= t)
        status.set_text(
            f"t = {t:5.1f} s    {traj.shape[1]} robots    delivered {done}/{summary['tasks_requested']}    "
            f"waiting requests {len(unclaimed)}    robot contacts {m['metrics']['robot_collisions']}    "
            f"replans {summary['replans']} (mean {summary['mean_planning_ms']:.1f} ms)"
        )
        rec.capture(repeat=6 if k == 0 else 1)
    path = rec.save(out, hold_last_seconds=2.5)
    pipeline.report(path, rec.num_frames)
    return path


def main() -> None:
    parser = pipeline.render_arguments(__doc__)
    parser.add_argument("--frames", type=int, default=240)
    args = parser.parse_args()
    result, _ = pipeline.obtain_result(args, "warehouse", CONFIG)
    out = args.output or pipeline.ASSETS_DIR / "warehouse_hero.gif"
    path = render(result, out, args.frames)
    pipeline.update_manifest(
        path,
        "12 robots, 31 x 13 m warehouse, task arrivals",
        "Hungarian allocation + prioritized planning/CBS + ORCA",
        result["config"]["seed"],
        "scripts/render/render_warehouse.py",
        CONFIG,
    )


if __name__ == "__main__":
    main()
