"""Renders docs/assets/cbs.gif from mrc_demo_mapf_conflicts output.

Left: independent shortest paths (no coordination) with collisions marked.
Right: Conflict-Based Search on the same instance, collision-free.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from matplotlib.patches import Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from multirobotcore_viz import draw, grid, pipeline, style
from multirobotcore_viz.animation import GifRecorder

CONFIG = "configs/mapf/conflicts.json"
FRAMES_PER_STEP = 5
WALL = "#cfcec8"  # darker than the default so one-lane corridors stand out


def _panel(fig, rect, rows, title, agents, paths, colors):
    ax = style.world_axes(fig, rect, (0, 1), (0, 1), title=title)
    grid.draw_grid(ax, rows, blocked_color=WALL)
    for i, agent in enumerate(agents):
        gx, gy = agent["goal"]
        ax.add_patch(
            Rectangle(
                (gx + 0.12, gy + 0.12), 0.76, 0.76, fill=False, edgecolor=colors[i], linewidth=1.6, zorder=2
            )
        )
        ax.plot(*paths[i].T, color=colors[i], alpha=0.35, linewidth=2.2, zorder=1.5)
    robots = draw.RobotLayer(
        ax,
        grid.positions_at(paths, 0.0),
        radius=0.36,
        colors=colors,
        labels=[str(i) for i in range(len(paths))],
        label_size=8,
    )
    return ax, robots


def _conflict_marks(conflicts: list[dict]) -> list[tuple[float, np.ndarray]]:
    """(time, location) of each collision; swaps meet mid-edge half a step later."""
    marks = []
    for c in conflicts:
        cell = np.asarray(c["cell"], dtype=float) + 0.5
        if c["type"] == "vertex":
            marks.append((float(c["t"]), cell))
        else:
            other = np.asarray(c["cell2"], dtype=float) + 0.5
            marks.append((c["t"] + 0.5, (cell + other) / 2))
    return marks


def render(result: dict, out: Path) -> Path:
    rows = result["map"]["rows"]
    agents = result["agents"]
    independent = result["independent"]
    cbs = result["cbs"]
    if cbs["status"] != "solved":
        raise RuntimeError("CBS did not solve the demo instance; nothing to render")
    ind_paths = grid.path_arrays(independent["paths"])
    cbs_paths = grid.path_arrays(cbs["paths"])
    colors = style.CATEGORICAL[: len(agents)]
    horizon = max(len(p) for p in ind_paths + cbs_paths) - 1

    fig = style.new_figure(960, 540)
    style.figure_title(
        fig,
        "Why coordination matters: Conflict-Based Search",
        f"one-lane intersection, {len(agents)} robots   |   "
        f"prioritized planning on this instance: {result['prioritized']['status']}",
    )
    ax_left, robots_left = _panel(
        fig, (0.03, 0.12, 0.45, 0.72), rows, "Independent A* (uncoordinated)", agents, ind_paths, colors
    )
    _, robots_right = _panel(
        fig, (0.52, 0.12, 0.45, 0.72), rows, "Conflict-Based Search (optimal)", agents, cbs_paths, colors
    )

    marks = _conflict_marks(independent["conflict_list"])
    ring = ax_left.scatter(
        [], [], s=900, facecolors="none", edgecolors=style.STATUS_CRITICAL, linewidths=2.4, zorder=7
    )
    left_status = fig.text(0.03, 0.06, "", fontsize=10, color=style.TEXT_SECONDARY)
    fig.text(
        0.52,
        0.06,
        f"collision-free  |  sum of costs {cbs['sum_of_costs']} "
        f"(bound {independent['sum_of_costs']})  |  {cbs['high_level_expanded']} CT nodes",
        fontsize=10,
        color=style.TEXT_SECONDARY,
    )

    rec = GifRecorder(fig, fps=15)
    for t in np.arange(0, horizon * FRAMES_PER_STEP + 1) / FRAMES_PER_STEP:
        robots_left.update(grid.positions_at(ind_paths, t))
        robots_right.update(grid.positions_at(cbs_paths, t))
        active = [centre for when, centre in marks if abs(when - t) <= 0.6]
        ring.set_offsets(np.asarray(active) if active else np.empty((0, 2)))
        happened = sum(1 for when, _ in marks if when <= t + 1e-9)
        left_status.set_text(f"t = {t:4.1f}   collisions so far: {happened}")
        left_status.set_color(style.STATUS_CRITICAL if happened else style.TEXT_SECONDARY)
        rec.capture(repeat=6 if t == 0 else 1)
    path = rec.save(out, hold_last_seconds=2.5)
    pipeline.report(path, rec.num_frames)
    return path


def main() -> None:
    args = pipeline.render_arguments(__doc__).parse_args()
    result, _ = pipeline.obtain_result(args, "mapf_conflicts", CONFIG)
    path = render(result, args.output or pipeline.ASSETS_DIR / "cbs.gif")
    pipeline.update_manifest(
        path,
        "one-lane intersection, 4 robots",
        "independent A* vs Conflict-Based Search",
        None,
        "scripts/render/render_cbs.py",
        CONFIG,
    )


if __name__ == "__main__":
    main()
