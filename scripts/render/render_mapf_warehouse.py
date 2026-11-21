"""Renders docs/assets/mapf_warehouse.gif from mrc_demo_mapf_warehouse output.

Robots leave the station columns for shelf access points through one-lane
aisles, following the collision-free prioritized-planning solution. CBS
results (optimal on a subset, timeout on the full team) are reported in the
caption, exactly as computed by the demo.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from matplotlib.collections import LineCollection

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from multirobotcore_viz import draw, grid, pipeline, style
from multirobotcore_viz.animation import GifRecorder

CONFIG = "configs/mapf/warehouse.json"
FRAMES_PER_STEP = 3
STATION_TINT = "#e2ecf8"
SHELF = "#c6c4bd"


def _choose_solution(result: dict) -> tuple[str, dict]:
    if result["cbs"]["status"] == "solved":
        return "Conflict-Based Search", result["cbs"]
    if result["prioritized"]["status"] == "solved":
        return "prioritized planning", result["prioritized"]
    raise RuntimeError("no collision-free solution in the result file")


def render(result: dict, out: Path) -> Path:
    rows = result["map"]["rows"]
    width, height = result["map"]["width"], result["map"]["height"]
    planner, solution = _choose_solution(result)
    paths = grid.path_arrays(solution["paths"])
    goals = np.asarray([a["goal"] for a in result["agents"]], dtype=float) + 0.5
    n = len(paths)
    horizon = max(len(p) for p in paths) - 1

    fig = style.new_figure(860, 510)
    subset = result["subset"]
    cbs_status = result["cbs"]["status"].replace("_", " ")
    cbs_note = (
        f"CBS optimal for {subset['num_agents']} robots ({subset['cbs']['sum_of_costs']} vs. "
        f"{subset['prioritized']['sum_of_costs']}), {cbs_status} for {n}"
        if subset["cbs"]["status"] == "solved"
        else f"CBS {cbs_status} for {n} robots"
    )
    style.figure_title(
        fig, "Warehouse multi-agent path finding", f"{n} robots, one-lane aisles, {planner}  |  {cbs_note}"
    )
    ax = style.world_axes(fig, (0.02, 0.16, 0.96, 0.7), (0, width), (0, height))
    grid.draw_grid(
        ax, rows, blocked_color=SHELF, highlight={STATION_TINT: [tuple(c) for c in result["stations"]]}
    )
    ax.scatter(
        goals[:, 0],
        goals[:, 1],
        marker="s",
        s=70,
        facecolors="none",
        edgecolors=style.TEXT_SECONDARY,
        linewidths=1.0,
        zorder=2,
    )
    routes = LineCollection([], colors=style.BLUE, linewidths=1.3, alpha=0.35, zorder=1.8)
    ax.add_collection(routes)
    robots = draw.RobotLayer(
        ax,
        grid.positions_at(paths, 0.0),
        radius=0.4,
        colors=style.BLUE,
        labels=[str(i) for i in range(n)],
        label_size=6.5,
    )
    ax.legend(
        handles=draw.legend_handles(
            [
                ("en route", style.BLUE, "disc"),
                ("arrived", style.STATUS_GOOD, "disc"),
                ("shelf access goal", style.TEXT_SECONDARY, "square"),
            ]
        ),
        loc="upper left",
        bbox_to_anchor=(0.0, -0.01),
        ncol=3,
        fontsize=9,
    )
    status = fig.text(0.02, 0.035, "", fontsize=9.5, color=style.TEXT_SECONDARY)
    metrics = (
        f"sum of costs {solution['sum_of_costs']} (bound {result['independent']['sum_of_costs']}; "
        f"uncoordinated: {result['independent']['conflicts']} conflicts)  |  "
        f"makespan {solution['makespan']}"
    )

    arrival = [len(p) - 1 for p in paths]
    rec = GifRecorder(fig, fps=12, max_colors=64)
    for t in np.arange(0, horizon * FRAMES_PER_STEP + 1) / FRAMES_PER_STEP:
        pos = grid.positions_at(paths, t)
        done = [t >= arrival[i] for i in range(n)]
        robots.update(pos, colors=[style.STATUS_GOOD if d else style.BLUE for d in done])
        routes.set_segments([grid.remaining_route(p, t) for p in paths])
        status.set_text(f"t = {t:4.1f}   arrived {sum(done):2d}/{n}   |   {metrics}")
        rec.capture(repeat=8 if t == 0 else 1)
    path = rec.save(out, hold_last_seconds=2.5)
    pipeline.report(path, rec.num_frames)
    return path


def main() -> None:
    args = pipeline.render_arguments(__doc__).parse_args()
    result, _ = pipeline.obtain_result(args, "mapf_warehouse", CONFIG)
    path = render(result, args.output or pipeline.ASSETS_DIR / "mapf_warehouse.gif")
    pipeline.update_manifest(
        path,
        "warehouse 31x13, 24 robots, stations to shelves",
        "prioritized planning (CBS on a subset)",
        result["config"].get("seed"),
        "scripts/render/render_mapf_warehouse.py",
        CONFIG,
    )


if __name__ == "__main__":
    main()
