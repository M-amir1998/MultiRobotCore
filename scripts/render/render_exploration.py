"""Renders the cooperative frontier-exploration animation from mrc_demo_exploration output.

Output (docs/assets/):
  exploration.gif   shared-map exploration: map expansion, frontiers, sensor
                    footprints, assigned targets and coverage versus time for
                    all assignment strategies
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from matplotlib.colors import to_rgb
from matplotlib.lines import Line2D
from matplotlib.patches import Circle

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from multirobotcore_viz import draw, pipeline, style
from multirobotcore_viz.animation import GifRecorder, frame_indices

CONFIG = "configs/exploration/exploration.json"
LABELS = {
    "nearest_frontier": "nearest frontier (uncoordinated)",
    "hungarian": "Hungarian assignment",
    "utility_discount": "utility discount (Burgard et al.)",
}
# Strategies are told apart by dash pattern in neutral ink, so that hue keeps
# encoding robot identity only.
STRATEGY_DASH = {"nearest_frontier": (0, (1.5, 1.5)), "hungarian": "-", "utility_discount": (0, (5, 2))}
FRONTIER = style.MAGENTA
OCCUPIED = "#3f3e3b"


def main() -> None:
    args = pipeline.render_arguments(__doc__).parse_args()
    result, _ = pipeline.obtain_result(args, "exploration", CONFIG)
    grid = result["map"]
    width, height, res = grid["width"], grid["height"], grid["resolution"]
    frames = result["frames"]
    step_s = result["step_seconds"]
    sensor_cells = result["sensor_range_cells"]
    n = len(frames[0]["robots"])
    robot_colors = style.CATEGORICAL[:n]

    # Belief image (row 0 = y 0, drawn with origin="lower").
    palette = {0: to_rgb(style.UNKNOWN), 1: to_rgb(style.FREE_CELL), 2: to_rgb(OCCUPIED)}
    belief = np.zeros((height, width), dtype=np.uint8)
    frontier_rgb = np.asarray(to_rgb(FRONTIER))

    fig = style.new_figure(960, 520)
    style.figure_title(
        fig,
        "Cooperative frontier exploration",
        f"{n} robots, shared map, {LABELS[result['strategy']]}, range sensor {sensor_cells * res:.1f} m",
    )
    ax = style.world_axes(fig, (0.015, 0.03, 0.6, 0.84), (0, width * res), (0, height * res))
    image = ax.imshow(
        np.zeros((height, width, 3)),
        origin="lower",
        extent=(0, width * res, 0, height * res),
        interpolation="nearest",
        zorder=0,
    )
    trails = draw.TrailLayer(ax, n, colors=robot_colors, length=10_000, alpha=0.55, width=1.3)
    footprints = [
        ax.add_patch(
            Circle((0, 0), sensor_cells * res, facecolor=c, alpha=0.10, edgecolor=c, linewidth=0.8, zorder=2)
        )
        for c in robot_colors
    ]
    target_lines = [
        ax.plot([], [], color=c, linewidth=1.0, linestyle=(0, (3, 2)), zorder=3)[0] for c in robot_colors
    ]
    target_marks = ax.scatter([], [], marker="s", s=60, facecolors="none", linewidths=1.6, zorder=4)
    robots = draw.RobotLayer(
        ax,
        np.zeros((n, 2)),
        radius=0.45,
        colors=robot_colors,
        labels=[str(i) for i in range(n)],
        label_size=7,
    )

    # Coverage panel with all strategies (final curves faint, live curve bold).
    ax_cov = fig.add_axes((0.69, 0.3, 0.29, 0.5))
    comparison = result["comparison"]
    t_max = max(len(v["coverage"]) for v in comparison.values()) * step_s
    ax_cov.set_xlim(0, t_max)
    ax_cov.set_ylim(0, 103)
    ax_cov.set_xlabel("time [s]")
    ax_cov.set_title("explored free space [%]", loc="left", fontsize=10)
    ax_cov.grid(True, axis="y")
    for name, data in comparison.items():
        cov = 100 * np.asarray(data["coverage"])
        ax_cov.plot(
            np.arange(len(cov)) * step_s,
            cov,
            color=style.TEXT_SECONDARY,
            linestyle=STRATEGY_DASH[name],
            linewidth=1.2,
            alpha=0.6,
        )
    (live,) = ax_cov.plot(
        [], [], color=style.TEXT_PRIMARY, linestyle=STRATEGY_DASH[result["strategy"]], linewidth=2.4
    )
    ax_cov.legend(
        handles=[
            Line2D(
                [],
                [],
                color=style.TEXT_SECONDARY,
                linestyle=STRATEGY_DASH[k],
                linewidth=1.6,
                label=LABELS[k] + (" (animated)" if k == result["strategy"] else ""),
            )
            for k in comparison
        ],
        loc="lower right",
        fontsize=8,
    )
    fig.legend(
        handles=[
            Line2D(
                [],
                [],
                marker="s",
                linestyle="",
                markersize=7,
                markerfacecolor=FRONTIER,
                markeredgecolor=FRONTIER,
                label="frontier cell",
            ),
            *draw.legend_handles([("assigned target", style.TEXT_SECONDARY, "square")]),
        ],
        loc="lower left",
        bbox_to_anchor=(0.685, 0.02),
        fontsize=8.5,
        ncol=2,
    )
    status = fig.text(0.69, 0.86, "", fontsize=10, color=style.TEXT_SECONDARY)
    coverage_live = 100 * np.asarray(result["coverage"])

    positions = np.asarray([[np.asarray(r) for r in f["robots"]] for f in frames], dtype=float)
    centers = (positions + 0.5) * res  # (F, N, 2)
    picked = set(frame_indices(len(frames), 120))
    rec = GifRecorder(fig, fps=15, max_colors=64)
    for k, f in enumerate(frames):
        for idx, value in f["known"]:
            belief.flat[idx] = value
        if k not in picked:
            continue
        rgb = np.empty((height, width, 3))
        for value, color in palette.items():
            rgb[belief == value] = color
        if f["frontier"]:
            rgb.reshape(-1, 3)[np.asarray(f["frontier"])] = frontier_rgb
        image.set_data(rgb)
        trails.update(centers[: k + 1])
        robots.update(centers[k])
        marks, mark_colors = [], []
        for i, target in enumerate(f["targets"]):
            footprints[i].center = tuple(centers[k, i])
            if target is None:
                target_lines[i].set_data([], [])
                continue
            tx, ty = (np.asarray(target) + 0.5) * res
            target_lines[i].set_data([centers[k, i, 0], tx], [centers[k, i, 1], ty])
            marks.append((tx, ty))
            mark_colors.append(robot_colors[i])
        target_marks.set_offsets(np.asarray(marks) if marks else np.empty((0, 2)))
        target_marks.set_edgecolors(mark_colors)
        live.set_data(np.arange(k + 1) * step_s, coverage_live[: k + 1])
        status.set_text(f"t = {f['step'] * step_s:5.0f} s    explored {100 * f['coverage']:5.1f} %")
        rec.capture(repeat=8 if k == 0 else 1)
    metrics = result["metrics"]
    status.set_text(f"done in {metrics['time_s']:.0f} s, 90 % at {metrics['time_to_90_s']:.0f} s")
    rec.capture(repeat=20)
    path = rec.save(pipeline.ASSETS_DIR / "exploration.gif", hold_last_seconds=1.5)
    pipeline.report(path, rec.num_frames)
    pipeline.update_manifest(
        path,
        f"office map {width}x{height} cells, {n} robots",
        f"frontier exploration ({result['strategy']})",
        None,
        "scripts/render/render_exploration.py",
        CONFIG,
    )


if __name__ == "__main__":
    main()
