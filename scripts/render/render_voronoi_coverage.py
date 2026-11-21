"""Renders the Voronoi coverage animation from mrc_demo_coverage output.

Outputs (docs/assets/):
  voronoi_coverage.gif   centralized gradient flow toward centroidal Voronoi
                         configuration over a non-uniform density
  coverage_distributed.gif  limited-range variant with communication links
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from multirobotcore_viz import draw, pipeline, style
from multirobotcore_viz.animation import GifRecorder, frame_indices

CONFIG = "configs/coverage/coverage.json"
HEAT = LinearSegmentedColormap.from_list(
    "density",
    [
        "#f6f9fd",
        style.SEQUENTIAL_BLUE[0],
        style.SEQUENTIAL_BLUE[2],
        style.SEQUENTIAL_BLUE[4],
        style.SEQUENTIAL_BLUE[6],
    ],
)


def _cell_segments(cells: list) -> list:
    segments = []
    for poly in cells:
        if len(poly) < 3:
            continue
        pts = np.asarray(poly)
        segments.extend(zip(pts, np.roll(pts, -1, axis=0), strict=True))
    return segments


def _edges_within(positions: np.ndarray, radius: float) -> list:
    edges = []
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            if np.linalg.norm(positions[i] - positions[j]) <= radius:
                edges.append((i, j))
    return edges


def render(result: dict, run_key: str, out: Path, title: str, subtitle: str, max_frames: int = 110) -> Path:
    run = result[run_key]
    traj = run["trajectory"]
    positions = np.asarray(traj["positions"])  # (F, N, 2)
    times = np.asarray(traj["t"])
    cells = run["cells"]
    centroids = run["centroids"]
    cost = np.asarray(run["cost"])
    error = np.asarray(run["max_centroid_error"])
    domain = np.asarray(result["domain"])
    grid = result["density_grid"]
    x0, x1, y0, y1 = grid["extent"]
    radius = run.get("communication_radius")
    frames = min(len(times), len(cells))

    fig = style.new_figure(940, 540)
    style.figure_title(fig, title, subtitle)
    ax = style.world_axes(
        fig, (0.015, 0.03, 0.64, 0.84), (x0 - 0.15, x1 + 0.15), (y0 - 0.15, y1 + 0.15), frame=False
    )
    values = np.asarray(grid["values"])
    # Banded (filled-contour) heatmap: flat colour regions read as iso-density
    # levels and keep the GIF small.
    xs = np.linspace(x0, x1, values.shape[1])
    ys = np.linspace(y0, y1, values.shape[0])
    levels = np.linspace(values.min(), values.max(), 9)
    ax.contourf(xs, ys, values, levels=levels, cmap=HEAT, zorder=0, antialiased=False)
    ax.add_patch(
        Polygon(domain, closed=True, fill=False, edgecolor=style.TEXT_SECONDARY, linewidth=1.2, zorder=1)
    )
    ax.scatter(*positions[0].T, s=8, color=style.TEXT_SECONDARY, alpha=0.6, zorder=2, linewidths=0)
    links = draw.EdgeLayer(ax, color=style.TEXT_SECONDARY, width=0.6, alpha=0.35, zorder=1.5)
    boundaries = LineCollection([], colors=style.TEXT_PRIMARY, linewidths=0.9, alpha=0.75, zorder=2)
    ax.add_collection(boundaries)
    trails = draw.TrailLayer(ax, positions.shape[1], colors=style.ORANGE, length=400, alpha=0.4, width=1.2)
    centroid_marks = ax.scatter([], [], marker="+", s=36, color=style.TEXT_PRIMARY, linewidths=1.1, zorder=6)
    robots = draw.RobotLayer(ax, positions[0], radius=0.2, colors=style.ORANGE)

    # Right panels: cost and centroid error over time (log scale).
    t_rec = times[:frames]
    ax_cost = fig.add_axes((0.72, 0.6, 0.26, 0.22))
    ax_cost.set_yscale("log")
    ax_cost.set_xlim(0, t_rec[-1])
    ax_cost.set_ylim(cost[:frames].min() * 0.7, cost[:frames].max() * 1.4)
    ax_cost.set_title("coverage cost H(P)", loc="left", fontsize=10)
    ax_cost.grid(True, axis="y", which="major")
    (cost_line,) = ax_cost.plot([], [], color=style.BLUE, linewidth=2.0)
    ax_err = fig.add_axes((0.72, 0.27, 0.26, 0.22))
    ax_err.set_yscale("log")
    ax_err.set_xlim(0, t_rec[-1])
    ax_err.set_ylim(max(error[:frames].min() * 0.5, 1e-3), error[:frames].max() * 1.5)
    ax_err.set_title("max ||p_i - C_i||  [m]", loc="left", fontsize=10)
    ax_err.set_xlabel("time [s]")
    ax_err.grid(True, axis="y", which="major")
    tol = result["config"].get("convergence_tolerance", 0.05)
    ax_err.axhline(tol, color=style.TEXT_SECONDARY, linewidth=1.0, linestyle=(0, (4, 3)))
    ax_err.text(
        t_rec[-1], tol * 1.15, "tolerance", ha="right", va="bottom", fontsize=8, color=style.TEXT_SECONDARY
    )
    (err_line,) = ax_err.plot([], [], color=style.BLUE, linewidth=2.0)
    status = fig.text(0.72, 0.875, "", fontsize=10, color=style.TEXT_SECONDARY)
    fig.legend(
        handles=[
            *draw.legend_handles(
                [("robot", style.ORANGE, "disc"), ("Voronoi boundary", style.TEXT_PRIMARY, "line")]
            ),
            Line2D(
                [],
                [],
                marker="+",
                linestyle="",
                color=style.TEXT_PRIMARY,
                markersize=7,
                label="cell centroid C_i",
            ),
        ],
        loc="lower left",
        bbox_to_anchor=(0.705, 0.0),
        ncol=1,
        fontsize=8.5,
        handletextpad=0.4,
        labelspacing=0.3,
    )

    rec = GifRecorder(fig, fps=16, max_colors=64)
    for f in frame_indices(frames, max_frames):
        p = positions[f]
        boundaries.set_segments(_cell_segments(cells[f]))
        if radius is not None:
            links.update(p, _edges_within(p, radius))
        trails.update(positions[: f + 1])
        robots.update(p)
        centroid_marks.set_offsets(np.asarray(centroids[f]))
        cost_line.set_data(t_rec[: f + 1], cost[: f + 1])
        err_line.set_data(t_rec[: f + 1], error[: f + 1])
        status.set_text(f"t = {t_rec[f]:5.1f} s")
        rec.capture(repeat=10 if f == 0 else 1)
    settle = run["settling_time"]
    if settle is not None and settle >= 0:
        status.set_text(f"settled below {tol:g} m at t = {settle:.1f} s")
    else:
        status.set_text(f"error > {tol:g} m at the end of the run")
    rec.capture(repeat=20)
    path = rec.save(out, hold_last_seconds=1.5)
    pipeline.report(path, rec.num_frames)
    return path


def main() -> None:
    args = pipeline.render_arguments(__doc__).parse_args()
    result, _ = pipeline.obtain_result(args, "coverage", CONFIG)
    seed = result["config"]["seed"]
    script = "scripts/render/render_voronoi_coverage.py"
    n = len(result["centralized"]["trajectory"]["positions"][0])
    hero = render(
        result,
        "centralized",
        pipeline.ASSETS_DIR / "voronoi_coverage.gif",
        "Voronoi coverage control",
        f"{n} robots, gradient flow p_i' = -k (p_i - C_i) toward density-weighted cell centroids",
    )
    pipeline.update_manifest(
        hero, f"{n} robots, 12 x 9 m, 3-bump density", "Lloyd/CVT gradient flow", seed, script, CONFIG
    )
    radius = result["distributed"]["communication_radius"]
    dist = render(
        result,
        "distributed",
        pipeline.ASSETS_DIR / "coverage_distributed.gif",
        "Limited-range Voronoi coverage",
        f"each robot clips its cell only with robots within R = {radius:.1f} m (links shown)",
        max_frames=75,
    )
    pipeline.update_manifest(
        dist, f"{n} robots, R = {radius:.1f} m", "limited-range coverage", seed, script, CONFIG
    )


if __name__ == "__main__":
    main()
