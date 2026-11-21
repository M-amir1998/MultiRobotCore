"""Renders the distributed-optimisation animation from
mrc_demo_distributed_optimization output.

Output: docs/assets/distributed_optimization.gif - cooperative bearing-only
localisation: each robot's estimate (left) and the error of ADMM vs. DGD
(right, log scale).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from multirobotcore_viz import draw, pipeline, style
from multirobotcore_viz.animation import GifRecorder

CONFIG = "configs/optimization/distributed_optimization.json"


def render(result: dict, out: Path) -> Path:
    robots = np.asarray(result["robots"])
    target = np.asarray(result["target"])
    optimum = np.asarray(result["optimum"])
    bearings = np.asarray(result["bearings"])
    field = result["field"]
    admm = result["admm"]
    dgd = result["dgd_diminishing"]
    admm_traj = np.asarray(admm["trajectory"])
    dgd_traj = np.asarray(dgd["trajectory"])
    steps = min(len(admm_traj), len(dgd_traj))

    fig = style.new_figure(1000, 520)
    style.figure_title(
        fig,
        "Distributed optimisation: cooperative bearing-only localisation",
        "each robot sees only its own line of bearing; neighbours exchange estimates",
    )
    (x0, y0), (x1, y1) = field["min"], field["max"]
    ax = style.world_axes(fig, (0.02, 0.06, 0.52, 0.78), (x0, x1), (y0, y1))
    links = draw.EdgeLayer(ax, width=0.8)
    links.update(robots, result["graph"]["edges"])
    span = max(x1 - x0, y1 - y0) * 1.5
    for p, b in zip(robots, bearings, strict=True):
        end = p + span * np.array([np.cos(b), np.sin(b)])
        ax.plot([p[0], end[0]], [p[1], end[1]], color=style.YELLOW, linewidth=0.8, alpha=0.6, zorder=1.5)
    ax.scatter(*target, marker="*", s=220, color=style.RED, zorder=6, edgecolors=style.SURFACE)
    ax.scatter(*optimum, marker="+", s=160, color=style.TEXT_PRIMARY, linewidths=1.6, zorder=7)
    draw.RobotLayer(ax, robots, radius=0.35, labels=[str(i) for i in range(len(robots))])
    admm_marks = ax.scatter(
        [], [], s=26, color=style.BLUE, zorder=5, edgecolors=style.SURFACE, linewidths=0.6
    )
    dgd_marks = ax.scatter(
        [], [], s=26, color=style.ORANGE, zorder=5, marker="s", edgecolors=style.SURFACE, linewidths=0.6
    )
    handles = draw.legend_handles(
        [
            ("true target", style.RED, "disc"),
            ("ADMM estimates", style.BLUE, "disc"),
            ("DGD estimates", style.ORANGE, "disc"),
            ("line of bearing", style.YELLOW, "line"),
        ]
    )
    ax.legend(handles=handles, loc="lower left", fontsize=8, ncol=2)

    ex = fig.add_axes((0.62, 0.14, 0.35, 0.66))
    ex.set_yscale("log")
    ex.set_xlim(0, steps - 1)
    a_err = np.maximum(np.asarray(admm["solution_error"]), 1e-9)
    d_err = np.maximum(np.asarray(dgd["solution_error"]), 1e-9)
    ex.set_ylim(min(a_err[:steps].min(), d_err[:steps].min()) * 0.5, max(a_err[0], d_err[0]) * 2)
    ex.set_xlabel("communication round")
    ex.set_ylabel("max_i ||x_i - x*|| [m]")
    ex.grid(True, axis="y")
    (a_line,) = ex.plot([], [], color=style.BLUE, linewidth=2.0, label="decentralised ADMM")
    (d_line,) = ex.plot([], [], color=style.ORANGE, linewidth=2.0, label="DGD (diminishing step)")
    ex.legend(loc="upper right")
    ex.set_title("distance to the centralised optimum", loc="left", fontsize=10)
    status = fig.text(0.62, 0.86, "", fontsize=10, color=style.TEXT_SECONDARY)

    rec = GifRecorder(fig, fps=15)
    for k in range(steps):
        admm_marks.set_offsets(admm_traj[k])
        dgd_marks.set_offsets(dgd_traj[k])
        a_line.set_data(np.arange(k + 1), a_err[: k + 1])
        d_line.set_data(np.arange(k + 1), d_err[: k + 1])
        status.set_text(f"round {k:3d}   ADMM {a_err[k]:.2e} m   DGD {d_err[k]:.2e} m")
        rec.capture(repeat=8 if k == 0 else 1)
    status.set_text(
        f"ADMM: {admm['iterations']} rounds to 1e-4; DGD: {dgd['iterations']} rounds, "
        f"{'converged' if dgd['converged'] else 'still converging'}"
    )
    rec.capture(repeat=25)
    path = rec.save(out, hold_last_seconds=1.0)
    pipeline.report(path, rec.num_frames)
    return path


def main() -> None:
    args = pipeline.render_arguments(__doc__).parse_args()
    result, _ = pipeline.obtain_result(args, "distributed_optimization", CONFIG)
    path = render(result, pipeline.ASSETS_DIR / "distributed_optimization.gif")
    pipeline.update_manifest(
        path,
        "10 robots, noisy bearings, disk graph",
        "DGD + decentralised consensus ADMM",
        result["config"]["seed"],
        "scripts/render/render_distributed_optimization.py",
        CONFIG,
    )


if __name__ == "__main__":
    main()
