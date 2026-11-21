"""Renders the consensus animations from mrc_demo_consensus output.

Outputs (docs/assets/):
  consensus.gif            scalar average consensus: network state + trajectories
  consensus_partition.gif  link failures split the network, recovery re-merges it
  rendezvous.gif           planar rendezvous of simulated robots on a disk graph
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from multirobotcore_viz import draw, pipeline, style
from multirobotcore_viz.animation import GifRecorder, frame_indices

CONFIG = "configs/consensus/consensus.json"


def _graph_panel(fig, rect, layout, edges, values, vmin, vmax, cut=None):
    ax = style.world_axes(fig, rect, (-1.35, 1.35), (-1.3, 1.3), frame=False)
    link_layer = draw.EdgeLayer(ax, width=1.3)
    link_layer.update(layout, edges)
    cut_layer = draw.EdgeLayer(ax, color=style.STATUS_CRITICAL, width=1.6, linestyle="--", zorder=2.2)
    colors = [style.sequential_color(v, vmin, vmax) for v in values]
    robots = draw.RobotLayer(ax, layout, radius=0.12, colors=colors)
    labels = [
        ax.text(
            p[0] * 1.24, p[1] * 1.24, "", ha="center", va="center", fontsize=8, color=style.TEXT_SECONDARY
        )
        for p in layout
    ]
    return ax, robots, labels, link_layer, cut_layer


def render_scalar(result: dict, out: Path) -> Path:
    data = result["scalar"]
    values = np.asarray(data["values"])  # (K + 1, N)
    layout = np.asarray(data["layout"])
    edges = data["graph"]["edges"]
    vmin, vmax = float(values.min()), float(values.max())
    k_max = values.shape[0] - 1

    fig = style.new_figure(960, 480)
    style.figure_title(
        fig,
        "Average consensus",
        f"x(k+1) = (I - eps L) x(k),  eps = 0.8 / d_max,  lambda_2 = {data['algebraic_connectivity']:.2f}",
    )
    _, robots, labels, _, _ = _graph_panel(fig, (0.02, 0.04, 0.42, 0.8), layout, edges, values[0], vmin, vmax)

    ax = fig.add_axes((0.53, 0.14, 0.44, 0.66))
    ax.set_xlim(0, k_max)
    ax.set_ylim(vmin - 0.4, vmax + 0.4)
    ax.set_xlabel("iteration k")
    ax.set_ylabel("agent state")
    ax.grid(True, axis="y")
    order = np.argsort(values[0])
    line_colors = {i: style.sequential_color(values[0, i], vmin, vmax) for i in range(values.shape[1])}
    lines = [ax.plot([], [], color=line_colors[i], linewidth=1.6)[0] for i in order]
    ax.axhline(data["average"], color=style.TEXT_SECONDARY, linewidth=1.0, linestyle=(0, (4, 3)))
    ax.text(
        k_max,
        data["average"] + 0.18,
        "initial average (preserved)",
        ha="right",
        va="bottom",
        fontsize=8.5,
        color=style.TEXT_SECONDARY,
    )
    status = fig.text(0.53, 0.86, "", fontsize=10, color=style.TEXT_SECONDARY)

    rec = GifRecorder(fig, fps=12)
    for k in range(k_max + 1):
        colors = [style.sequential_color(v, vmin, vmax) for v in values[k]]
        robots.update(layout, colors=colors)
        for label, v in zip(labels, values[k], strict=True):
            label.set_text(f"{v:.1f}")
        for line, i in zip(lines, order, strict=True):
            line.set_data(np.arange(k + 1), values[: k + 1, i])
        status.set_text(f"k = {k:2d}    disagreement max|x_i - avg| = {data['disagreement'][k]:.3f}")
        rec.capture(repeat=6 if k == 0 else 1)
    path = rec.save(out, hold_last_seconds=2.0)
    pipeline.report(path, rec.num_frames)
    return path


def render_partition(result: dict, out: Path) -> Path:
    data = result["partition"]
    values = np.asarray(data["values"])
    layout = np.asarray(data["layout"])
    edges = data["graph"]["edges"]
    cut_links = data["cut_links"]
    cut_k, restore_k = data["cut_round"], data["restore_round"]
    vmin, vmax = float(values.min()), float(values.max())
    k_max = values.shape[0] - 1
    cut_set = {tuple(sorted(link)) for link in cut_links}
    live_edges = [e for e in edges if tuple(sorted(e[:2])) not in cut_set]

    fig = style.new_figure(960, 480)
    style.figure_title(
        fig,
        "Consensus under link failure and recovery",
        "messages through the network simulator: 5% random loss + two link outages",
    )
    _, robots, labels, link_layer, cut_layer = _graph_panel(
        fig, (0.02, 0.04, 0.42, 0.8), layout, edges, values[0], vmin, vmax
    )

    ax = fig.add_axes((0.53, 0.14, 0.44, 0.66))
    ax.set_xlim(0, k_max)
    ax.set_ylim(vmin - 0.4, vmax + 0.4)
    ax.set_xlabel("communication round")
    ax.set_ylabel("agent state")
    ax.axvspan(cut_k, restore_k, color=style.STATUS_CRITICAL, alpha=0.07, linewidth=0)
    ax.text(
        (cut_k + restore_k) / 2,
        vmax + 0.25,
        "links down: network partitioned",
        ha="center",
        va="top",
        fontsize=8.5,
        color=style.STATUS_CRITICAL,
    )
    ax.axhline(data["average"], color=style.TEXT_SECONDARY, linewidth=1.0, linestyle=(0, (4, 3)))
    ax.text(
        k_max,
        data["average"] + 0.15,
        "initial average",
        fontsize=8.5,
        color=style.TEXT_SECONDARY,
        va="bottom",
        ha="right",
    )
    lines = [
        ax.plot([], [], color=style.sequential_color(values[0, i], vmin, vmax), linewidth=1.5)[0]
        for i in range(values.shape[1])
    ]
    status = fig.text(0.53, 0.86, "", fontsize=10, color=style.TEXT_SECONDARY)

    rec = GifRecorder(fig, fps=15)
    for k in range(k_max + 1):
        down = cut_k <= k < restore_k
        link_layer.update(layout, live_edges if down else edges)
        cut_layer.update(layout, cut_links if down else [])
        robots.update(layout, colors=[style.sequential_color(v, vmin, vmax) for v in values[k]])
        for label, v in zip(labels, values[k], strict=True):
            label.set_text(f"{v:.1f}")
        for i, line in enumerate(lines):
            line.set_data(np.arange(k + 1), values[: k + 1, i])
        phase = (
            "partitioned: two clusters agree separately"
            if down
            else ("connected" if k < cut_k else "links restored: global agreement")
        )
        status.set_text(f"round {k:3d}   {phase}")
        rec.capture()
    final = values[-1].mean()
    status.set_text(
        f"agreement {final:.2f} vs. initial average {data['average']:.2f}: losses bias the result"
    )
    rec.capture(repeat=30)
    path = rec.save(out, hold_last_seconds=1.0)
    pipeline.report(path, rec.num_frames)
    return path


def render_rendezvous(result: dict, out: Path) -> Path:
    data = result["rendezvous"]
    run = data["separation"]
    traj = run["trajectory"]
    positions = np.asarray(traj["positions"])  # (T, N, 2)
    times = np.asarray(traj["t"])
    edges = run["edges"]
    arena = data["arena"]
    pure = data["pure"]["metrics"]

    fig = style.new_figure(720, 640)
    style.figure_title(
        fig,
        "Rendezvous on a proximity graph",
        f"consensus cohesion + short-range separation, {positions.shape[1]} robots, "
        f"comm. radius {data['comm_radius']:.1f} m",
    )
    ax = style.world_axes(fig, (0.04, 0.03, 0.92, 0.82), (-0.3, arena + 0.3), (-0.3, arena + 0.3))
    links = draw.EdgeLayer(ax)
    trails = draw.TrailLayer(ax, positions.shape[1], length=200, alpha=0.35)
    robots = draw.RobotLayer(
        ax,
        positions[0],
        radius=data["robot_radius"] * 1.6,
        labels=[str(i) for i in range(positions.shape[1])],
        label_size=6.5,
    )
    ax.scatter(*positions[0].T, s=10, color=style.TEXT_SECONDARY, alpha=0.6, zorder=2.5, linewidths=0)
    status = ax.text(0.2, arena - 0.1, "", fontsize=9.5, color=style.TEXT_SECONDARY, va="top")
    note = ax.text(0.2, 0.15, "", fontsize=8.5, color=style.TEXT_SECONDARY, va="bottom")

    rec = GifRecorder(fig, fps=20)
    for f in frame_indices(len(times), 140):
        links.update(positions[f], edges[f])
        trails.update(positions[: f + 1])
        robots.update(positions[f])
        status.set_text(f"t = {times[f]:5.2f} s    spread = {run['spread'][f]:.2f} m")
        rec.capture(repeat=8 if f == 0 else 1)
    m = run["metrics"]
    note.set_text(
        f"robot contacts: {m['robot_collisions']} with separation, "
        f"{pure['robot_collisions']} with pure consensus (point agreement)"
    )
    rec.capture(repeat=30)
    path = rec.save(out, hold_last_seconds=1.0)
    pipeline.report(path, rec.num_frames)
    return path


def main() -> None:
    parser = pipeline.render_arguments(__doc__)
    args = parser.parse_args()
    result, _ = pipeline.obtain_result(args, "consensus", CONFIG)
    seed = result["config"]["seed"]
    script = "scripts/render/render_consensus.py"
    assets = pipeline.ASSETS_DIR
    outputs = [
        (
            render_scalar(result, assets / "consensus.gif"),
            "ring with chords, 12 agents",
            "discrete average consensus",
        ),
        (
            render_partition(result, assets / "consensus_partition.gif"),
            "ring, link outages + 5% loss",
            "message-passing consensus",
        ),
        (
            render_rendezvous(result, assets / "rendezvous.gif"),
            "12 robots, disk graph",
            "rendezvous (consensus + separation)",
        ),
    ]
    for path, scenario, algorithm in outputs:
        pipeline.update_manifest(path, scenario, algorithm, seed, script, CONFIG)


if __name__ == "__main__":
    main()
