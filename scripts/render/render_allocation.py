"""Renders the task-allocation animations from mrc_demo_allocation output.

Outputs (docs/assets/):
  hungarian.gif            candidate pairs -> greedy vs. optimal -> execution
  auction.gif              synchronous auction rounds: bids, prices, owners
  allocation_dynamic.gif   arrivals, cancellations, completions, re-planning
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from matplotlib.collections import LineCollection

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from multirobotcore_viz import draw, pipeline, style
from multirobotcore_viz.animation import GifRecorder, frame_indices

CONFIG = "configs/allocation/allocation.json"
TASK = style.AQUA


def _field_axes(fig, rect, field):
    (x0, y0), (x1, y1) = field["min"], field["max"]
    return style.world_axes(fig, rect, (x0 - 0.3, x1 + 0.3), (y0 - 0.3, y1 + 0.3))


def _pairs(robots, tasks, assignment):
    return [(robots[i], tasks[t]) for i, t in enumerate(assignment) if t is not None]


def render_hungarian(result: dict, out: Path) -> Path:
    robots = np.asarray(result["robots"])
    tasks = np.asarray(result["tasks"])
    hung = result["hungarian"]
    greedy = result["greedy"]
    traj = np.asarray(hung["execution"]["trajectory"]["positions"])
    fig = style.new_figure(960, 640)
    style.figure_title(
        fig,
        "Optimal task assignment (Hungarian method)",
        "minimise total travel distance: every robot gets exactly one task",
    )
    ax = _field_axes(fig, (0.03, 0.08, 0.94, 0.78), result["field"])
    candidates = LineCollection(
        [(r, t) for r in robots for t in tasks], colors=style.LINK, linewidths=0.6, alpha=0.5, zorder=1
    )
    ax.add_collection(candidates)
    greedy_lines = LineCollection(
        [], colors=style.TEXT_SECONDARY, linewidths=1.4, linestyles=(0, (3, 3)), zorder=2
    )
    ax.add_collection(greedy_lines)
    opt_lines = LineCollection([], colors=style.BLUE, linewidths=2.2, zorder=2.5)
    ax.add_collection(opt_lines)
    task_marks = ax.scatter(
        tasks[:, 0],
        tasks[:, 1],
        marker="s",
        s=150,
        color=TASK,
        edgecolors=style.SURFACE,
        linewidths=1.2,
        zorder=3,
    )
    for j, t in enumerate(tasks):
        ax.text(t[0], t[1] - 0.48, f"T{j}", ha="center", va="top", fontsize=8, color=style.TEXT_SECONDARY)
    robot_layer = draw.RobotLayer(ax, robots, radius=0.3, labels=[str(i) for i in range(len(robots))])
    caption = fig.text(0.03, 0.035, "", fontsize=10.5, color=style.TEXT_PRIMARY)
    handles = draw.legend_handles(
        [
            ("robot", style.BLUE, "disc"),
            ("task", TASK, "disc"),
            ("greedy nearest pair", style.TEXT_SECONDARY, "line"),
            ("optimal assignment", style.BLUE, "line"),
        ]
    )
    fig.legend(handles=handles, loc="lower right", ncol=4, bbox_to_anchor=(0.98, 0.0))
    rec = GifRecorder(fig, fps=15)

    caption.set_text("all robot-task pairs are candidates")
    rec.capture(repeat=22)
    candidates.set_alpha(0.18)
    greedy_lines.set_segments(_pairs(robots, tasks, greedy["assignment"]))
    caption.set_text(f"greedy nearest-pair assignment: total {greedy['cost']:.2f} m")
    rec.capture(repeat=26)
    opt_lines.set_segments(_pairs(robots, tasks, hung["assignment"]))
    caption.set_text(
        f"Hungarian optimum: total {hung['cost']:.2f} m  "
        f"({100 * (1 - hung['cost'] / greedy['cost']):.0f}% shorter than greedy)"
    )
    rec.capture(repeat=30)
    greedy_lines.set_segments([])
    candidates.set_segments([])
    reached = np.zeros(len(tasks), dtype=bool)
    for k in frame_indices(len(traj), 110):
        positions = traj[k]
        segs = []
        for i, t in enumerate(hung["assignment"]):
            if t is None:
                continue
            if np.linalg.norm(positions[i] - tasks[t]) < 0.12:
                reached[t] = True
            else:
                segs.append((positions[i], tasks[t]))
        opt_lines.set_segments(segs)
        task_marks.set_facecolors([style.STATUS_GOOD if r else TASK for r in reached])
        robot_layer.update(positions)
        caption.set_text(
            f"execution: {int(reached.sum())}/{len(tasks)} tasks reached, "
            f"{hung['execution']['metrics']['robot_collisions']} robot contacts"
        )
        rec.capture()
    path = rec.save(out, hold_last_seconds=2.0)
    pipeline.report(path, rec.num_frames)
    return path


def render_auction(result: dict, out: Path) -> Path:
    robots = np.asarray(result["robots"])
    tasks = np.asarray(result["tasks"])
    auction = result["auction"]
    rounds = auction["trace"]
    fig = style.new_figure(1000, 560)
    style.figure_title(
        fig,
        "Auction-based task allocation (Bertsekas)",
        f"synchronous rounds: unassigned robots bid, prices rise by at least eps = {auction['epsilon']:.1f}",
    )
    ax = _field_axes(fig, (0.02, 0.1, 0.6, 0.74), result["field"])
    owner_lines = LineCollection([], colors=style.BLUE, linewidths=2.0, zorder=2)
    bid_lines = LineCollection([], colors=style.ORANGE, linewidths=1.4, linestyles=(0, (2, 2)), zorder=2.2)
    ax.add_collection(owner_lines)
    ax.add_collection(bid_lines)
    ax.scatter(
        tasks[:, 0],
        tasks[:, 1],
        marker="s",
        s=150,
        color=TASK,
        edgecolors=style.SURFACE,
        linewidths=1.2,
        zorder=3,
    )
    for j, t in enumerate(tasks):
        ax.text(t[0], t[1] - 0.48, f"T{j}", ha="center", va="top", fontsize=8, color=style.TEXT_SECONDARY)
    robot_layer = draw.RobotLayer(ax, robots, radius=0.3, labels=[str(i) for i in range(len(robots))])

    # Price panel: relative price increase per task (benefit units).
    bx = fig.add_axes((0.68, 0.16, 0.29, 0.62))
    base = np.asarray(rounds[0]["prices"]) * 0.0
    finals = np.asarray(rounds[-1]["prices"])
    bars = bx.barh(np.arange(len(tasks)), base, color=TASK, height=0.6)
    bx.set_yticks(np.arange(len(tasks)), [f"T{j}" for j in range(len(tasks))])
    bx.invert_yaxis()
    bx.set_xlim(0, finals.max() * 1.08 if finals.max() > 0 else 1)
    bx.set_xlabel("price")
    bx.set_title("task prices", loc="left", fontsize=10)
    bx.grid(True, axis="x")
    caption = fig.text(0.02, 0.035, "", fontsize=10.5, color=style.TEXT_PRIMARY)
    handles = draw.legend_handles(
        [("bid this round", style.ORANGE, "line"), ("current owner", style.BLUE, "line")]
    )
    fig.legend(handles=handles, loc="lower right", ncol=2, bbox_to_anchor=(0.98, 0.0))
    rec = GifRecorder(fig, fps=10)
    rec.capture(repeat=8)
    for k, rnd in enumerate(rounds):
        bid_lines.set_segments([(robots[r], tasks[t]) for r, t, _ in rnd["bids"]])
        owner_lines.set_segments([(robots[r], tasks[t]) for t, r in enumerate(rnd["owner"]) if r is not None])
        assigned = {r for r in rnd["owner"] if r is not None}
        robot_layer.update(
            robots, colors=[style.BLUE if i in assigned else style.FAILED for i in range(len(robots))]
        )
        for bar, p in zip(bars, rnd["prices"], strict=True):
            bar.set_width(p)
        caption.set_text(
            f"round {k + 1}/{len(rounds)}: {len(rnd['bids'])} bids, "
            f"{len(assigned)}/{len(robots)} robots assigned (gray: unassigned)"
        )
        rec.capture(repeat=6)
    bid_lines.set_segments([])
    caption.set_text(
        f"auction cost {auction['cost']:.2f} m vs. optimum "
        f"{result['hungarian']['cost']:.2f} m (bound n*eps = "
        f"{len(robots) * auction['epsilon']:.1f}); {auction['messages']} messages"
    )
    rec.capture(repeat=30)
    path = rec.save(out, hold_last_seconds=1.0)
    pipeline.report(path, rec.num_frames)
    return path


def render_dynamic(result: dict, out: Path) -> Path:
    data = result["dynamic"]
    traj = np.asarray(data["trajectory"]["positions"])
    times = np.asarray(data["trajectory"]["t"])
    tasks = {t["id"]: np.asarray(t["position"]) for t in data["tasks"]}
    fig = style.new_figure(960, 640)
    style.figure_title(
        fig,
        "Dynamic task allocation",
        "tasks arrive and are cancelled during execution; re-planning with commitment "
        "and a switching penalty",
    )
    ax = _field_axes(fig, (0.03, 0.08, 0.94, 0.78), result["field"])
    lines = LineCollection([], colors=style.BLUE, linewidths=1.8, zorder=2)
    ax.add_collection(lines)
    open_marks = ax.scatter([], [], marker="s", s=150, color=TASK, edgecolors=style.SURFACE, zorder=3)
    done_marks = ax.scatter([], [], marker="s", s=110, color=style.STATUS_GOOD, alpha=0.55, zorder=2.8)
    cancel_marks = ax.scatter([], [], marker="x", s=90, color=style.STATUS_CRITICAL, linewidths=2, zorder=3)
    robot_layer = draw.RobotLayer(ax, traj[0], radius=0.3, labels=[str(i) for i in range(traj.shape[1])])
    caption = fig.text(0.03, 0.035, "", fontsize=10.5, color=style.TEXT_PRIMARY)
    handles = draw.legend_handles(
        [
            ("open task", TASK, "disc"),
            ("completed", style.STATUS_GOOD, "disc"),
            ("cancelled", style.STATUS_CRITICAL, "disc"),
        ]
    )
    fig.legend(handles=handles, loc="lower right", ncol=3, bbox_to_anchor=(0.98, 0.0))
    events = data["events"]
    rec = GifRecorder(fig, fps=20)
    for k in frame_indices(len(times), 220):
        t = times[k]
        open_ids = data["open_tasks"][k]
        done = [tasks[e["task"]] for e in events if e["type"] == "completed" and e["time"] <= t]
        cancelled = [tasks[e["task"]] for e in events if e["type"] == "cancelled" and e["time"] <= t]
        open_marks.set_offsets(np.asarray([tasks[i] for i in open_ids]) if open_ids else np.empty((0, 2)))
        done_marks.set_offsets(np.asarray(done) if done else np.empty((0, 2)))
        cancel_marks.set_offsets(np.asarray(cancelled) if cancelled else np.empty((0, 2)))
        segs = [(traj[k][i], tasks[tid]) for i, tid in enumerate(data["robot_tasks"][k]) if tid is not None]
        lines.set_segments(segs)
        robot_layer.update(traj[k])
        recent = [e for e in events if e["type"] in ("arrival", "cancelled") and t - 1.5 <= e["time"] <= t]
        note = f"   new: task {recent[-1]['task']} {recent[-1]['type']}" if recent else ""
        caption.set_text(f"t = {t:4.1f} s   completed {len(done)}   open {len(open_ids)}{note}")
        rec.capture()
    path = rec.save(out, hold_last_seconds=1.5)
    pipeline.report(path, rec.num_frames)
    return path


def main() -> None:
    args = pipeline.render_arguments(__doc__).parse_args()
    result, _ = pipeline.obtain_result(args, "allocation", CONFIG)
    seed = result["config"]["seed"]
    script = "scripts/render/render_allocation.py"
    assets = pipeline.ASSETS_DIR
    outputs = [
        (render_hungarian(result, assets / "hungarian.gif"), "8 robots, 8 tasks", "Hungarian assignment"),
        (render_auction(result, assets / "auction.gif"), "8 robots, 8 tasks", "Jacobi auction"),
        (
            render_dynamic(result, assets / "allocation_dynamic.gif"),
            "6 robots, arrivals + cancellation",
            "dynamic reallocation with hysteresis",
        ),
    ]
    for path, scenario, algorithm in outputs:
        pipeline.update_manifest(path, scenario, algorithm, seed, script, CONFIG)


if __name__ == "__main__":
    main()
