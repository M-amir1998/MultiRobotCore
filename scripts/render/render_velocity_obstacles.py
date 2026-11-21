"""Renders the velocity-space illustration from mrc_demo_avoidance_velocity_space.

Output: docs/assets/velocity_obstacles.gif - left: world view; right: velocity
space of robot 0 with the truncated velocity obstacles of its neighbours, the
ORCA half-plane boundaries, the resulting feasible set inside the speed
limit, and the preferred and selected velocities. All cone and half-plane
parameters are computed in C++; this script only draws them.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch, Patch, Polygon

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from multirobotcore_viz import draw, pipeline, style
from multirobotcore_viz.animation import GifRecorder

CONFIG = "configs/avoidance/velocity_space.json"
VLIM = 1.6  # velocity panel half-width [m/s]
FAR = 6.0  # leg length used to draw (unbounded) cones [m/s]


def cone_polygon(cone: dict) -> np.ndarray | None:
    """Outline of a truncated velocity obstacle: both legs from their tangent
    points outwards, closed by the near arc of the cut-off circle."""
    if cone["overlapping"]:
        return None
    apex = np.asarray(cone["apex"])
    center = np.asarray(cone["cutoff_center"])
    r = cone["cutoff_radius"]
    left = np.asarray(cone["left_leg"])
    right = np.asarray(cone["right_leg"])
    c = center - apex
    tangent = np.sqrt(max(c @ c - r * r, 0.0))
    t_left = apex + tangent * left
    t_right = apex + tangent * right
    a_left = np.arctan2(*(t_left - center)[::-1])
    a_right = np.arctan2(*(t_right - center)[::-1])
    a_near = np.arctan2(*(-c)[::-1])
    # Sweep from the right tangent point to the left one through the near point.
    span = (a_left - a_right) % (2 * np.pi)
    if not 0 <= (a_near - a_right) % (2 * np.pi) <= span:
        span -= 2 * np.pi
    arc = [center + r * np.array([np.cos(a), np.sin(a)]) for a in a_right + np.linspace(0.0, span, 24)]
    return np.array([*arc, apex + FAR * left, apex + FAR * right])


def feasible_region(planes: list[dict], max_speed: float) -> np.ndarray:
    """Speed disc clipped by every ORCA half-plane n . v >= offset."""
    angles = np.linspace(0.0, 2 * np.pi, 97)[:-1]
    poly = [max_speed * np.array([np.cos(a), np.sin(a)]) for a in angles]
    for plane in planes:
        n = np.asarray(plane["normal"])
        off = plane["offset"]
        out = []
        for k, cur in enumerate(poly):
            nxt = poly[(k + 1) % len(poly)]
            dc, dn = n @ cur - off, n @ nxt - off
            if dc >= 0:
                out.append(cur)
            if (dc >= 0) != (dn >= 0):
                out.append(cur + dc / (dc - dn) * (nxt - cur))
        poly = out
        if len(poly) < 3:
            return np.empty((0, 2))
    return np.asarray(poly)


def half_plane_segment(plane: dict) -> np.ndarray:
    n = np.asarray(plane["normal"])
    point = plane["offset"] * n
    d = np.array([-n[1], n[0]])
    return np.array([point - 3 * VLIM * d, point + 3 * VLIM * d])


def render(result: dict, out: Path) -> Path:
    frames = result["frames"]
    focus = result["focus"]
    radius = result["robot_radius"]
    vmax = result["max_speed"]
    goals = np.asarray(result["goals"])
    positions = np.asarray([f["positions"] for f in frames])
    n = positions.shape[1]
    colors = style.CATEGORICAL[:n]

    fig = style.new_figure(1040, 560)
    style.figure_title(
        fig,
        "Velocity obstacles and ORCA in velocity space",
        f"robot {focus}: truncated VOs of its neighbours (tau = "
        f"{result['time_horizon']:.0f} s), ORCA half-planes and the chosen velocity",
    )
    extent = float(np.abs(positions).max()) + 0.8
    world = style.world_axes(
        fig, (0.02, 0.03, 0.46, 0.80), (-extent, extent), (-extent, extent), title="World"
    )
    for g, c in zip(goals, colors, strict=True):
        world.scatter(*g, marker="s", s=40, facecolors="none", edgecolors=c, linewidths=1.3, zorder=2)
    trails = draw.TrailLayer(world, n, colors=colors, length=400, alpha=0.5)
    robots = draw.RobotLayer(
        world, positions[0], radius=radius, colors=colors, labels=[str(i) for i in range(n)], label_size=8
    )
    clock = world.text(-extent + 0.3, -extent + 0.3, "", fontsize=9, color=style.TEXT_SECONDARY)

    vel = style.world_axes(
        fig, (0.52, 0.03, 0.46, 0.80), (-VLIM, VLIM), (-VLIM, VLIM), title=f"Velocity space of robot {focus}"
    )
    vel.axhline(0, color=style.GRID, linewidth=0.8, zorder=0)
    vel.axvline(0, color=style.GRID, linewidth=0.8, zorder=0)
    vel.add_patch(
        Circle((0, 0), vmax, fill=False, edgecolor=style.FRAME, linestyle=(0, (4, 3)), linewidth=1.0)
    )
    vel.text(vmax * 0.72, -vmax * 0.82, "speed limit", fontsize=8, color=style.TEXT_SECONDARY)
    dynamic: list = []

    legend = [
        Patch(facecolor=style.CATEGORICAL[1], alpha=0.25, label="VO of a neighbour"),
        Patch(facecolor=style.BLUE, alpha=0.18, edgecolor=style.BLUE, label="ORCA feasible set"),
        *draw.legend_handles(
            [
                ("ORCA boundary (neighbour colour)", style.AQUA, "line"),
                ("preferred velocity", style.TEXT_SECONDARY, "line"),
                ("selected velocity", style.BLUE, "line"),
            ]
        ),
    ]
    vel.legend(handles=legend, loc="upper left", fontsize=8)

    rec = GifRecorder(fig, fps=15, max_colors=96)
    for k, f in enumerate(frames):
        for artist in dynamic:
            artist.remove()
        dynamic.clear()
        robots.update(positions[k])
        trails.update(positions[: k + 1])
        clock.set_text(f"t = {f['t']:4.1f} s")
        for cone in f["vo"]:
            poly = cone_polygon(cone)
            if poly is None:
                continue
            c = colors[cone["neighbor"]] if cone["is_robot"] else style.OBSTACLE_EDGE
            dynamic.append(
                vel.add_patch(
                    Polygon(poly, closed=True, facecolor=c, alpha=0.16, edgecolor=c, linewidth=0.8, zorder=1)
                )
            )
        for plane in f["orca"]:
            c = colors[plane["neighbor"]] if plane["is_robot"] else style.OBSTACLE_EDGE
            seg = half_plane_segment(plane)
            dynamic.append(vel.plot(seg[:, 0], seg[:, 1], color=c, linewidth=1.4, zorder=3)[0])
        region = feasible_region(f["orca"], vmax)
        if len(region):
            dynamic.append(
                vel.add_patch(
                    Polygon(
                        region,
                        closed=True,
                        facecolor=style.BLUE,
                        alpha=0.14,
                        edgecolor=style.BLUE,
                        linewidth=1.0,
                        zorder=2,
                    )
                )
            )
        pref = np.asarray(f["preferred"][focus])
        sel = np.asarray(f["selected"])
        dynamic.append(
            vel.add_patch(
                FancyArrowPatch(
                    (0, 0),
                    tuple(pref),
                    arrowstyle="-|>",
                    mutation_scale=12,
                    color=style.TEXT_SECONDARY,
                    linewidth=1.4,
                    linestyle="--",
                    zorder=4,
                )
            )
        )
        dynamic.append(
            vel.add_patch(
                FancyArrowPatch(
                    (0, 0),
                    tuple(sel),
                    arrowstyle="-|>",
                    mutation_scale=14,
                    color=style.BLUE,
                    linewidth=2.4,
                    zorder=5,
                )
            )
        )
        rec.capture(repeat=8 if k == 0 else 1)
    path = rec.save(out, hold_last_seconds=2.0)
    pipeline.report(path, rec.num_frames)
    return path


def main() -> None:
    args = pipeline.render_arguments(__doc__).parse_args()
    result, _ = pipeline.obtain_result(args, "avoidance_velocity_space", CONFIG)
    out = args.output or pipeline.ASSETS_DIR / "velocity_obstacles.gif"
    path = render(result, out)
    pipeline.update_manifest(
        path,
        "4 robots crossing",
        "velocity obstacles + ORCA",
        result["config"].get("seed"),
        "scripts/render/render_velocity_obstacles.py",
        CONFIG,
    )


if __name__ == "__main__":
    main()
