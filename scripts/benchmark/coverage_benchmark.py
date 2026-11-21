"""Coverage benchmark report: runs mrc_experiment_coverage_benchmark and writes
docs/benchmarks/coverage.md plus plots in docs/assets/plots/.
"""

from __future__ import annotations

import argparse
import platform
import statistics
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

import matplotlib.pyplot as plt

from multirobotcore_viz import pipeline, style

CONFIG = "configs/coverage/benchmark.json"
PLOTS = pipeline.ASSETS_DIR / "plots"
DOC = pipeline.REPO_ROOT / "docs" / "benchmarks" / "coverage.md"


def _mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else float("nan")


def scalability_table(rows: list[dict]) -> tuple[str, dict]:
    by_n: dict[int, list[dict]] = defaultdict(list)
    for r in rows:
        by_n[r["robots"]].append(r)
    lines = [
        "| Robots | Converged | Lloyd iterations | Time / iteration [ms] | Initial cost | Final cost "
        "| Mean distance [m] | Max uncovered [m] | Min separation [m] |",
        "|---:|:---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    summary = {}
    for n in sorted(by_n):
        rs = by_n[n]
        conv = sum(r["converged"] for r in rs)
        ms = _mean([r["ms_per_iteration"] for r in rs])
        summary[n] = ms
        lines.append(
            f"| {n} | {conv}/{len(rs)} | {_mean([r['iterations'] for r in rs]):.0f} | {ms:.3f} "
            f"| {_mean([r['initial_cost'] for r in rs]):.0f} | {_mean([r['final_cost'] for r in rs]):.1f} "
            f"| {_mean([r['mean_distance_m'] for r in rs]):.3f} "
            f"| {_mean([r['max_uncovered_m'] for r in rs]):.3f} "
            f"| {_mean([r['min_separation_m'] for r in rs]):.3f} |"
        )
    return "\n".join(lines), summary


def limited_table(study: dict) -> tuple[str, dict]:
    by_f: dict[float, list[dict]] = defaultdict(list)
    for r in study["runs"]:
        by_f[r["radius_factor"]].append(r)
    lines = [
        "| R / spacing | R [m] | Certified-exact cells | Cost / centralized cost "
        "| Max exact-centroid error [m] | Position messages / iteration |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    summary = {}
    for f in sorted(by_f):
        rs = by_f[f]
        summary[f] = (
            _mean([r["cost_ratio"] for r in rs]),
            _mean([r["messages_per_iteration"] for r in rs]),
        )
        certified = 100 * _mean([r["certified_fraction"] for r in rs])
        lines.append(
            f"| {f:.1f} | {rs[0]['radius_m']:.2f} | {certified:.0f} % "
            f"| {summary[f][0]:.3f} | {_mean([r['exact_centroid_error_m'] for r in rs]):.3f} "
            f"| {summary[f][1]:.0f} |"
        )
    return "\n".join(lines), summary


def plot_scaling(summary: dict, path: Path) -> None:
    style.apply_style()
    fig, ax = plt.subplots(figsize=(5.2, 3.2), dpi=110)
    ns = sorted(summary)
    ax.plot(ns, [summary[n] for n in ns], color=style.BLUE, marker="o", markersize=6)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xticks(ns, [str(n) for n in ns])
    ax.set_xlabel("robots")
    ax.set_ylabel("ms per Lloyd iteration")
    ax.set_title("Voronoi partition + centroids, one iteration", loc="left", fontsize=10)
    ax.grid(True, which="major", axis="y")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_limited(summary: dict, path: Path) -> None:
    style.apply_style()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.4, 3.0), dpi=110)
    fs = sorted(summary)
    ax1.plot(fs, [summary[f][0] for f in fs], color=style.BLUE, marker="o", markersize=6)
    ax1.axhline(1.0, color=style.TEXT_SECONDARY, linewidth=1.0, linestyle=(0, (4, 3)))
    ax1.set_xlabel("R / nominal spacing")
    ax1.set_title("final cost / centralized cost", loc="left", fontsize=10)
    ax1.grid(True, axis="y")
    ax2.plot(fs, [summary[f][1] for f in fs], color=style.BLUE, marker="o", markersize=6)
    ax2.set_xlabel("R / nominal spacing")
    ax2.set_title("position messages per iteration", loc="left", fontsize=10)
    ax2.grid(True, axis="y")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=None)
    parser.add_argument("--result", type=Path, default=None)
    args = parser.parse_args()
    if args.result is not None:
        result = pipeline.load_result(args.result)
    else:
        result = pipeline.run_executable(
            "mrc_experiment_coverage_benchmark",
            args.build_dir,
            pipeline.REPO_ROOT / CONFIG,
            pipeline.RESULTS_DIR / "coverage_benchmark.json",
        )
    PLOTS.mkdir(parents=True, exist_ok=True)
    scal_md, scal = scalability_table(result["scalability"])
    lim_md, lim = limited_table(result["limited_range"])
    plot_scaling(scal, PLOTS / "coverage_scaling.png")
    plot_limited(lim, PLOTS / "coverage_limited_range.png")
    study = result["limited_range"]
    DOC.parent.mkdir(parents=True, exist_ok=True)
    DOC.write_text(
        f"""# Coverage benchmark

Generated by `python scripts/benchmark/coverage_benchmark.py` from
`mrc_experiment_coverage_benchmark` ({CONFIG}). Release build, single thread,
{platform.system()} {platform.machine()}. Values are means over seeds
{result["config"]["seeds"]}.

Setup: {result["domain_side_m"]:.0f} m x {result["domain_side_m"]:.0f} m square, two-bump Gaussian
density with a constant floor, robots initially uniform at random. Discrete
Lloyd iterations (every robot moves to its density-weighted cell centroid)
until the largest centroid distance falls below 1 % of the nominal cell size
`side / sqrt(N)`, or {result["config"]["max_iterations"]} iterations.

## Scalability

{scal_md}

![runtime per Lloyd iteration](../assets/plots/coverage_scaling.png)

Each cell sorts the other N - 1 generators by distance (O(N log N) per cell,
O(N^2 log N) per partition) but clips against only the few nearest ones,
because clipping stops once no farther bisector can intersect the cell;
quadrature is O(N) with a constant set by the subdivision level. Over this
range the measured time per iteration grows roughly like N^1.4. A spatial
index (k-d tree or Delaunay neighbours) would remove the quadratic sorting
term for much larger teams.

## Limited communication range

{study["robots"]} robots, {study["iterations"]} Lloyd iterations of the r-limited controller (each
robot uses its Voronoi cell intersected with B(p_i, R/2)), compared with the
centralized partition from the same start. Nominal spacing
`side / sqrt(N)` = {study["nominal_spacing_m"]:.2f} m.

{lim_md}

![limited-range study](../assets/plots/coverage_limited_range.png)

With R of about twice the spacing every cell fits in its R/2 disc, the local
computation is exact and the result matches the centralized optimum at about
half the message load of R = 3x spacing. With R equal to the spacing, robots
optimise the limited-range cost: they stop spreading once their R/2 discs are
well placed, which leaves a visibly higher coverage cost (a known property of
the limited-range objective, not a numerical failure).
""",
        encoding="utf-8",
    )
    print(f"wrote {DOC.relative_to(pipeline.REPO_ROOT)}")


if __name__ == "__main__":
    main()
