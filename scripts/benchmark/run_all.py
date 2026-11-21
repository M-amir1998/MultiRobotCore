"""Runs every experiment and regenerates all benchmark pages and plots.

Usage: python scripts/benchmark/run_all.py [--build-dir build/ci-release]
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REPORTS = [
    ("coordination_report.py", ["--run"]),
    ("mapf_benchmark.py", []),
    ("avoidance_report.py", []),
    ("coverage_benchmark.py", []),
    ("exploration_benchmark.py", []),
]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=None)
    args = parser.parse_args()
    for script, extra in REPORTS:
        cmd = [sys.executable, str(ROOT / "scripts" / "benchmark" / script), *extra]
        if args.build_dir is not None:
            cmd += ["--build-dir", str(args.build_dir)]
        start = time.perf_counter()
        print(f"== {script}", flush=True)
        subprocess.run(cmd, check=True, cwd=ROOT)
        print(f"   done in {time.perf_counter() - start:.1f} s", flush=True)


if __name__ == "__main__":
    main()
