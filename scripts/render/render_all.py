"""Re-renders every documentation animation from fresh C++ demo output.

Each render_<name>.py script runs its demo executable, writes the GIF(s) to
docs/assets/ and records provenance in docs/assets/manifest.json.

Usage: python scripts/render/render_all.py [--build-dir build/ci-release] [--only cbs,warehouse]
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=None)
    parser.add_argument("--only", default="", help="comma-separated script names without render_ prefix")
    args = parser.parse_args()
    wanted = {name.strip() for name in args.only.split(",") if name.strip()}
    scripts = sorted(p for p in HERE.glob("render_*.py") if p.name != "render_all.py")
    for script in scripts:
        name = script.stem.removeprefix("render_")
        if wanted and name not in wanted:
            continue
        cmd = [sys.executable, str(script)]
        if args.build_dir is not None:
            cmd += ["--build-dir", str(args.build_dir)]
        start = time.perf_counter()
        print(f"== {script.name}", flush=True)
        subprocess.run(cmd, check=True, cwd=ROOT)
        print(f"   done in {time.perf_counter() - start:.1f} s", flush=True)


if __name__ == "__main__":
    main()
