"""Orchestration: locate C++ executables, run demos, maintain the asset manifest.

Render scripts follow one pipeline:

    C++ demo executable --(JSON result)--> Python renderer --> docs/assets/*.gif

so every animation is reproducible from (configuration, seed, commit).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
ASSETS_DIR = REPO_ROOT / "docs" / "assets"
RESULTS_DIR = REPO_ROOT / "results"
MANIFEST = ASSETS_DIR / "manifest.json"
BUILD_CANDIDATES = ("release", "ci-release", "clang-release", "clang-cl-release", "debug", "ci-debug")


def executable_name(stem: str) -> str:
    return f"{stem}.exe" if sys.platform.startswith("win") else stem


def find_executable(stem: str, build_dir: Path | None = None) -> Path:
    """Finds `apps/<stem>` in an explicit build directory or the preset trees."""
    candidates = [build_dir] if build_dir else [REPO_ROOT / "build" / c for c in BUILD_CANDIDATES]
    for base in candidates:
        if base is None:
            continue
        path = Path(base) / "apps" / executable_name(stem)
        if path.exists():
            return path
    searched = ", ".join(str(c) for c in candidates)
    raise FileNotFoundError(f"{stem} not found; build the project first (searched: {searched})")


def run_executable(stem: str, build_dir: Path | None, config: Path | None, output: Path) -> dict:
    """Runs a demo/experiment executable and returns its parsed JSON result."""
    exe = find_executable(stem, build_dir)
    cmd = [str(exe), "--output", str(output)]
    if config is not None:
        cmd += ["--config", str(config)]
    output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(cmd, check=True, cwd=REPO_ROOT)
    return load_result(output)


def load_result(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def render_arguments(description: str) -> argparse.ArgumentParser:
    """Common command line for render scripts."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=None,
        help="CMake build directory containing apps/ (default: search presets)",
    )
    parser.add_argument(
        "--result", type=Path, default=None, help="use an existing result JSON instead of running the demo"
    )
    parser.add_argument("--output", type=Path, default=None, help="output animation path")
    return parser


def obtain_result(args: argparse.Namespace, demo: str, config_rel: str | None = None) -> tuple[dict, Path]:
    """Runs the demo (or loads --result) and returns (result, result_path)."""
    if args.result is not None:
        return load_result(args.result), args.result
    output = RESULTS_DIR / f"{demo}.json"
    config = REPO_ROOT / config_rel if config_rel else None
    return run_executable(f"mrc_demo_{demo}", args.build_dir, config, output), output


def _git_commit() -> str | None:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=REPO_ROOT, check=True, capture_output=True, text=True
        )
        return out.stdout.strip() or None
    except (OSError, subprocess.CalledProcessError):
        return None


def update_manifest(
    asset: Path, scenario: str, algorithm: str, seed: int | None, script: str, config: str | None = None
) -> None:
    """Records how an asset was generated in docs/assets/manifest.json."""
    asset = Path(asset)
    manifest = {"assets": {}}
    if MANIFEST.exists():
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    key = asset.relative_to(ASSETS_DIR).as_posix() if asset.is_relative_to(ASSETS_DIR) else asset.name
    manifest.setdefault("assets", {})[key] = {
        "scenario": scenario,
        "algorithm": algorithm,
        "seed": seed,
        "config": config,
        "script": script,
        "source_commit": _git_commit(),
        "bytes": asset.stat().st_size,
    }
    manifest["assets"] = dict(sorted(manifest["assets"].items()))
    MANIFEST.parent.mkdir(parents=True, exist_ok=True)
    MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def report(path: Path, recorder_frames: int) -> None:
    size_mb = path.stat().st_size / 1e6
    print(
        f"wrote {path.relative_to(REPO_ROOT) if path.is_relative_to(REPO_ROOT) else path} "
        f"({recorder_frames} frames, {size_mb:.2f} MB)"
    )
