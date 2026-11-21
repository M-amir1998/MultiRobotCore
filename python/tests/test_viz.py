"""Tests for the multirobotcore_viz package (no C++ build required)."""

from __future__ import annotations

import itertools
import json
import sys
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from multirobotcore_viz import draw, pipeline, plots, style
from multirobotcore_viz.animation import GifRecorder, frame_indices


def test_frame_indices_are_sorted_unique_and_cover_endpoints():
    idx = frame_indices(1000, 50)
    assert idx[0] == 0
    assert idx[-1] == 999
    assert idx == sorted(set(idx))
    assert len(idx) == 50
    assert frame_indices(10, 50) == list(range(10))
    assert frame_indices(0, 5) == []


def test_gif_recorder_writes_looping_gif_with_seeded_palette(tmp_path):
    fig = style.new_figure(160, 120)
    ax = style.world_axes(fig, (0.05, 0.05, 0.9, 0.9), (0, 10), (0, 10))
    layer = draw.RobotLayer(
        ax, np.array([[2.0, 2.0], [8.0, 8.0]]), radius=0.8, colors=[style.BLUE, style.ORANGE]
    )
    rec = GifRecorder(fig, fps=10)
    for k in range(5):
        layer.update(np.array([[2.0 + k, 2.0], [8.0, 8.0 - k]]))
        rec.capture()
    out = rec.save(tmp_path / "test.gif", hold_last_seconds=0.5)
    assert out.exists()
    with Image.open(out) as im:
        assert im.format == "GIF"
        assert im.info.get("loop") == 0
        assert im.n_frames >= 2
        palette = im.getpalette()[: 3 * 256]
    colours = {tuple(palette[i : i + 3]) for i in range(0, len(palette), 3)}
    # Exact brand colours survive quantisation.
    for hex_colour in (style.BLUE, style.ORANGE, style.SURFACE):
        rgb = tuple(int(hex_colour[i : i + 2], 16) for i in (1, 3, 5))
        assert rgb in colours


def test_gif_recorder_validates_arguments():
    fig = style.new_figure(64, 64)
    with pytest.raises(ValueError):
        GifRecorder(fig, fps=0)
    with pytest.raises(ValueError):
        GifRecorder(fig, max_colors=1)
    with pytest.raises(RuntimeError):
        GifRecorder(fig).save("unused.gif")


def test_sequential_colour_is_monotone_in_lightness():
    def luminance(hex_colour: str) -> float:
        r, g, b = (int(hex_colour[i : i + 2], 16) for i in (1, 3, 5))
        return 0.2126 * r + 0.7152 * g + 0.0722 * b

    values = [style.sequential_color(v, 0.0, 1.0) for v in np.linspace(0, 1, 11)]
    lum = [luminance(c) for c in values]
    assert all(a >= b for a, b in itertools.pairwise(lum))
    assert style.sequential_color(5.0, 1.0, 1.0) in style.SEQUENTIAL_BLUE


def test_markdown_table_and_number_formatting():
    table = plots.markdown_table(["A", "B"], [[1, "x"], [2, "y"]])
    lines = table.splitlines()
    assert lines[0] == "| A | B |"
    assert lines[1] == "|---|---|"
    assert len(lines) == 4
    assert plots.fmt(123456.0) == "123,456"
    assert plots.fmt(None) == "-"
    assert plots.fmt(0.000123, 3) == "0.000123"


def test_line_chart_writes_png(tmp_path):
    out = plots.line_chart(
        {"a": ([1, 2, 3], [1, 4, 9]), "b": ([1, 2, 3], [2, 3, 4])},
        "x",
        "y",
        "title",
        tmp_path / "chart.png",
        logy=True,
    )
    with Image.open(out) as im:
        assert im.size == (760, 440)


def test_find_executable_reports_missing_binary(tmp_path):
    with pytest.raises(FileNotFoundError):
        pipeline.find_executable("definitely_not_built", tmp_path)


def test_update_manifest_records_asset(tmp_path, monkeypatch):
    assets = tmp_path / "assets"
    assets.mkdir()
    asset = assets / "demo.gif"
    asset.write_bytes(b"GIF89a")
    monkeypatch.setattr(pipeline, "ASSETS_DIR", assets)
    monkeypatch.setattr(pipeline, "MANIFEST", assets / "manifest.json")
    pipeline.update_manifest(asset, "scenario", "algorithm", 42, "scripts/render/x.py", "configs/x.json")
    manifest = json.loads((assets / "manifest.json").read_text())
    entry = manifest["assets"]["demo.gif"]
    assert entry["seed"] == 42
    assert entry["bytes"] == 6
    assert entry["script"] == "scripts/render/x.py"
