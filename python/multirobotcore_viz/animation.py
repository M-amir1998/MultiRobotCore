"""Deterministic, size-conscious GIF recording for matplotlib figures.

Frames are rasterised with the Agg backend, quantised to one global palette
computed from a sample of frames (stable colours, no flicker) and written with
Pillow. Unchanged regions between frames are stored once by Pillow's GIF
encoder, which keeps light-background scientific animations small.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from matplotlib.figure import Figure
from PIL import Image

from multirobotcore_viz import style


class GifRecorder:
    """Collects figure frames and writes an optimised looping GIF."""

    def __init__(self, fig: Figure, fps: int = 20, max_colors: int = 160) -> None:
        if fps <= 0:
            raise ValueError("fps must be positive")
        if not 2 <= max_colors <= 256:
            raise ValueError("max_colors must be in [2, 256]")
        self.fig = fig
        self.fps = fps
        self.max_colors = max_colors
        self.frames: list[Image.Image] = []

    def capture(self, repeat: int = 1) -> None:
        """Rasterises the current figure state `repeat` times."""
        self.fig.canvas.draw()
        rgba = np.asarray(self.fig.canvas.buffer_rgba())
        image = Image.fromarray(rgba[..., :3].copy(), mode="RGB")
        for _ in range(repeat):
            self.frames.append(image)

    def _palette(self) -> Image.Image:
        """Global palette: the exact style colours first (so small but
        meaningful marks such as a leader keep their hue), then adaptive
        colours from a sample of frames."""
        fixed = list(dict.fromkeys(c.lower() for c in style.PALETTE_SEED))
        count = len(self.frames)
        picks = sorted({0, count - 1, *np.linspace(0, count - 1, num=min(count, 10), dtype=int)})
        sample = [self.frames[i].reduce(2) for i in picks]
        mosaic = Image.new("RGB", (sample[0].width, sum(s.height for s in sample)))
        y = 0
        for s in sample:
            mosaic.paste(s, (0, y))
            y += s.height
        adaptive_count = max(self.max_colors - len(fixed), 8)
        adaptive = mosaic.quantize(colors=adaptive_count, method=Image.Quantize.MEDIANCUT)
        adaptive_rgb = adaptive.getpalette()[: 3 * adaptive_count]
        values: list[int] = []
        for color in fixed:
            values += [int(color[i : i + 2], 16) for i in (1, 3, 5)]
        values += adaptive_rgb
        values += [0] * (768 - len(values))
        palette = Image.new("P", (1, 1))
        palette.putpalette(values[:768])
        return palette

    def save(self, path: str | Path, hold_last_seconds: float = 1.5) -> Path:
        """Writes the GIF and returns its path."""
        if not self.frames:
            raise RuntimeError("no frames captured")
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        palette = self._palette()
        quantised = [f.quantize(palette=palette, dither=Image.Dither.NONE) for f in self.frames]
        frame_ms = round(1000 / self.fps)
        durations = [frame_ms] * len(quantised)
        durations[-1] = max(frame_ms, round(hold_last_seconds * 1000))
        quantised[0].save(
            path,
            save_all=True,
            append_images=quantised[1:],
            duration=durations,
            loop=0,
            optimize=False,
            disposal=1,
        )
        return path

    @property
    def num_frames(self) -> int:
        return len(self.frames)


def frame_indices(num_samples: int, num_frames: int) -> list[int]:
    """Evenly spaced indices into a recorded trajectory of `num_samples`
    samples, used to subsample long simulations to a fixed animation length."""
    if num_samples <= 0:
        return []
    count = min(num_samples, num_frames)
    return sorted({round(float(i)) for i in np.linspace(0, num_samples - 1, num=count)})
