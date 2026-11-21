"""Visualization, orchestration and reporting for MultiRobotCore.

Algorithms live in C++. This package only reads the JSON result files written
by the C++ demo and experiment executables and turns them into animations,
plots and tables. It never re-implements an algorithm.
"""

from multirobotcore_viz.animation import GifRecorder
from multirobotcore_viz.style import apply_style

__all__ = ["GifRecorder", "apply_style"]
__version__ = "0.1.0"
