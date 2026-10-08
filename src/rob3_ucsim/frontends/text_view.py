"""Text viewer — a terminal frontend showing each axis current value vs min/max.

Renders the six ROB3 axes as live ASCII bars: the firmware position byte
(0..255), the mapped joint value, and the joint's min..max range. No GUI
dependencies — works in any terminal and is the default viewer.
"""

from __future__ import annotations

import sys
from typing import Sequence

from ..axes import AXES

_BAR_W = 24


def _fmt_joint(ax, joint: float) -> str:
    if ax.prismatic:
        return f"{joint * 1000:6.1f} mm"
    return f"{joint * 57.29578:+7.1f}\u00b0"   # rad -> deg


class TextViewer:
    """Live terminal readout of the six axes (current value vs min/max)."""

    def __init__(self, stream=None):
        self._out = stream or sys.stdout
        self._lines = len(AXES) + 1
        self._primed = False

    def set_positions(self, positions: Sequence[int], status: str = "") -> None:
        rows = ["ROB3 axes — pos byte (0..255) | joint value | range"]
        for ax in AXES:
            b = positions[ax.index] if ax.index < len(positions) else -1
            if b is None or b < 0:
                # Always emit a row (fixed count) so the in-place redraw stays
                # aligned, even before the first live reading.
                rows.append(f"{ax.label:16s} [{'-' * _BAR_W}]  ...  (no data)")
                continue
            frac = max(0, min(255, b)) / 255.0
            fill = int(frac * _BAR_W)
            bar = "#" * fill + "-" * (_BAR_W - fill)
            joint = ax.to_joint(b)
            lo = _fmt_joint(ax, ax.to_joint(0))
            hi = _fmt_joint(ax, ax.to_joint(255))
            rows.append(
                f"{ax.label:16s} [{bar}] {b:3d}  {_fmt_joint(ax, joint)}  "
                f"({lo} .. {hi})"
            )
        rows.append(f"last: {status}" if status else "")   # managed status line
        # Redraw in place: move the cursor up over exactly the rows we wrote last
        # time (constant row count), clear + rewrite each.
        if self._primed:
            self._out.write(f"\x1b[{self._lines}A")
        for r in rows:
            self._out.write("\x1b[2K" + r + "\n")
        self._out.flush()
        self._lines = len(rows)   # always len(AXES)+1, but stay exact
        self._primed = True

    def close(self) -> None:
        self._out.flush()
