"""Viewer frontend protocol — a read-only renderer of ROB3 axis positions.

A viewer consumes the firmware's 6-byte position vector (``read_positions()``,
IRAM 0x50+N, 0..255) each frame and renders it however it likes (3D, terminal,
nothing). Viewers do NOT drive the firmware — input (teachbox/serial/program)
is a separate concern. See :mod:`rob3_ucsim.axes` for the shared byte->joint map.
"""

from __future__ import annotations

from typing import Protocol, Sequence


class Viewer(Protocol):
    """A read-only ROB3 visualization frontend."""

    def set_positions(self, positions: Sequence[int]) -> None:
        """Render the robot from a 6-byte firmware position vector (0..255)."""
        ...

    def close(self) -> None:
        """Release any resources (windows, connections)."""
        ...
