"""Shared ROB3 axis table — the single source of truth for all viz frontends.

Maps the firmware's per-axis position byte (IRAM 0x50+N, 0..255) to a joint
value and carries the display metadata both the Tkinter GUI and the PyBullet
3D view use. Mapping is backend-independent (it is the robot's geometry, not a
rendering concern); each frontend renders these joint values however it likes.

ROB3 axis table (README): byte 0 .. 255 maps linearly across each joint's range.
Several axes invert (byte 0 = the positive/max end).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

_D = math.radians


@dataclass(frozen=True)
class Axis:
    index: int
    name: str          # firmware/joint name (also the URDF joint name)
    label: str         # human label (README q-number)
    at0: float         # joint value at byte 0   (rad for revolute, m for prismatic)
    at255: float       # joint value at byte 255
    units: str         # display units string
    prismatic: bool = False

    def to_joint(self, value: int) -> float:
        """Map a firmware position byte (0..255) to this axis's joint value."""
        v = max(0, min(255, value)) / 255.0
        return self.at0 + (self.at255 - self.at0) * v


#: The six ROB3 axes in firmware order (axis 0..5). URDF joint names match.
AXES = [
    Axis(0, "base",        "q1 Base",        _D(80.0),  _D(-80.0),  "-80..+80 deg"),
    Axis(1, "shoulder",    "q2 Shoulder",    _D(70.0),  _D(-30.0),  "-30..+70 deg"),
    Axis(2, "elbow",       "q3 Elbow",       _D(0.0),   _D(-100.0), "-100..0 deg"),
    Axis(3, "wrist_pitch", "q4 Wrist pitch", _D(100.0), _D(-100.0), "-100..+100 deg"),
    Axis(4, "wrist_roll",  "q5 Wrist roll",  _D(100.0), _D(-100.0), "-100..+100 deg"),
    Axis(5, "gripper",     "Gripper",        0.0,       0.060,      "0..60 mm", True),
]

JOINT_NAMES = [a.name for a in AXES]


def positions_to_joints(positions) -> list[float]:
    """Map the 6-byte firmware position vector to the 6 joint values."""
    return [AXES[i].to_joint(positions[i]) for i in range(len(AXES))]
