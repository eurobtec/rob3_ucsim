"""PyBullet viewer — a 3D frontend rendering the ROB3 from firmware positions.

Loads the ROB3 URDF (shipped in this package) and sets its six joints from the
firmware position vector each frame. The PyBullet GUI window keeps its NATIVE
mouse camera (orbit / zoom / pan — like urdf-viz); this viewer only reads
positions and renders, it does not capture the keyboard or drive the firmware.

PyBullet is an OPTIONAL dependency:  ``pip install rob3_ucsim[viz]``.
"""

from __future__ import annotations

import os
from typing import Sequence

from ..axes import AXES, JOINT_NAMES


def urdf_path() -> str:
    """Absolute path to the ROB3 URDF shipped in this package."""
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(here, "urdf", "rob3.urdf")


class PyBulletViewer:
    """A PyBullet 3D scene; joints set from firmware position bytes."""

    def __init__(self, gui: bool = True):
        try:
            import pybullet as p
            import pybullet_data
        except ImportError as e:  # pragma: no cover
            raise RuntimeError(
                "PyBullet not installed — `pip install rob3_ucsim[viz]`"
            ) from e
        self._p = p
        self._cid = p.connect(p.GUI if gui else p.DIRECT)
        self._gui = gui
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, 0)              # positions are commanded, not dynamic
        try:
            p.loadURDF("plane.urdf")
        except Exception:
            pass
        self._body = p.loadURDF(urdf_path(), useFixedBase=True)
        self._joint_index = {}
        for j in range(p.getNumJoints(self._body)):
            name = p.getJointInfo(self._body, j)[1].decode()
            self._joint_index[name] = j
        if gui:
            # Native orbit camera, same feel as urdf-viz.
            p.resetDebugVisualizerCamera(1.2, 45, -30, [0, 0, 0.3])

    def set_positions(self, positions: Sequence[int]) -> None:
        p = self._p
        for ax in AXES:
            j = self._joint_index.get(ax.name)
            if j is None or positions[ax.index] < 0:
                continue
            p.resetJointState(self._body, j, ax.to_joint(positions[ax.index]))
        if self._gui:
            p.stepSimulation()

    def close(self) -> None:
        try:
            self._p.disconnect(self._cid)
        except Exception:
            pass
