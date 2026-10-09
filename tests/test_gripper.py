"""Gripper tests: CAD open/close geometry + URDF↔CAD consistency.

These verify two things the bench measurements pinned down:
  1. the parametric gripper CAD (cad/gripper.py) reproduces the MEASURED
     open jaw span (85 mm) and closed axis-to-tip length (125 mm), and that
     opening the gripper really widens the jaw (motion sanity); and
  2. the simulation URDF stays CONSISTENT with that CAD — it references the
     generated gripper mesh rather than a stand-in primitive, and still loads
     with all six firmware joints.

No CAD kernel is needed: the gripper module exposes analytic geometry helpers
(open_inner_span / axis_to_tip) that share the model's own parameters, so the
test asserts against the same math the solid is built from.
"""
from __future__ import annotations

import importlib.util
import os
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CAD = os.path.join(ROOT, "src", "rob3_ucsim", "cad")
URDF = os.path.join(ROOT, "src", "rob3_ucsim", "urdf", "rob3.urdf")
MESHES = os.path.join(ROOT, "src", "rob3_ucsim", "urdf", "meshes")


def _load_gripper_module():
    """Import cad/gripper.py (params + helpers) without running its __main__."""
    spec = importlib.util.spec_from_file_location(
        "rob3_gripper", os.path.join(CAD, "gripper.py")
    )
    m = importlib.util.module_from_spec(spec)
    import sys

    sys.modules["rob3_gripper"] = m
    spec.loader.exec_module(m)
    return m


G = _load_gripper_module()

# Measured ground truth (hardware/mechanics.md, Gripper).
OPEN_SPAN_MM = 85.0
CLOSED_LEN_MM = 125.0
TOL = 1.5       # mm — tolerance for spans (open/straight), which are solved exactly
TOL_LEN = 3.0   # mm — tolerance for the closed axis-to-tip length: it is a DERIVED
                # consequence of the (exactly-matched) stick-hole spacings and jaw
                # spans, so it lands ~1.6 mm short of the separately hand-measured
                # 125 mm. That residual is the model-vs-caliper agreement.


# --------------------------------------------------------------------------- #
# 1. CAD open/close geometry matches the measurements
# --------------------------------------------------------------------------- #
def test_open_span_matches_measurement():
    span = G.open_inner_span(G.SPLAY_OPEN_DEG)
    assert span == pytest.approx(OPEN_SPAN_MM, abs=TOL), (
        f"open inner-face span {span:.1f} mm != measured {OPEN_SPAN_MM} mm"
    )


def test_closed_length_matches_measurement():
    length = G.axis_to_tip(G.SPLAY_CLOSED_DEG)
    assert length == pytest.approx(CLOSED_LEN_MM, abs=TOL_LEN), (
        f"closed axis-to-tip {length:.1f} mm != measured {CLOSED_LEN_MM} mm"
    )


def test_opening_widens_the_jaw():
    """Opening (larger splay) must increase the jaw span, and the open pose
    must be wider than the closed pose — the open/close motion direction."""
    assert G.SPLAY_OPEN_DEG > G.SPLAY_CLOSED_DEG
    assert G.open_inner_span(G.SPLAY_OPEN_DEG) > G.open_inner_span(G.SPLAY_CLOSED_DEG)


def test_params_are_hw_measured():
    """Guard the key measured dimensions so a careless edit is caught."""
    assert G.PALM_W == 75.0
    assert G.PALM_H == 20.0
    assert G.PALM_THK == 15.2
    assert G.LINK_LEN == 50.0
    assert G.STICK_W == 6.0
    assert G.LINK_W == 14.0
    assert G.FINGER_GAP == 52.0
    assert G.TIP_LEG == 25.0
    assert G.HUB_DIA == 17.0
    assert G.HUB_LEN == 50.0
    assert G.AXIS_TO_PALM == 41.0


# --------------------------------------------------------------------------- #
# 2. URDF is consistent with the CAD
# --------------------------------------------------------------------------- #
def test_gripper_mesh_files_exist():
    for f in ("gripper.stl", "gripper_closed.stl"):
        p = os.path.join(MESHES, f)
        assert os.path.isfile(p) and os.path.getsize(p) > 0, f"missing mesh {f}"


def test_urdf_gripper_link_uses_cad_mesh():
    """The gripper_link must reference the CAD mesh, not a stand-in primitive."""
    root = ET.parse(URDF).getroot()
    link = next(l for l in root.findall("link") if l.get("name") == "gripper_link")
    vis = link.find("visual")
    mesh = vis.find("geometry/mesh")
    assert mesh is not None, "gripper_link visual is not a mesh (still a stand-in?)"
    assert mesh.get("filename").endswith("gripper.stl")
    # no primitive box/cylinder left in the gripper visual
    assert vis.find("geometry/box") is None
    assert vis.find("geometry/cylinder") is None


def test_urdf_has_six_firmware_joints():
    root = ET.parse(URDF).getroot()
    joints = [j.get("name") for j in root.findall("joint")]
    assert joints == [
        "base",
        "shoulder",
        "elbow",
        "wrist_pitch",
        "wrist_roll",
        "gripper",
    ]


def test_urdf_loads_in_pybullet_if_available():
    pb = pytest.importorskip("pybullet")
    cid = pb.connect(pb.DIRECT)
    try:
        body = pb.loadURDF(os.path.abspath(URDF), useFixedBase=True)
        names = {
            pb.getJointInfo(body, j)[1].decode()
            for j in range(pb.getNumJoints(body))
        }
        assert {"wrist_roll", "gripper"} <= names
    finally:
        pb.disconnect(cid)
