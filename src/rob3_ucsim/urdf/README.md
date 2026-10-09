# ROB3 URDF (PyBullet / RViz visualization)

`rob3.urdf` describes the ROB3 6-axis arm for visualization. It is now
**mesh-based and synced to the TR5 3D CAD** (the source of truth), with
primitive stand-ins for the axes the CAD does not yet model.

The joint names/order match the firmware axis order the viewers drive **by name**
(`base`, `shoulder`, `elbow`, `wrist_pitch`, `wrist_roll`, `gripper` - see
`axes.py`). Do **not** rename them or the viewers stop finding the joints.

## Source of truth: the TR5 3D CAD

The geometry comes from the TR5_Roboterarm CAD model
(<https://github.com/Fuchsfuchsfuchs/TR5_Roboterarm>, `3D_CAD/`). The raw CAD
and the original Fusion 360 URDF export are vendored under `../cad/` for
provenance:

```
cad/
├── TR5_Modell_V1.step        # neutral CAD (STEP)
├── TR5_Modell_V1.f3z         # Fusion 360 archive (editable source)
├── CAD_Information.txt        # upstream note: gripper + details missing
└── fusion_urdf_export/        # the original Fusion-generated ROS 2 URDF package
```

The four CAD meshes are copied into `meshes/` under sim-friendly names
(mm; scaled `0.001` to metres in the URDF):

| URDF link | Mesh file          | CAD part (German) | Firmware axis |
| :-------- | :----------------- | :---------------- | :------------ |
| `base_link` | `base_link.stl`  | base_link         | (fixed base)  |
| `link1`     | `tower.stl`      | Turm (tower)      | 0 `base`      |
| `link2`     | `upperarm.stl`   | Oberarm           | 1 `shoulder`  |
| `link3`     | `forearm.stl`    | Unterarm          | 2 `elbow`     |

Joint origins/axes for `base`/`shoulder`/`elbow` are taken verbatim from the
Fusion export (CAD joints `Umdrehung 14`, `Gelenk 2`, `Gelenk 3`). The joint
**limits**, however, are the **firmware** limits from `axes.py` (the sim is
driven by the firmware, not by the Fusion-default limits).

## Completeness gap (CAD is work in progress)

Per `cad/CAD_Information.txt`: *"3D CAD Modell is still work in Progress, the
Gripper is missing and also some Details."* The CAD models only the lower 3
axes. The upper 3 axes are therefore **primitive stand-ins**, rendered in
translucent orange (`rob3_standin`) so they are visually obvious:

| URDF link    | Firmware axis   | Status                      |
| :----------- | :-------------- | :-------------------------- |
| `link4`      | 3 `wrist_pitch` | **STAND-IN** (not in CAD)   |
| `link5`      | 4 `wrist_roll`  | **STAND-IN** (not in CAD)   |
| `gripper_link` | 5 `gripper`   | **STAND-IN** (not in CAD)   |

The stand-in chain attaches at the **forearm tip**, computed from the forearm
mesh bounding box in the `link3` frame (x ≈ 0.148 m, y ≈ 0.033 m).

### To complete the model

1. Model the wrist (pitch + roll) and gripper in `cad/TR5_Modell_V1.f3z`
   (Fusion 360), re-export, and drop the new STLs into `meshes/`.
2. Replace the three `standin_cyl` / `gripper_link` primitives in `rob3.urdf`
   (and `rob3.urdf.xacro`) with `<mesh>` refs, like the CAD links above.
3. Set each new joint's `<origin>` from the CAD and keep the joint **name**,
   **axis**, and **firmware limits** unchanged.

## Provenance tags

Following the project convention:

- **[HW/CAD]** `base_link`, `link1`, `link2`, `link3` - CAD meshes + transforms.
- **[INFER]** `link4`, `link5`, `gripper_link` - approximate stand-ins, not yet
  verified against hardware or CAD.

## Verifying

The URDF loads in PyBullet (DIRECT) with all six firmware joints present and
drivable through `axes.AXES.to_joint`:

```bash
.venv/bin/python - <<'PY'
import pybullet as p
c = p.connect(p.DIRECT)
b = p.loadURDF("src/rob3_ucsim/urdf/rob3.urdf", useFixedBase=True)
print([p.getJointInfo(b, j)[1].decode() for j in range(p.getNumJoints(b))])
p.disconnect(c)
PY
# -> ['base', 'shoulder', 'elbow', 'wrist_pitch', 'wrist_roll', 'gripper']
```

Notes:
- Mesh paths resolve **relative to the URDF file's directory** (`meshes/...`).
- CAD STL is in mm; the URDF scales `0.001` to metres.
- `rob3.urdf.xacro` is kept in sync with `rob3.urdf` (the plain URDF the sim
  loads directly); it uses `cad_link` / `standin_cyl` macros for the two kinds.
