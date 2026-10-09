# ROB3 CAD (vendored)

Our own copies of the ROB3 arm 3D CAD, kept here so the repo is self-contained.
The upstream donor model is referenced below for attribution.

## Source (upstream)

The base / tower / upper-arm / forearm geometry originates from the
**TR5_Roboterarm** project (the same physical arm this project reverse-engineers
as ROB3):

- Repo: <https://github.com/Fuchsfuchsfuchs/TR5_Roboterarm>, path `3D_CAD/`
- Note: the upstream model is work in progress — the **gripper and wrist are
  not modelled** there.

We vendor our own copy as a neutral **STEP** file (open format, editable in any
CAD tool). We do **not** keep the Autodesk `.f3z` (Fusion-only, proprietary, and
not needed for the simulation — the sim uses the STL meshes). If the solid CAD
ever needs editing in Fusion, it is still available in the upstream repo.

## Files here

| File | What it is | Used by |
| :--- | :--------- | :------ |
| `rob3_arm.step` | neutral CAD of the ARM ONLY (base, tower, upper arm, forearm; no gripper/wrist) | reference / re-export |
| `rob3_complete.stl` | the FULL assembled robot (arm + wrist + gripper) baked in the default pose | reference / viz / print |
| `gripper.py` | **our** parametric gripper CAD (CadQuery), built from bench measurements | generates the files below |
| `gripper.stl` | gripper, default OPEN pose (STL only; regenerate from gripper.py) | `../urdf/meshes/gripper.stl` |
| `gripper_open.stl` / `gripper_closed.stl` | gripper open / closed poses | reference / viz |

> The internal part names inside `rob3_arm.step` may still read
> `TR5_Modell_V1_*` (baked into the donor CAD); they can only be renamed by
> re-authoring the solid in a CAD tool. The file name and this repo use ROB3.

The four arm link **meshes** are not duplicated here; they live where the URDF
loads them: `../urdf/meshes/{base_link,tower,upperarm,forearm}.stl`. See
`../urdf/README.md` for the link ↔ mesh ↔ firmware-axis mapping.

## Regenerating the gripper

```bash
# from the rob3_ucsim repo root, with cadquery in the venv:
.venv/bin/python src/rob3_ucsim/cad/gripper.py
# -> writes gripper.stl, gripper_open.stl, gripper_closed.stl
```

The gripper dimensions are the [HW]-measured values recorded in the firmware
repo at `hardware/mechanics.md` (Gripper section): palm 75×20×15.2, two-stick
fingers (6 mm each, 50 mm long), triangular tips (25 mm legs), jaw span 0 (closed)
→ 28 (straight) → 85 mm (open).
