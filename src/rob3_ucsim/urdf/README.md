# ROB3 URDF (PyBullet / RViz visualization)

`rob3.urdf` describes the ROB3 6-axis arm for visualization. The default uses
simple geometric primitives (boxes/cylinders) so it works with **zero external
assets**. The joint names match the firmware axis order used by the viewers
(`base`, `shoulder`, `elbow`, `wrist_pitch`, `wrist_roll`, `gripper`).

## Using CAD / STL meshes

PyBullet loads STL (and OBJ/Collada) meshes directly. To render the real CAD
shapes instead of primitives:

1. Put the mesh files here: `urdf/meshes/link2.stl`, etc. (shipped as package
   data).
2. In `rob3.urdf`, replace a link's `<geometry>` primitive with a mesh ref —
   paths resolve **relative to the URDF file's directory**:

   ```xml
   <visual>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <geometry>
       <!-- CAD STL is usually in mm; URDF is metres -> scale 0.001 -->
       <mesh filename="meshes/link2.stl" scale="0.001 0.001 0.001"/>
     </geometry>
     <material name="rob3_grey"/>
   </visual>
   ```

Tips:
- Keep `<collision>` as a cheap primitive even if `<visual>` is a mesh.
- Check units: scale `0.001` for mm→m CAD exports.
- The joints/limits are unchanged — meshes are a visual upgrade only, driven by
  the same firmware positions.
