# DeepCybo Lite V2 Flash URDF

This branch contains the cleaned, merged-STL URDF package for Lite/BAR Lite robot work.

## Contents

- `urdf/lite_arm_gripper.urdf` - cleaned URDF with standardized link/joint names.
- `urdf/lite_arm_gripper_headless.urdf` - variant without the head camera frame
  or head/top visual geometry.
- `meshes/` - merged STL assets referenced by the URDFs.
- `meshes/world_root_headless.stl` - headless `world_root` visual mesh used by
  `lite_arm_gripper_headless.urdf`.
- `mappings/name_mapping.json` - original CAD link/joint names to cleaned names.
- `mappings/mesh_mapping.json` - original CAD STL filenames to cleaned filenames.

## Current State

- Visual materials use a GoldenGlow-inspired color palette mirrored across the
  left and right arms. Gripper finger links are black in both URDF variants.
- Camera frames are included:
  - `head_camera_link`
  - `left_wrist_camera_link`
  - `right_wrist_camera_link`
- The headless URDF removes `head_camera_link` and `head_camera_joint`, while
  keeping both wrist camera frames.
- Gripper tip midpoint endpoint frames are included:
  - `left_gripper_tip_middle_link`
  - `right_gripper_tip_middle_link`
  - both are fixed to the retained wrist-pitch links with the original
    slider-base endpoint placement baked into the merged URDF
- Optical camera frames are removed from the merged URDF.
- Gripper drive gear links/joints are removed.
- Each hand exposes one gripper control joint:
  - `left_gripper_joint`
  - `right_gripper_joint`
- Same-side gripper fingers are coupled with URDF `mimic` tags on passive joints:
  - `left_gripper_passive_joint` mimics `left_gripper_joint`
  - `right_gripper_passive_joint` mimics `right_gripper_joint`
- Arm joint order, zero pose, limits, and positive directions follow the older
  `bhl_arm_1` scheme. Downstream controllers, RL code, and ROS 2 integrations
  that were built around the old convention should be able to use this URDF
  without remapping joint semantics.
- Link, joint, and mesh names are ASCII snake_case.
- Mesh paths use `package://lite_urdf/meshes/...`.
- Fixed-link STL geometry has been merged into retained movable/root link
  meshes. Use the archived temp branch if the merge script or source experiment
  notes are needed.

## RL Notes

This is a good visual and kinematic source package, but it is not yet a final physics model.
Before high-throughput RL training, add or verify:

- collision geometry
- actuator/transmission metadata
- simulator support for URDF `mimic`, or equivalent gripper coupling in simulator config
- inertial parameters
- simulator-specific configuration

The merged URDF is the current source of truth. Use the headless URDF when the
upper head/top assembly should be omitted from visualization/import.

For RL simulation, prefer the target simulator/importer fixed-joint merge option instead of a locally collapsed URDF. See `config/rl_handoff.md` for Isaac, MuJoCo, Genesis, and Gazebo notes.
