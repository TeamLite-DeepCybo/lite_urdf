# DeepCybo Lite V2 URDF

This branch contains the cleaned, merged-STL URDF package for Lite/BAR Lite
robot work, with Flash and Pro collision variants.

## Contents

- `urdf/lite_flash_arm_gripper.urdf` - lightweight-collision URDF with
  standardized link/joint names.
- `urdf/lite_flash_arm_gripper_headless.urdf` - Flash variant without the head
  camera frame or head/top visual geometry.
- `urdf/lite_pro_arm_gripper.urdf` - Pro URDF where collision meshes mirror
  visual meshes.
- `urdf/lite_pro_arm_gripper_headless.urdf` - Pro headless variant.
- `meshes/` - merged STL assets referenced by the URDFs.
- `meshes/world_root_headless.stl` - headless `world_root` visual mesh used by
  the headless URDFs.
- `mappings/name_mapping.json` - original CAD link/joint names to cleaned names.
- `mappings/mesh_mapping.json` - original CAD STL filenames to cleaned filenames.

## Current State

- Visual materials use a GoldenGlow-inspired color palette mirrored across the
  left and right arms. Gripper finger links are black in both URDF variants.
- Camera frames are included:
  - `head_camera_link`
  - `left_wrist_camera_link`
  - `right_wrist_camera_link`
- The headless URDF keeps all camera task frames while using the headless
  torso mesh.
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
- Flash URDFs use the lightweight collision geometry inherited from the RL-ready
  handoff model.
- Pro URDFs use visual STL meshes as collision meshes and assume simulator-side
  adjacent/internal self-collision filtering.

## RL Notes

Use `lite_flash_*` when you want conservative lightweight collisions. Use
`lite_pro_*` for MuJoCo/Genesis-style pipelines that can filter adjacent and
internal self-collisions while using visual meshes for contact geometry.

Before high-throughput RL training, add or verify:

- actuator/transmission metadata
- simulator support for URDF `mimic`, or equivalent gripper coupling in simulator config
- inertial parameters
- simulator-specific configuration
- self-collision filtering when using the Pro visual-mesh collision URDFs

Use a headless URDF when the upper head/top assembly should be omitted from
visualization/import.

For RL simulation, prefer the target simulator/importer fixed-joint merge option instead of a locally collapsed URDF. See `config/rl_handoff.md` for Isaac, MuJoCo, Genesis, and Gazebo notes.
