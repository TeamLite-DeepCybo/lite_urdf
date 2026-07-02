# DeepCybo Lite V2 URDF

This branch contains the cleaned, merged-STL URDF package for Lite/BAR Lite
robot work, with Flash and Pro collision variants.

## Contents

- `urdf/lite_flash_arm_gripper.urdf` - lightweight-collision URDF with
  standardized link/joint names.
- `urdf/lite_flash_arm_hand.urdf` - Flash variant with O6 dexterous hands
  mounted at the wrist-pitch links.
- `urdf/lite_flash_arm_hand_headless.urdf` - Flash O6 hand variant with the
  headless torso mesh.
- `urdf/lite_flash_arm_gripper_headless.urdf` - Flash variant with the
  headless torso mesh and retained task frames.
- `urdf/lite_pro_arm_gripper.urdf` - Pro URDF where collision meshes mirror
  visual meshes.
- `urdf/lite_pro_arm_hand.urdf` - Pro variant with O6 dexterous hands mounted
  at the wrist-pitch links.
- `urdf/lite_pro_arm_hand_headless.urdf` - Pro O6 hand headless variant.
- `urdf/lite_pro_arm_gripper_headless.urdf` - Pro headless variant.
- `meshes/` - merged STL assets referenced by the URDFs.
- `meshes/left_o6/`, `meshes/right_o6/` - O6 dexterous hand mesh assets.
- `meshes/world_root_headless.stl` - headless `world_root` visual mesh used by
  the headless URDFs.
- `mappings/name_mapping.json` - original CAD link/joint names to cleaned names.
- `mappings/mesh_mapping.json` - original CAD STL filenames to cleaned filenames.

## Current State

- Visual materials use a GoldenGlow-inspired color palette mirrored across the
  left and right arms. Gripper finger links are black in both URDF variants.
- Gripper variants include camera task frames:
  - `head_camera_link`
  - `left_wrist_camera_link`
  - `right_wrist_camera_link`
- Hand variants keep `head_camera_link` and omit the wrist/gripper camera
  frames.
- The headless URDFs use the headless torso mesh while retaining the applicable
  task frames for each end effector variant.
- Gripper variants include tip midpoint endpoint frames:
  - `left_gripper_tip_middle_link`
  - `right_gripper_tip_middle_link`
  - both are fixed to the retained wrist-pitch links with the original
    slider-base endpoint placement baked into the merged URDF
- Optical camera frames are removed from the merged URDF.
- Gripper drive gear links/joints are removed.
- Each gripper exposes one control joint:
  - `left_gripper_joint`
  - `right_gripper_joint`
- Same-side gripper fingers are coupled with URDF `mimic` tags on passive joints:
  - `left_gripper_passive_joint` mimics `left_gripper_joint`
  - `right_gripper_passive_joint` mimics `right_gripper_joint`
- O6 hand variants replace the two-finger gripper geometry with dexterous hand
  links. Their O6 wrist-roll adapter joints are fixed:
  - `lh_wrist_roll` uses `rpy="0 0 1.5707963268"`
  - `rh_wrist_roll` uses `rpy="0 0 -1.5707963268"`
  - the hand alignment offsets are baked into the arm wrist-pitch joint origins
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
