# Lite URDF - RL Ready Branch

This branch contains the cleaned full CAD URDF package for Lite/BAR Lite robot work.

## Contents

- `urdf/lite_000_asm.urdf` - cleaned URDF with standardized link/joint names.
- `meshes/` - renamed STL assets referenced by the URDF.
- `mappings/name_mapping.json` - original CAD link/joint names to cleaned names.
- `mappings/mesh_mapping.json` - original CAD STL filenames to cleaned filenames.

## Current State

- Camera frames are included:
  - `head_camera_link`
  - `head_camera_optical_frame`
  - `left_wrist_camera_link`
  - `left_wrist_camera_optical_frame`
  - `right_wrist_camera_link`
  - `right_wrist_camera_optical_frame`
- Gripper drive gear links/joints are removed.
- Same-side gripper fingers are coupled with URDF `mimic` tags:
  - `left_gripper_right_finger_joint` mimics `left_gripper_left_finger_joint`
  - `right_gripper_right_finger_joint` mimics `right_gripper_left_finger_joint`
- Link, joint, and mesh names are ASCII snake_case.
- Mesh paths use `package://lite_urdf/meshes/...`.

## RL Notes

This is a good visual and kinematic source package, but it is not yet a final physics model.
Before high-throughput RL training, add or verify:

- collision geometry
- actuator/transmission metadata
- simulator support for URDF `mimic`, or equivalent gripper coupling in simulator config
- inertial parameters
- simulator-specific configuration

Do not use an automatically collapsed fixed-joint version unless it has been visually verified in a URDF viewer. The full cleaned URDF is the current source of truth.

For RL simulation, prefer the target simulator/importer fixed-joint merge option instead of a locally collapsed URDF. See `config/rl_handoff.md` for Isaac, MuJoCo, Genesis, and Gazebo notes.
