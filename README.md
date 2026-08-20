# Lite URDF Description

This package is the Lite ROS 2 production description package. It keeps the
existing `lite_urdf` ROS package name, but the robot model is now
the newer `lite_urdf` arms+grippers assembly.

## Contents

- `urdf/lite.urdf.xacro` - canonical arms+grippers model.
- `urdf/lite.ros2_control.xacro` - ros2_control metadata for real hardware,
  fake hardware, and MuJoCo.
- `meshes/` - merged STL assets referenced by the canonical model.
- `mjcf/lite.xml` - MuJoCo scene matching the canonical arms+grippers joint
  tree.
- `mappings/` - source CAD name/mesh mapping retained for provenance.
- `scripts/generate_mjcf.py` - regenerates `mjcf/lite.xml` from the xacro
  through MuJoCo's URDF compiler and refreshes MuJoCo-only decimated STL files.
- `launch/view_lite.launch.py` - standalone joint-slider RViz inspector.
- `launch/live_rviz.launch.py` - RViz wrapper for viewing the model from a
  live `/lite/joint_states` stream.

## Joint Convention

The exposed control joint order is:

```text
left_shoulder_pitch
left_shoulder_roll
left_shoulder_yaw
left_elbow_pitch
left_wrist_yaw
left_wrist_roll
left_wrist_pitch
right_shoulder_pitch
right_shoulder_roll
right_shoulder_yaw
right_elbow_pitch
right_wrist_yaw
right_wrist_roll
right_wrist_pitch
left_gripper
right_gripper
```

The arm and gripper names match `bar_bringup_lite/config/lite_hardware.yaml`.
The joint origins, axes, parent/child topology, and STL frame assumptions stay
with the newer `lite_urdf` model. Do not copy link or joint transforms from the
old model into this package; the STL frames are different.

`right_elbow_pitch` uses the corrected Lite URDF axis and positive motion
range. Motor direction signs live in `lite.ros2_control.xacro` and calibration
is supplied at runtime through an explicit absolute `calibration_file` path to
a robot-local, untracked YAML; the repository ships no machine calibration.

## RViz

For slider-based inspection:

```bash
ros2 launch lite_urdf view_lite.launch.py
```

For real robot visualization beside `bar_bringup_lite real.launch.py`:

```bash
ros2 launch lite_urdf live_rviz.launch.py
```

For namespaced real bringup, point the viewer at the namespaced joint-state
topic:

```bash
ros2 launch lite_urdf live_rviz.launch.py joint_state_topic:=/master/lite/joint_states
```

## MuJoCo

The MuJoCo scene is installed from `mjcf/lite.xml` and is selected by the Lite
bringup launch:

```bash
ros2 launch bar_bringup_lite mujoco.launch.py mode:=arms_grippers
```

The MJCF is intentionally kept in the same package as the xacro and STL assets
so `mujoco_sim_ros2` can resolve it through `model_package:=lite_urdf`.

Regenerate the MJCF after kinematic or mesh changes with:

```bash
MUJOCO_COMPILE=/path/to/mujoco/bin/compile scripts/generate_mjcf.py
```

The generator expects `xacro`, `trimesh`, `scipy`, and `fast-simplification`
in the active environment.

## Grippers

Each physical gripper exposes one commanded joint:

```text
left_gripper
right_gripper
```

The opposite finger on each side remains coupled through the URDF mimic joint:

```text
left_gripper_passive_joint  mimics left_gripper
right_gripper_passive_joint mimics right_gripper
```

Controllers should command only `left_gripper` and `right_gripper`.
