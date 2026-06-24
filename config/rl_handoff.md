# RL Handoff Notes

Recommended URDF for RL starting work:

```text
urdf/lite_000_asm.urdf
```

This file keeps the full cleaned CAD assembly while preserving:

```text
14 arm revolute joints
4 gripper prismatic finger joints
2 gripper mimic constraints
```

## Joint Convention Compatibility

The arm joints intentionally follow the older `bhl_arm_1` convention for:

- XML joint ordering
- zero-pose meaning
- lower/upper limits
- positive joint direction

The movable joint order is:

```text
left_wrist_pitch_joint
left_wrist_roll_joint
left_wrist_yaw_joint
left_elbow_pitch_joint
left_shoulder_yaw_joint
left_shoulder_roll_joint
left_shoulder_pitch_joint
right_wrist_pitch_joint
right_wrist_roll_joint
right_wrist_yaw_joint
right_elbow_pitch_joint
right_shoulder_yaw_joint
right_shoulder_roll_joint
right_shoulder_pitch_joint
left_gripper_left_finger_joint
left_gripper_right_finger_joint
right_gripper_left_finger_joint
right_gripper_right_finger_joint
```

Integrations should treat `urdf/lite_000_asm.urdf` as a drop-in replacement for
the old arm convention. Avoid adding simulator-side sign flips, reordered joint
lists, or extra home offsets unless the target stack explicitly needs them.

Do not use an automatically collapsed fixed-joint simplification unless it has been visually verified. A previous generated simplification produced incorrect geometry placement, so the full cleaned URDF remains the reliable handoff file.

## Fixed Joint Merge Guidance

Use the simulator/importer fixed-joint merge option instead of a hand-collapsed URDF:

| Target | Recommended setting |
| --- | --- |
| Isaac Lab / Isaac Sim | Use `merge_fixed_joints=True` or the CLI `--merge-joints` option when importing through Isaac Lab. In the Isaac Sim URDF importer UI, enable **Merge Fixed Joints**. Note: Isaac Sim 5.1 has a known caveat where fixed links with mass/inertia may not merge as expected in the raw importer; Isaac Lab currently pins the older importer behavior for compatibility. |
| MuJoCo | URDF import defaults to `fusestatic=true`; keep it enabled. |
| Genesis | `merge_fixed_links=True` is the default. Use `links_to_keep` only for frames that must survive merging. |
| Gazebo / SDFormat | Fixed joints are lumped by default. Do not set `preserveFixedJoint=true` or `disableFixedJointLumping=true` unless preserving a specific fixed frame. |

If the target simulator ignores URDF `mimic` tags, implement the same-side gripper coupling in the simulator actuator/controller layer.

## Collision Notes

The URDF uses the sparse collision style from the older `bhl_arm_1` model:

- chest/body has one primitive box
- each arm has simple cylinders only on shoulder-yaw and wrist-yaw links
- gripper fingertips use small primitive boxes
- camera links, internal housings, and adjacent joint-stack components are intentionally collisionless

This avoids false self-collisions in the assembled arm. Treat the collisions as lightweight RL contact geometry, not full visual-geometry coverage.

Remaining physics work:

```text
verify collision behavior in the target simulator
choose simulator-specific actuator model
verify/tune mass and inertia values
confirm whether the simulator supports URDF mimic tags
optionally simplify fixed CAD subparts with a verified CAD/URDF pipeline
```

References checked:

- Isaac Lab URDF import: https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html
- Isaac Lab Isaac Sim 5.1 importer caveat: https://isaac-sim.github.io/IsaacLab/main/source/refs/issues.html
- Isaac Sim URDF importer UI: https://docs.isaacsim.omniverse.nvidia.com/4.2.0/features/environment_setup/ext_omni_isaac_urdf.html
- MuJoCo XML reference, `fusestatic`: https://mujoco.readthedocs.io/en/latest/XMLreference.html
- Genesis URDF options: https://genesis-world.readthedocs.io/en/latest/api_reference/options/morph/file_morph/urdf.html
- SDFormat URDF extensions: https://sdformat.org/tutorials/specification/sdformat_urdf_extensions/1.6/
