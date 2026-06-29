# RL Handoff Notes

Recommended URDFs for RL starting work:

```text
urdf/lite_flash_arm_gripper.urdf
urdf/lite_pro_arm_gripper.urdf
```

Both files keep the full cleaned CAD assembly while preserving:

```text
14 arm revolute joints
2 gripper prismatic control joints
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
left_gripper_joint
right_gripper_joint
```

The passive gripper joints are:

```text
left_gripper_passive_joint
right_gripper_passive_joint
```

They are URDF mimic joints used to keep each same-side finger pair coupled and
should not have individual finger controllers.

The gripper control coordinate is normalized so the default initial position is
the lower limit:

```text
left_gripper_joint  initial=0.0 lower=0.0 upper=0.047
right_gripper_joint initial=0.0 lower=0.0 upper=0.047
```

Integrations should treat both `lite_flash_*` and `lite_pro_*` URDFs as
drop-in replacements for the old arm convention. Avoid adding simulator-side
sign flips, reordered joint lists, or extra home offsets unless the target stack
explicitly needs them.

The merged URDF keeps the three camera links:

```text
head_camera_link
left_wrist_camera_link
right_wrist_camera_link
```

It also keeps the gripper midpoint endpoint links:

```text
left_gripper_tip_middle_link
right_gripper_tip_middle_link
```

These endpoint frames are fixed to the retained wrist-pitch links with the old
slider-base placement baked in. They match the centered fingertip x/y position
with local `z=0.063` from the unmerged URDF.

Optical camera frames have been removed. Add simulator site/frame definitions
outside this URDF if a downstream policy needs those optical targets.

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

The branch carries two collision variants:

- `lite_flash_*` uses the lightweight collision style from the older
  `bhl_arm_1` model: sparse primitives and gripper collision meshes intended to
  avoid false self-collisions in simple importers.
- `lite_pro_*` uses visual STL meshes as collision meshes for every link with a
  visual mesh. Camera and task-frame links remain collisionless because they do
  not have visual meshes.

For Pro, adjacent joint-stack visual meshes can overlap or touch in the default
pose. This is expected for a visual-mesh collision asset and should be handled
with simulator-side self-collision filtering:

- MuJoCo filters parent-child body contacts by default and supports
  `contype`/`conaffinity` plus explicit `<exclude>` pairs.
- Genesis defaults to disabling adjacent collision.
- Gazebo/SDFormat does not collide joint-connected links when model
  self-collision is enabled.
- Isaac/PhysX should be checked with filtered collision pairs for overlapping
  robot internals.

Keep the contact plane clear of unintended visual mesh contact at the default
pose. The internal robot contacts are the pairs intended to be filtered.

Remaining physics work:

```text
choose Flash or Pro collision variant for the target simulator
verify visual-mesh collision behavior and self-collision filtering if using Pro
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
