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
- `urdf/*_gripper*_para_Labrefer.urdf` - physically projected dynamics
  candidates for the lab reference setup and specified assembly.
- `para_Labrefer/identified_params.json` - original identified dynamics
  parameter file retained as the export reference.
- `scripts/export_para_Labrefer_urdf.py` - reproducible para_Labrefer
  projection/export tool.

## Current State

- Visual materials use an EVA-inspired color palette mirrored across the left
  and right arms, with orange gripper fingers and shoulder-roll links.
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
- Arm joint order and positive directions follow the older `bhl_arm_1` scheme.
  The shoulder-roll zero pose is shifted to match `bar_description_lite` /
  `lite_ros2`, where `q=0` places the arms parallel to the ground. Shoulder-roll
  lower/upper limits preserve the old physical endpoints after that zero shift.
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

Recommended starting points for RL are:

```text
urdf/lite_flash_arm_gripper.urdf
urdf/lite_pro_arm_gripper.urdf
```

Both keep the cleaned CAD assembly while preserving 14 arm revolute joints, 2
gripper prismatic control joints, and 2 gripper mimic constraints.

### Joint Convention Compatibility

Arm joints keep the older `bhl_arm_1` XML ordering and positive directions. The
movable joint order is:

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

The shoulder-roll zero pose is intentionally aligned with
`bar_description_lite` / `lite_ros2`, where `q=0` places the arms parallel to
the ground. The old physical endpoints are preserved by shifting the coordinate
zero inside the old range:

```text
left_shoulder_roll_joint   old q at new zero =  1.39626215 rad  (45% through old range)
right_shoulder_roll_joint  old q at new zero = -1.39626215 rad  (55% through old range)
```

Avoid adding simulator-side sign flips, reordered joint lists, or extra home
offsets unless the target stack explicitly needs them.

### Gripper Coupling

Each physical gripper has two prismatic finger joints driven by one mechanism.
In the URDF, each hand exposes one actuated gripper joint and one passive mimic
joint:

```text
left_gripper_passive_joint  mimics left_gripper_joint
right_gripper_passive_joint mimics right_gripper_joint
```

Controllers should command only `left_gripper_joint` and `right_gripper_joint`.
Do not create separate controllers for individual finger joints. The mimic
multiplier is `1.0`; the finger axes are opposite directions, so equal scalar
displacement produces symmetric open/close motion. If the target simulator
ignores URDF `mimic` tags, implement the same-side coupling in the simulator
actuator/controller layer.

The gripper control coordinate starts at the lower limit:

```text
left_gripper_joint  initial=0.0 lower=0.0 upper=0.047
right_gripper_joint initial=0.0 lower=0.0 upper=0.047
```

### Frames and Imports

The merged URDF keeps `head_camera_link`, `left_wrist_camera_link`,
`right_wrist_camera_link`, `left_gripper_tip_middle_link`, and
`right_gripper_tip_middle_link`. Optical camera frames are removed; add
simulator-specific site/frame definitions outside this URDF if a downstream
policy needs optical targets.

Use the simulator/importer fixed-joint merge option instead of a hand-collapsed
URDF. Keep MuJoCo `fusestatic` enabled, use Isaac/Genesis fixed-joint merge
options where appropriate, and preserve named task frames only when the target
simulator needs them. Do not use an automatically collapsed simplification
unless it has been visually verified.

### Collision Notes

`lite_flash_*` uses sparse lightweight collision geometry inherited from the
older `bhl_arm_1` model. `lite_pro_*` uses visual STL meshes as collision meshes
for every visual link, while camera and task-frame links remain collisionless.

For Pro, adjacent joint-stack visual meshes can overlap or touch in the default
pose. This is expected and should be handled with simulator-side self-collision
filtering. Keep the contact plane clear of unintended visual mesh contact at the
default pose; internal robot contacts are the pairs intended to be filtered.

Before high-throughput RL training, add or verify:

- actuator/transmission metadata
- simulator support for URDF `mimic`, or equivalent gripper coupling in simulator config
- inertial parameters
- simulator-specific configuration
- self-collision filtering when using the Pro visual-mesh collision URDFs

Use a headless URDF when the upper head/top assembly should be omitted from
visualization/import.

### para_Labrefer Identified Dynamics Candidates

Regenerate the para_Labrefer candidates after replacing the identification
results:

```bash
python3 scripts/export_para_Labrefer_urdf.py
```

The `_para_Labrefer.urdf` suffix means the file is a dynamics URDF for the
laboratory reference environment and the specified robot assembly. These files
carry identified dynamics content: center-of-mass position, mass, rotational
inertia matrix, viscous damping, and dry Coulomb friction. They are meant to
restore the measured lab-assembly behavior more closely in simulation than the
visual/CAD-oriented baseline URDFs.

The exporter writes the four gripper variants only. It does not export O6 hand
variants because the identification model used the two-finger gripper
mechanism. Identified inertial parameters are converted from Pinocchio's
link-origin convention to URDF center-of-mass inertias after a
positive-semidefinite pseudo-inertia projection. Projected centers of mass more
than `0.30 m` from the link origin are rejected in favor of a regularized
nominal block. The two shoulder-pitch groups are handled separately: their mass
is nearly unobservable because the physical structure lies predominantly along
the pitch axes. A free PSD projection produces artificial off-axis COM values,
so the exporter retains their small nominal mass/inertia and projects each
nominal inertial origin onto its joint axis.

Viscous and Coulomb terms are written to each arm joint's standard URDF
`<dynamics>` element. Constant effort offsets are listed in
`urdf/PARA_LABREFER_EXPORT_REPORT.md` for reference because URDF has no
standard field for them and they may contain torque-sensor bias. Review that
report before selecting a generated candidate at runtime. The source fit is
rank deficient (`132 / 222`), so the projected files are simulator-loadable
candidates rather than uniquely identified true per-link inertias.
