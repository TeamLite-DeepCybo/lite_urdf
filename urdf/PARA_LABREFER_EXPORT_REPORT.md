# para_Labrefer URDF export report

These files are physically loadable candidates, not a guarantee of the true
per-link inertial distribution. The source least-squares problem is rank
deficient, so its raw individual inertial values are not physically unique.

The `para_Labrefer` suffix means the URDF is intended for the laboratory
reference setup and specified assembly. It contains identified dynamics
fields: center-of-mass position, mass, rotational inertia matrix, viscous
damping, and dry Coulomb friction.

- Source parameters: `identified_params.json`
- Fit RMSE: `0.23291045130782007 Nm`
- Regressor rank: `132 / 222`
- Projection epsilon: `1e-08`
- Maximum accepted projected COM norm: `0.3 m`
- Raw non-PSD inertia groups: `18 / 18`
- Projected groups written per URDF: `16`
- Axis-aligned negligible-mass groups per URDF: `2`
- Other nominal fallback groups per URDF: `0`

## Outputs

- `urdf/lite_flash_arm_gripper_para_Labrefer.urdf`
- `urdf/lite_flash_arm_gripper_headless_para_Labrefer.urdf`
- `urdf/lite_pro_arm_gripper_para_Labrefer.urdf`
- `urdf/lite_pro_arm_gripper_headless_para_Labrefer.urdf`

Only the gripper variants are exported because para_Labrefer was identified with
the gripper model. Applying its distal inertia to the O6 hand variants would
describe a different mechanism.

## Representative inertial export

All four outputs share the same moving-link inertia and drive values.

| identified body | target link | action | raw mass (kg) | output mass (kg) | output COM norm (m) | raw pseudo min eig | output inertia min eig |
|---|---|---|---:|---:|---:|---:|---:|
| left_elbow_pitch | left_elbow_pitch_link | write_projected | 0.20938569 | 0.20988193 | 0.14817979 | -0.034833202 | 2.016288e-08 |
| left_gripper | left_gripper_finger_left_link | write_projected | 0.077707574 | 0.077710018 | 0.056098796 | -0.0011013007 | 2.0031385e-08 |
| left_gripper_passive_joint | left_gripper_finger_right_link | write_projected | 0.081412022 | 0.081414197 | 0.053407589 | -0.0010960904 | 2.0028445e-08 |
| left_shoulder_pitch | left_shoulder_pitch_link | keep_nominal_axis_aligned_regularized | 5.7934779e-18 | 4.97e-05 | 0.017422 | -0.060693249 | 1e-08 |
| left_shoulder_roll | left_shoulder_roll_link | write_projected | 1.1627053 | 1.1628652 | 0.091859993 | -0.060285268 | 2.0031741e-08 |
| left_shoulder_yaw | left_shoulder_yaw_link | write_projected | 0.36226796 | 0.36257604 | 0.11891565 | -0.046142858 | 2.0088187e-08 |
| left_wrist_pitch | left_wrist_pitch_link | write_projected | 0.14466724 | 0.14466806 | 0.03469441 | -0.0014176126 | 2.0009136e-08 |
| left_wrist_roll | left_wrist_roll_link | write_projected | 0.13702142 | 0.13704747 | 0.055390333 | -0.012739115 | 2.0029911e-08 |
| left_wrist_yaw | left_wrist_yaw_motor_link | write_projected | 0.20784004 | 0.20862065 | 0.16853224 | -0.030007666 | 0.00020346358 |
| right_elbow_pitch | right_elbow_pitch_link | write_projected | 0.2234522 | 0.22406738 | 0.13335335 | -0.038096571 | 2.0165655e-08 |
| right_gripper | right_gripper_finger_left_link | write_projected | 0.077646595 | 0.077648645 | 0.063089736 | -0.0019930219 | 2.0037659e-08 |
| right_gripper_passive_joint | right_gripper_finger_right_link | write_projected | 0.074889935 | 0.074892183 | 0.065485259 | -0.0019956084 | 2.0040607e-08 |
| right_shoulder_pitch | right_shoulder_pitch_link | keep_nominal_axis_aligned_regularized | 1.1444474e-16 | 4.97e-05 | 0.017422 | -0.041479222 | 1e-08 |
| right_shoulder_roll | right_shoulder_roll_link | write_projected | 0.67530051 | 0.67655916 | 0.16579856 | -0.053988486 | 2.023875e-08 |
| right_shoulder_yaw | right_shoulder_yaw_link | write_projected | 0.39255637 | 0.39326521 | 0.11415291 | -0.057626177 | 2.012837e-08 |
| right_wrist_pitch | right_wrist_pitch_link | write_projected | 0.13681893 | 0.13682098 | 0.038046151 | -0.0026596758 | 2.0007962e-08 |
| right_wrist_roll | right_wrist_roll_motor_link | write_projected | 0.1316142 | 0.13161916 | 0.035893033 | -0.0061470451 | 2.0011026e-08 |
| right_wrist_yaw | right_wrist_yaw_motor_link | write_projected | 0.21435238 | 0.21488905 | 0.13732878 | -0.030215931 | 7.4600741e-05 |

The shoulder-pitch links are nearly unobservable in this identification because
their physical mass distribution lies predominantly along the pitch rotation
axes. The JSON `mx`, `my`, and `mz` values are first moments, not COM
coordinates. Dividing them by the near-zero identified mass is ill-conditioned;
the approximately 0.92 m values produced by unconstrained PSD projection are not
physical COM estimates. The exporter therefore retains the small nominal
mass/inertia needed for simulator stability and projects each nominal inertial
origin onto its joint axis.

| identified body | output COM xyz (m) | output first moment xyz (kg m) | output inertia diagonal (kg m^2) |
|---|---:|---:|---:|
| left_shoulder_pitch | `-0.017422 0 0` | `-8.658734e-07 0 0` | `1e-08 1e-08 1e-08` |
| right_shoulder_pitch | `0.017422 0 0` | `8.658734e-07 0 0` | `1e-08 1e-08 1e-08` |

## Drive export

| target joint | damping | raw Coulomb | URDF friction | effort offset |
|---|---:|---:|---:|---:|
| left_elbow_pitch_joint | 0.20839211 | 0.089132873 | 0.089132873 | -0.23504683 |
| left_shoulder_pitch_joint | 0.53367973 | 0.020904733 | 0.020904733 | 0.94260036 |
| left_shoulder_roll_joint | 0.49619136 | 0.15516465 | 0.15516465 | 0.48040028 |
| left_shoulder_yaw_joint | 0.053060908 | 0.067145172 | 0.067145172 | -0.21226589 |
| left_wrist_pitch_joint | 0.012360968 | 0.021280627 | 0.021280627 | 0.011843624 |
| left_wrist_roll_joint | 0.057922865 | 0.022105433 | 0.022105433 | -0.024704439 |
| left_wrist_yaw_joint | 0.010935759 | 0.029245428 | 0.029245428 | -0.013606262 |
| right_elbow_pitch_joint | 0.26588298 | 0.08342476 | 0.08342476 | -0.318645 |
| right_shoulder_pitch_joint | 0.66990946 | -0.0040399933 | 0 | 0.4400405 |
| right_shoulder_roll_joint | 0.60210176 | 0.14223079 | 0.14223079 | -0.29942419 |
| right_shoulder_yaw_joint | 0.18446161 | 0.056097761 | 0.056097761 | 0.1361161 |
| right_wrist_pitch_joint | 0.025139083 | 0.019814467 | 0.019814467 | -0.0305686 |
| right_wrist_roll_joint | 0.29323824 | 0.036205224 | 0.036205224 | 0.029651431 |
| right_wrist_yaw_joint | 0.012878703 | 0.023934911 | 0.023934911 | -0.021182313 |

`damping` and non-negative Coulomb friction are written to standard URDF
`<dynamics>` fields. Constant effort offsets are reported in the table
above for reference only; standard URDF has no equivalent field, and an
offset may include torque-sensor bias rather than passive mechanical
friction.

## Required validation

Before selecting a generated URDF as the runtime default, rerun torque
prediction on the held-out trajectories using the projected URDF parameters.
The existing validation metrics apply to the raw unconstrained parameter
vector, not to this PSD-projected candidate.
