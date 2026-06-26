# Gripper Coupling

Each gripper has two physical prismatic finger joints driven by a center gear in the real mechanism.
The two fingers must move together to avoid sim-to-real mismatch.

In the URDF, each hand exposes one actuated gripper joint. The passive finger
joint on each gripper mimics that hand-level joint:

```text
left_gripper_passive_joint  mimics left_gripper_joint
right_gripper_passive_joint mimics right_gripper_joint
```

The mimic multiplier is `1.0`.
The finger joint axes are opposite directions, so equal scalar displacement produces symmetric open/close motion.

Controllers should command only `left_gripper_joint` and `right_gripper_joint`.
Do not create separate controllers for individual finger joints.

If the target simulator ignores URDF `mimic` tags, implement this coupling in the simulator actuator/controller layer.
