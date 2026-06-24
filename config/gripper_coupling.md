# Gripper Coupling

Each gripper has two physical prismatic finger joints driven by a center gear in the real mechanism.
The two fingers must move together to avoid sim-to-real mismatch.

In the URDF, the right finger on each gripper mimics the left finger:

```text
left_gripper_right_finger_joint  mimics left_gripper_left_finger_joint
right_gripper_right_finger_joint mimics right_gripper_left_finger_joint
```

The mimic multiplier is `1.0`.
The finger joint axes are opposite directions, so equal scalar displacement produces symmetric open/close motion.

If the target simulator ignores URDF `mimic` tags, implement this coupling in the simulator actuator/controller layer.
