# xArm6 Cartesian Controller

A lightweight ROS 2 Humble package for real-time Cartesian control of the xArm6. It includes:

* A controller node publishing end‑effector pose commands.
* A sinusoidal-trajectory demo publisher.
* A keyboard teleoperation node for direct manual control.


## Quick start

```bash
# Controller: publishes to /xarm6/ee_pose_cmd
ros2 run xarm_control ee_pose_controller

# Sinusoidal demo
ros2 run xarm_control ee_demo_sinus_publisher

# Keyboard teleop: use WASD, QE, IJKL, UO to move
ros2 run xarm_control teleop_keyboard
```


## Keyboard Teleoperation

**Features:**
* Hold keys to incrementally adjust pose derivatives.
* Press <kbd>Esc</kbd> to return to the initial pose and exit.

**Default bindings:**

| Key | Motion  |   |   | Key | Motion    |
| --- | ------- | - | - | --- | --------- |
| w/s | +X / -X |   |   | i/k | +Rx / -Rx |
| a/d | +Y / -Y |   |   | j/l | +Ry / -Ry |
| q/e | +Z / -Z |   |   | u/o | +Rz / -Rz |

**Initial pose:**

* Linear: `[210.0, 0.0, 115.0]`
* Angular: `[-180.0, 0.0, 0.0]`

You can change bindings and scaling values directly in `teleop_keyboard.py`, especially in the `key_bindings` dictionary.

---

**Author:** bourinto\
Tokyo University of Science\
Yoshida Laboratory
