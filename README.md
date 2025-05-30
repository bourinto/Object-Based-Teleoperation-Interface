# xArm6 Cartesian Controller

This workspace contains two ROS 2 packages for streaming controller poses from Unity into Cartesian commands for the xArm6 robot and for real-time control of the xArm6 itself.

## Packages

### unity_transform

**Description:** \
Converts Vive controller pose (from Unity) into Cartesian end-effector commands for the xArm6.

**Key Features:**
- Subscribes to `/unity/controller_pose` (geometry_msgs/Twist).
- Transforms poses from the Unity coordinate frame to the robot’s reference frame.
- Publishes Cartesian commands to `/xarm6/ee_pose_cmd`.

**Run:**  
```bash
ros2 run unity_transform vive_to_xarm
```

**Parameters (with defaults):**

Offsets are specific to one particular Vive setup and may need to be adjusted for others. Translations are in millimeters, and rotations are in degrees.

---

### xarm\_control

**Description:**\
Lightweight ROS 2 package for real-time Cartesian control of the xArm6.

**Includes:**

* **`ee_pose_controller`**:
  Streams real-time Cartesian servo commands to the xArm6 based on incoming `/xarm6/ee_pose_cmd` messages.
* **`ee_demo_sinus_publisher`**:
  Publishes a simple sinusoidal end-effector trajectory for demonstration.
* **`teleop_keyboard`**:
  Keyboard teleoperation interface for manual Cartesian control and gripper toggling.

**Run:**

```bash
# Controller node
ros2 run xarm_control ee_pose_controller

# Sinusoidal trajectory demo
ros2 run xarm_control ee_demo_sinus_publisher

# Keyboard teleoperation
ros2 run xarm_control teleop_keyboard
```

---

## Keyboard Teleoperation

**Features:**

* Hold keys to incrementally adjust the end-effector pose.
* Press <kbd>Esc</kbd> to reset to initial pose and exit.
* Press <kbd>Enter</kbd> or <kbd>Backspace</kbd> to open/close the gripper.

**Default Key Bindings:**

| Key   | Motion  |   | Key   | Motion    |
| ----- | ------- | - | ----- | --------- |
| w / s | +X / –X |   | i / k | +Rx / –Rx |
| d / a | +Y / –Y |   | l / j | +Ry / –Ry |
| e / q | +Z / –Z |   | o / u | +Rz / –Rz |


You can modify key bindings and scaling factors in the `key_bindings` dictionary inside `teleop_keyboard.py`.

---

**Author:** bourinto\
Tokyo University of Science, Yoshida Laboratory