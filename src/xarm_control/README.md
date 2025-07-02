# xarm_control

ROS 2 package containing nodes for commanding the xArm6.

## Nodes

### `xarm_control`
Real-time Cartesian controller for the xArm6.

- **Subscriptions**
  - `/xarm6/ee_pose_cmd` (`geometry_msgs/Twist`): target pose of the end effector.
  - `/xarm6/gripper_cmd` (`control_msgs/GripperCommand`): gripper position and effort.
- **Publications**
  - `/xarm6/ee_pose_current` (`geometry_msgs/Twist`): pose feedback published at 50 Hz.
  - `/xarm6/joints_values` (`std_msgs/Float64MultiArray`): current joint angles published at 50 Hz.
- **Parameters**
  - `robot_ip` – IP address of the arm (default `192.168.1.217`).

Run the controller with:

```bash
ros2 run xarm_control xarm_control --ros-args -p robot_ip:=<robot-ip>
```

### `demo_circle_publisher`
Publishes a simple circular trajectory on `/xarm6/ee_pose_cmd` for testing.

### `teleop_keyboard`
Keyboard interface that sends Cartesian pose commands and gripper commands. See the source for key bindings.
