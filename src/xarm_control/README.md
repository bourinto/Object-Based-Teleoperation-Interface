# xarm_control

ROS 2 package containing nodes for commanding the xArm6.

## Nodes

### `ee_pose_controller`
Real-time Cartesian controller for the xArm6.

- **Subscriptions**
  - `/xarm6/ee_pose_cmd` (`geometry_msgs/Twist`): target pose of the end effector.
  - `/xarm6/gripper_cmd` (`control_msgs/GripperCommand`): gripper position and effort.
- **Publications**
  - `/xarm6/ee_pose_current` (`geometry_msgs/Twist`): pose feedback published at 50 Hz.
- **Parameters**
  - `robot_ip` – IP address of the arm (default `192.168.1.217`).

Run the controller with:

```bash
ros2 run xarm_control ee_pose_controller --ros-args -p robot_ip:=<robot-ip>
```

### `ee_demo_sinus_publisher`
Publishes a simple sinusoidal trajectory on `/xarm6/ee_pose_cmd` for testing.

### `teleop_keyboard`
Keyboard interface that sends Cartesian pose commands and gripper commands. See the source for key bindings.
