# unity_transform

ROS 2 package that converts Vive controller data from Unity into xArm6 commands.

## Node

### `vive_absolute_motion`

- **Subscriptions**
  - `/unity/controller_pose` (`geometry_msgs/Twist`): pose of the controller.
  - `/unity/touchpad` (`std_msgs/Float32`): touchpad position used for gripper control.
- **Publications**
  - `/xarm6/ee_pose_cmd` (`geometry_msgs/Twist`): Cartesian pose command.
  - `/xarm6/gripper_cmd` (`control_msgs/GripperCommand`): gripper command.
- **Parameters**
  - `offset_x`, `offset_y`, `offset_z` – translation offsets in millimetres.
  - `offset_roll`, `offset_pitch`, `offset_yaw` – orientation offsets in degrees.

These offsets were tuned for one Vive room configuration and may need
adjustment in a different environment.

Run the node with:

```bash
ros2 run unity_transform vive_absolute_motion
```
