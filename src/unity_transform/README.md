# unity_transform

ROS 2 package that converts Vive controller data from Unity into xArm6 commands.

## Node

### `vive_to_xarm`

Unified node that converts Vive controller poses into xArm6 Cartesian commands.
It supports two modes:

- **Relative** (`--is_relative True`, default) – controller motion is applied
  relative to the robot pose when the trigger button is pressed.
- **Absolute** (`--is_relative False`) – controller pose is mapped directly into
  xArm coordinates.

The relative mode implementation is inspired by the approach presented in
[this paper](https://ieeexplore.ieee.org/document/9197517).

- **Subscriptions**
  - `/unity/controller_pose` (`geometry_msgs/Twist`): pose of the controller.
  - `/unity/touchpad` (`std_msgs/Float32`): touchpad position used for gripper control.
  - `/unity/trigger` (`std_msgs/Bool`): enables relative motion while pressed.
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
ros2 run unity_transform vive_to_xarm --is_relative True
```
