# xArm6 Teleoperation Workspace

This repository contains a ROS 2 Humble workspace and a companion Unity project
for teleoperating an xArm6 robot. Controller poses from Unity are streamed over
a WebSocket and converted into Cartesian commands for the arm.

The workspace is organised as follows:

- **`unity/`** – Unity assets and scripts used to publish controller poses.
- **`src/`** – ROS 2 packages with the bridge and control nodes.

Each directory includes its own README with further details.

## Prerequisites

- Ubuntu 22.04 with **ROS 2 Humble** installed
- **Unity 6000.1.4** (matching version recommended)
- `rosbridge_server` from `rosbridge_suite`
- An xArm6 robot or simulator reachable on the network
## Unity and ROS 2 Interaction

`RosBridgeTwistPublisher.cs` publishes controller data via WebSocket to
`rosbridge_server`. The `unity_transform` package then converts these messages
into pose commands that are consumed by the `xarm_control` nodes.

## Typical Usage

First, start the WebSocket bridge (see `unity/README.md`).\
Then run the converter and controller:

```bash
ros2 run unity_transform unity_to_xarm
ros2 run xarm_control xarm_control --ros-args -p robot_ip:=<ip>
```

Finally, launch the Unity scene and move the controller to drive
the robot.

---

**Author:** bourinto – Tokyo University of Science, Yoshida Laboratory