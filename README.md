# Object-Based Teleoperation Interface for Collaborative Manipulation
This repository is associated with a research article of the same name.
We recommend reading it beforehand to better understand the concepts
and functionality of this project.

This framework contains a ROS 2 Humble workspace and a companion Unity project
for teleoperating an xArm6 robot.  

It is organized as follows:

- **`unity/`** – Unity assets and scripts used to publish controller poses.
- **`src/`** – ROS 2 packages with the bridge and control nodes.

Each directory includes its own README with further details.

## Prerequisites

- Ubuntu 22.04 with **ROS 2 Humble** installed
- **Unity 6000.1.4** (matching version recommended)
- `rosbridge_server` from `rosbridge_suite`
- An xArm6 robot or simulator reachable on the network

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

**Author:** T. Bourin
