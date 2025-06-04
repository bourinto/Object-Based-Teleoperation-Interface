# ROS 2 Workspace

This directory holds the ROS 2 Humble packages. To build the workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Packages contained here:

- **`unity_transform`** – Converts controller data from Unity into Cartesian commands for the xArm6.
- **`xarm_control`** – Nodes for real-time Cartesian control, demo trajectories and keyboard teleoperation.

Each package has its own README for a description of the available nodes.
