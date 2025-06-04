# Unity Scene Setup

This folder contains the Unity components required to stream controller data to ROS 2.

1. **websocket-sharp.dll** – WebSocket library used by the scene to connect to a running ROS bridge (e.g. `rosbridge_server`). Place this DLL inside a `Plugins` folder in your Unity project.
2. **RosBridgeTwistPublisher.cs** – Attach this script to a GameObject and assign the `rightController` and `thumbStick` transforms. It publishes:
   - `/unity/controller_pose` (`geometry_msgs/Twist`)
   - `/unity/touchpad` (`std_msgs/Float32`)

Make sure a ROS bridge is running on `ws://localhost:9090` or adjust the URL in the script. When you run the Unity scene the controller pose and touchpad value will be sent to the ROS 2 workspace.

From a Windows machine you can forward the websocket port using an elevated PowerShell:

```powershell
ssh -L 9090:localhost:9090 user@255.255.255.0
```

Then start the bridge on the Ubuntu side:

```bash
ros2 launch rosbridge_server rosbridge_websocket.launch.py
```
