# Unity Environment

This project was built using **[Unity 6000.1.4](https://unity.com/releases/editor/whats-new/6000.1.4)**. Please use this version for best compatibility.

## HTC Vive-Ready Scene

If you're using an **HTC Vive**, a preconfigured Unity scene is available **[Here](https://drive.usercontent.google.com/download?id=1DA54HXuRPVssflVRsW1H0aUAetDlI9Bc&export=download)**!

1. Unzip `VivetoXarm.zip`.
2. Open it as a *New Project* in Unity.

This scene is preconfigured for the HTC Vive right controller and ready to communicate with ROS 2.
Before launching, ensure:

* **SteamVR is running** (mandatory for Vive-Unity).
* **Your play area is calibrated** via SteamVR.

## Creating Your Own Unity Scene

You’ll find all the key components needed to stream controller data to ROS 2 in this folder. If you'd rather build your own scene or integrate the features manually, use the following assets:

1. **`websocket-sharp.dll`** – WebSocket library that connects Unity to a running ROS bridge (e.g., `rosbridge_server`).
   Place this file in a `Plugins` folder inside your Unity project.

2. **`RosBridgeTwistPublisher.cs`** – Attach this script to a GameObject, and assign the `rightController` and `thumbStick` transforms. It publishes:

   * `/unity/controller_pose` (`geometry_msgs/Twist`)
   * `/unity/touchpad` (`std_msgs/Float32`)

## WebSocket Setup

Whether you are using the preconfigured scene or your own, ensure a ROS bridge is running at `ws://localhost:9090`, or update the URL in the script accordingly.

To forward the WebSocket port from a Windows machine, run this in an **elevated PowerShell**:

```powershell
ssh -L 9090:localhost:9090 user@255.255.255.0
```

Then, on the Ubuntu machine, launch the bridge:

```bash
ros2 launch rosbridge_server rosbridge_websocket.launch.py
```
