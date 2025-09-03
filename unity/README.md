# Unity Environment

This project was built using **[Unity 6000.1.4](https://unity.com/releases/editor/whats-new/6000.1.4)**. Please use this version for best compatibility.

## MetaQuest3 - xArm6 Ready Scene

If you're using a **Meta Quest 3**, a preconfigured Unity scene is available **[Here](https://drive.google.com/drive/folders/1r1TNFW372G5PVKYh7jwA6eGltpLQNAVy?usp=drive_link)**!

1. Unzip `MRInterfaceForObjectBasedManipulation.zip`.
2. Open it as a *New Project* in Unity.

This scene is preconfigured for the Meta Quest 3 and ready to communicate with ROS 2.
Before launching, ensure your headset is connected via Link or Air Link and recognized by Unity.

## Creating Your Own Unity Scene

You’ll find all the key components needed to stream Meta Quest 3 controller data to ROS 2 in this folder. If you'd rather build your own scene or integrate the features manually, use the following assets:

1. **`websocket-sharp.dll`** – WebSocket library that connects Unity to a running ROS bridge (e.g., `rosbridge_server`).
   Place this file in a `Plugins` folder inside your Unity project.

2. **`RosBridgeTwistPublisher.cs`** – Attach this script to a GameObject, assign the controller, thumbstick, and trigger transforms, and it publishes `/unity/controller_pose`, `/unity/touchpad`, and `/unity/trigger`.

3. **`RosBridgeJointSubscriber.cs`** – Subscribes to xArm joint angles and gripper commands to animate the robot model in Unity.

4. **`CylinderProjection.cs`** – Projects a controller position onto a cylinder surface and sends the locked point to ROS 2.

## WebSocket Setup

Whether you are using the preconfigured scene or your own, ensure a ROS bridge is running at `ws://localhost:9090`, or update the URL in the script accordingly.

To forward the WebSocket port from a Windows machine, run this in an **elevated PowerShell**:

```powershell
ssh -L 9090:localhost:9090 user@255.255.255.0
```

Once connected, launch the bridge:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
