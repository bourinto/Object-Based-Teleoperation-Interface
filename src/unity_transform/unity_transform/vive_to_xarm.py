#!/usr/bin/env python3
"""
Real-time converter from Vive controller pose to xArm6 end-effector commands.
Subscribes to Unity Twist messages for controller pose and publishes Cartesian
Twist commands to the xArm6.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np


class ViveToXarm(Node):
    """ROS2 node to transform Vive controller pose into xArm6 Cartesian commands."""

    def __init__(self):
        super().__init__("unity_to_xarm")

        # Declare translation offsets (in millimeters)
        self.declare_parameter("offset_x", 200)
        self.declare_parameter("offset_y", -412)
        self.declare_parameter("offset_z", -611)
        # Declare rotation offsets (in degrees)
        self.declare_parameter("offset_roll", 0)
        self.declare_parameter("offset_pitch", 90)
        self.declare_parameter("offset_yaw", 30)
        self.offset_x = self.get_parameter("offset_x").value
        self.offset_y = self.get_parameter("offset_y").value
        self.offset_z = self.get_parameter("offset_z").value
        self.offset_roll = self.get_parameter("offset_roll").value
        self.offset_pitch = self.get_parameter("offset_pitch").value
        self.offset_yaw = self.get_parameter("offset_yaw").value

        # Precompute 2D rotation matrix for yaw adjustment around Z-axis
        theta = np.radians(self.offset_yaw)
        self.R_matrix = np.array(
            [
                [np.cos(theta), -np.sin(theta)],
                [np.sin(theta), np.cos(theta)],
            ]
        )

        # Subscribe to Unity controller pose topic
        self.sub = self.create_subscription(
            Twist, "/unity/controller_pose", self.cb, 10
        )
        # Publisher for xArm6 end-effector pose commands
        self.pub = self.create_publisher(Twist, "/xarm6/ee_pose_cmd", 10)

    def cb(self, msg):
        """
        Callback for incoming controller pose messages.
        Transforms Vive pose into xArm6 Cartesian command and publishes it.
        """
        new_msg = Twist()

        # Map Unity axes (m) to robot axes (mm) and apply yaw rotation
        pose = self.R_matrix @ np.array(
            [
                -(msg.linear.z) * 1000.0,
                (msg.linear.x) * 1000.0,
            ]
        )

        # Compute Cartesian position and orientation
        new_msg.linear.x = pose[0] + self.offset_x
        new_msg.linear.y = pose[1] + self.offset_y
        new_msg.linear.z = (msg.linear.y * 1000.0) + self.offset_z

        new_msg.angular.x = (msg.angular.z + self.offset_roll) % 360 - 180
        new_msg.angular.y = (msg.angular.x + self.offset_pitch) % 360 - 180
        new_msg.angular.z = (-msg.angular.y + self.offset_yaw) % 360 - 180

        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ViveToXarm()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
