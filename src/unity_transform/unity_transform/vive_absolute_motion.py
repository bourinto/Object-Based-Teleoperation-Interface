#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from control_msgs.msg import GripperCommand
import numpy as np


class ViveToXarm(Node):
    def __init__(self):
        super().__init__("vive_absolute_motion")
        self.get_logger().info("Starting Vive ABSOLUTE motion node")

        self.declare_parameter("offset_x", 200)
        self.declare_parameter("offset_y", -412)
        self.declare_parameter("offset_z", -611)
        self.declare_parameter("offset_roll", 0)
        self.declare_parameter("offset_pitch", 90)
        self.declare_parameter("offset_yaw", 30)
        self.declare_parameter("grip_cmd", 500.0)
        self.declare_parameter("grip_kp", 50.0)

        self.offset_x = self.get_parameter("offset_x").value
        self.offset_y = self.get_parameter("offset_y").value
        self.offset_z = self.get_parameter("offset_z").value
        self.offset_roll = self.get_parameter("offset_roll").value
        self.offset_pitch = self.get_parameter("offset_pitch").value
        self.offset_yaw = self.get_parameter("offset_yaw").value
        self.grip_cmd = self.get_parameter("grip_cmd").value
        self.grip_kp = self.get_parameter("grip_kp").value

        self.get_logger().info(
            f"Offsets: x={self.offset_x}, y={self.offset_y}, z={self.offset_z}, "
            f"roll={self.offset_roll}, pitch={self.offset_pitch}, yaw={self.offset_yaw}"
        )
        self.get_logger().info(
            f"Gripper initial position: {self.grip_cmd}, Kp: {self.grip_kp}"
        )

        theta = np.radians(self.offset_yaw)
        self.R_matrix = np.array(
            [
                [np.cos(theta), -np.sin(theta)],
                [np.sin(theta), np.cos(theta)],
            ]
        )

        self.sub = self.create_subscription(
            Twist, "/unity/controller_pose", self.cb, 10
        )

        self.grip_sub = self.create_subscription(
            Float32, "/unity/touchpad", self.touchpad_cb, 10
        )

        self.pub = self.create_publisher(Twist, "/xarm6/ee_pose_cmd", 10)

        self.grip_pub = self.create_publisher(GripperCommand, "/xarm6/gripper_cmd", 10)

        time.sleep(1)

    def cb(self, msg):
        self.get_logger().debug(
            f"Received controller pose: linear=({msg.linear.x:.3f}, {msg.linear.y:.3f}, {msg.linear.z:.3f}), "
            f"angular=({msg.angular.x:.3f}, {msg.angular.y:.3f}, {msg.angular.z:.3f})"
        )

        pose = self.R_matrix @ np.array(
            [
                -(msg.linear.z) * 1000.0,
                (msg.linear.x) * 1000.0,
            ]
        )

        new_msg = Twist()
        new_msg.linear.x = pose[0] + self.offset_x
        new_msg.linear.y = pose[1] + self.offset_y
        new_msg.linear.z = (msg.linear.y * 1000.0) + self.offset_z

        new_msg.angular.x = 180.
        new_msg.angular.y = ((msg.angular.x + self.offset_pitch + 180) % 360) - 180
        new_msg.angular.z = ((-msg.angular.y + self.offset_yaw + 180) % 360) - 180

        self.pub.publish(new_msg)
        self.get_logger().info(
            f"Published xArm6 pose: linear=({new_msg.linear.x:.1f}, {new_msg.linear.y:.1f}, {new_msg.linear.z:.1f}), "
            f"angular=({new_msg.angular.x:.1f}, {new_msg.angular.y:.1f}, {new_msg.angular.z:.1f})"
        )

    def touchpad_cb(self, msg):
        self.get_logger().debug(f"Touchpad input: {msg.data:.3f}")
        prev_cmd = self.grip_cmd
        self.grip_cmd = np.clip(self.grip_cmd + self.grip_kp * msg.data, -10, 850)
        gripper_msg = GripperCommand()
        gripper_msg.position = self.grip_cmd
        gripper_msg.max_effort = 800.0
        self.grip_pub.publish(gripper_msg)
        self.get_logger().info(
            f"Gripper command updated from {prev_cmd:.1f} to {self.grip_cmd:.1f}"
        )


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
