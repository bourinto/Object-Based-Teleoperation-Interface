#!/usr/bin/env python3
"""Unified Vive to xArm control node."""
import argparse
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from control_msgs.msg import GripperCommand

from .absolute_motion import AbsoluteMotion
from .relative_motion import RelativeMotion


def sawtooth(angle: float) -> float:
    """Wrap angle to [-180, 180) degrees."""
    return (angle + 180.0) % 360.0 - 180.0


class ViveToXarm(Node):
    """Publish xArm Cartesian commands from Vive controller data."""

    TwistType = Twist  # alias used by helper modules

    def __init__(self, *, is_relative: bool = True):
        super().__init__("vive_to_xarm")
        mode = "RELATIVE" if is_relative else "ABSOLUTE"
        self.get_logger().info(f"Starting ViveToXarm node in {mode} mode")

        # Declare and read parameters common to both modes
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

        # Rotation matrix for Unity->xArm frame conversion
        theta = np.radians(self.offset_yaw)
        self.R_matrix = np.array(
            [
                [np.cos(theta), -np.sin(theta)],
                [np.sin(theta), np.cos(theta)],
            ]
        )

        # Publishers
        self.pub = self.create_publisher(Twist, "/xarm6/ee_pose_cmd", 10)
        self.grip_pub = self.create_publisher(GripperCommand, "/xarm6/gripper_cmd", 10)

        # Subscriptions common to both modes
        self.create_subscription(Twist, "/unity/controller_pose", self.cb, 10)
        self.create_subscription(Float32, "/unity/touchpad", self.touchpad_cb, 10)

        # Motion mode handler
        self.motion = RelativeMotion(self) if is_relative else AbsoluteMotion(self)

        time.sleep(1)

    # ------------------------------------------------------------------
    def convert_pose(self, msg: Twist) -> Twist:
        """Convert Unity controller pose to an xArm Cartesian command."""
        R = self.R_matrix
        xy = R @ [-(msg.linear.z) * 1000.0, msg.linear.x * 1000.0]
        converted = Twist()
        converted.linear.x = xy[0] + self.offset_x
        converted.linear.y = xy[1] + self.offset_y
        converted.linear.z = msg.linear.y * 1000.0 + self.offset_z
        converted.angular.x = 180.0
        converted.angular.y = sawtooth(msg.angular.x + self.offset_pitch)
        converted.angular.z = sawtooth(-msg.angular.y + self.offset_yaw)
        return converted

    # Helper conversion utilities used by the RelativeMotion class
    @staticmethod
    def twist_to_vector(t: Twist) -> np.ndarray:
        return np.array(
            [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z],
            dtype=float,
        )

    @staticmethod
    def vector_to_twist(vec: np.ndarray) -> Twist:
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = vec[0], vec[1], vec[2]
        t.angular.x, t.angular.y, t.angular.z = vec[3], vec[4], vec[5]
        return t

    # ROS callbacks -----------------------------------------------------
    def cb(self, msg: Twist) -> None:
        self.motion.handle_pose(msg)

    def touchpad_cb(self, msg: Float32) -> None:
        prev_cmd = self.grip_cmd
        self.grip_cmd = float(np.clip(self.grip_cmd + self.grip_kp * msg.data, -10, 850))
        gripper_msg = GripperCommand()
        gripper_msg.position = self.grip_cmd
        gripper_msg.max_effort = 800.0
        self.grip_pub.publish(gripper_msg)
        self.get_logger().info(
            f"Gripper command updated from {prev_cmd:.1f} to {self.grip_cmd:.1f}"
        )


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    parser = argparse.ArgumentParser(description="Vive to xArm teleoperation")
    parser.add_argument(
        "--is_relative",
        default="True",
        help="Set to False for absolute mode",
    )
    args, ros_args = parser.parse_known_args(argv)
    is_relative = str(args.is_relative).lower() in ("true", "1", "t", "yes")

    rclpy.init(args=ros_args)
    node = ViveToXarm(is_relative=is_relative)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
