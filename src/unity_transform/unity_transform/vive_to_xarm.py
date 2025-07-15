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
        self.declare_parameter("offset_yaw", 0)
        self.declare_parameter("grip_cmd", 355.0)
        self.declare_parameter("grip_kp", 10.0)

        self.offsets = {
            "x": self.get_parameter("offset_x").value,
            "y": self.get_parameter("offset_y").value,
            "z": self.get_parameter("offset_z").value,
            "roll": self.get_parameter("offset_roll").value,
            "pitch": self.get_parameter("offset_pitch").value,
            "yaw": self.get_parameter("offset_yaw").value,
        }
        self.grip_cmd = self.get_parameter("grip_cmd").value
        self.grip_kp = self.get_parameter("grip_kp").value

        self.get_logger().info(
            "Offsets: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}".format(
                **self.offsets
            )
        )
        self.get_logger().info(
            f"Gripper initial position: {self.grip_cmd}, Kp: {self.grip_kp}"
        )

        # Rotation matrix for Unity->xArm frame conversion
        theta = np.radians(self.offsets["yaw"])
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

    # ROS callbacks -----------------------------------------------------
    def cb(self, msg: Twist) -> None:
        self.motion.handle_pose(msg)

    def touchpad_cb(self, msg: Float32) -> None:
        self.grip_cmd = float(np.clip(self.grip_cmd - self.grip_kp * msg.data, -10, 850))
        gripper_msg = GripperCommand()
        gripper_msg.position = self.grip_cmd
        gripper_msg.max_effort = 1500.
        self.grip_pub.publish(gripper_msg)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    parser = argparse.ArgumentParser(description="Vive to xArm teleoperation")
    parser.add_argument(
        "--is_relative",
        default="True",
        help="Set to False for absolute mode",
    )
    args, ros_args = parser.parse_known_args(argv)
    is_relative = str(args.is_relative).lower() in ("true", "1", "t", "yes", "y")

    rclpy.init(args=ros_args)
    node = ViveToXarm(is_relative=is_relative)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
