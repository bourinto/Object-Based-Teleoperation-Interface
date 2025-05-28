#!/usr/bin/env python3
"""
Real-time Cartesian controller for the xArm6 robotic arm.
Subscribes to Twist messages for end-effector pose commands
and streams servo commands to the arm.
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from control_msgs.msg import GripperCommand
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from xarm.wrapper import XArmAPI


class XArmCartesianController(Node):
    """ROS2 node for controlling xArm in Cartesian space."""

    def __init__(self):
        super().__init__("ee_pose_controller")

        # Declare and read robot IP parameter
        self.declare_parameter("robot_ip", "192.168.1.217")
        ip = self.get_parameter("robot_ip").get_parameter_value().string_value

        # Initialize xArm API and enable motion
        self.arm = XArmAPI(ip, is_radian=False)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(1)
        self.arm.set_state(0)

        self.arm.clean_gripper_error()
        self.arm.set_gripper_enable(True)
        time.sleep(1)

        # Get current end-effector pose
        _, current_pose = self.arm.get_position(is_radian=False)
        self.ee_cmd = current_pose

        # Publisher for current end-effector pose at 50 Hz
        self.pose_pub = self.create_publisher(Twist, "/xarm6/ee_pose_current", 10)
        self.create_timer(0.02, self._publish_current_pose)

        # Subscribe to Cartesian pose commands
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Twist,
            "/xarm6/ee_pose_cmd",
            self._ee_command_callback,
            qos_profile,
        )

        self.create_subscription(
            GripperCommand, "/xarm6/gripper_cmd", self._gripper_cb, 1
        )

        # Timers for streaming commands and preventing lock
        self.create_timer(0.005, self._stream_callback)
        self.create_timer(1.0, self._anti_lock_callback)

        self.get_logger().info("xArm Cartesian realtime controller initialized.")
        self.get_logger().info(f"Initial xArm pose: {current_pose}")
        self.get_logger().info(
            f"Initial gripper pose: {self.arm.get_gripper_position()}"
        )

    def _publish_current_pose(self) -> None:
        _, pose = self.arm.get_position(is_radian=False)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = pose[:3]
        msg.angular.x, msg.angular.y, msg.angular.z = pose[3:]
        self.pose_pub.publish(msg)

    def _ee_command_callback(self, msg: Twist) -> None:
        """
        Store desired end-effector pose when a Twist message is received.
        """
        self.ee_cmd = [
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        ]
        self.get_logger().info(f"End-effector command received: {self.ee_cmd}")

    def _gripper_cb(self, msg):
        pos_mm = np.clip(msg.position, -10.0, 850.0)  # 0-85 mm for xArm gripper
        speed = np.clip(msg.max_effort, 350.0, 5000.0)

        self.arm.set_gripper_position(pos_mm, speed=speed, wait=False)
        # self.get_logger().info(
        #     f"Gripper command received: {(pos_mm/10.):.2f} mm, "
        #     f"speed: {speed:.2f} r/min"
        # )

    def _stream_callback(self) -> None:
        """Send interpolated Cartesian servo command to the robot."""
        command = self._compute_ee_command()
        self.arm.set_servo_cartesian(
            command,
            speed=1.0,
            mvacc=50.0,
        )

    def _compute_ee_command(self) -> np.ndarray:
        """
        Compute next end-effector command based on current and target poses.
        Returns a 6-element array: [x, y, z, roll, pitch, yaw].
        """
        target = np.array(self.ee_cmd, dtype=float)
        _, actual = self.arm.get_position(is_radian=False)
        current = np.array(actual, dtype=float)

        # Position interpolation: move up to 1 unit per step
        delta_pos = target[:3] - current[:3]
        distance = np.linalg.norm(delta_pos)
        if distance < 0.01:
            cmd_xyz = current[:3]
        else:
            cmd_xyz = current[:3] + (delta_pos / distance)

        # Normalize angular difference to [-180, 180]
        delta_ang = (target[3:] - current[3:] + 180.0) % 360.0 - 180.0
        cmd_rpy = current[3:] + delta_ang
        cmd_rpy = np.clip(cmd_rpy, -360.0, 360.0)

        return np.concatenate((cmd_xyz, cmd_rpy))

    def _anti_lock_callback(self) -> None:
        """
        Periodically re-enable motion and reset state to avoid controller lock.
        """
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(1)
        self.arm.set_state(0)
        self.arm.clean_gripper_error()
        self.arm.set_gripper_mode(0)
        self.arm.set_gripper_enable(True)

    def destroy_node(self) -> None:
        """Disconnect the arm and clean up on shutdown."""
        if self.arm:
            self.arm.disconnect()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    controller = XArmCartesianController()
    try:
        rclpy.spin(controller)
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
