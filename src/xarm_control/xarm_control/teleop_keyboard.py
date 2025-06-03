#!/usr/bin/env python3
"""
Keyboard teleoperation for the xArm6 robotic arm.
Reads key inputs to generate end-effector pose and gripper commands,
subscribes to current pose feedback, and publishes Cartesian Twist and
GripperCommand messages.
"""

import sys, termios, tty, threading, select, fcntl, os

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from control_msgs.msg import GripperCommand
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


# Key mappings: key → (message field, index, direction multiplier)
key_bindings = {
    "w": ("linear", 0, 1.0),  # +X
    "s": ("linear", 0, -1.0),  # -X
    "a": ("linear", 1, 1.0),  # +Y
    "d": ("linear", 1, -1.0),  # -Y
    "e": ("linear", 2, 1.0),  # +Z
    "q": ("linear", 2, -1.0),  # -Z
    "i": ("angular", 0, 1.0),  # +Roll
    "k": ("angular", 0, -1.0),  # -Roll
    "j": ("angular", 1, 1.0),  # +Pitch
    "l": ("angular", 1, -1.0),  # -Pitch
    "u": ("angular", 2, 1.0),  # +Yaw
    "o": ("angular", 2, -1.0),  # -Yaw
}

# Scaling factors and gripper limits
LINEAR_SCALE = 5
ANGULAR_SCALE = 1
GRIPPER_SPEED = 800.0
GRIPPER_STEP = 10.0
GRIPPER_MIN = -10.0
GRIPPER_MAX = 850.0


class TeleopKeyboard(Node):
    """ROS2 node for keyboard teleoperation of xArm6 in Cartesian space."""

    def __init__(self):
        super().__init__("teleop_keyboard")

        # Publishers for end-effector and gripper
        self.pub = self.create_publisher(Twist, "/xarm6/ee_pose_cmd", 10)

        self.gripper_pub = self.create_publisher(
            GripperCommand, "/xarm6/gripper_cmd", 10
        )

        # Storage for the most recent end-effector pose
        self.curr_linear = None
        self.curr_angular = None
        # Initialize gripper position
        self.gripper = 500.

        # Subscribe to current end-effector pose topic
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Twist,
            "/xarm6/ee_pose_current",
            self._current_pose_cb,
            qos,
        )

        # Wait until the first current pose message has been received
        self.get_logger().info("Waiting for current pose...")
        while rclpy.ok() and self.curr_linear is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(
            f"Current pose acquired: {self.curr_linear}, {self.curr_angular}"
        )

        # Configure terminal for non-blocking raw input
        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        self.old_flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.old_flags | os.O_NONBLOCK)

    @property
    def gripper(self):
        return self._gripper

    @gripper.setter
    def gripper(self, value):
        # Clamp gripper position within its limits
        self._gripper = np.clip(value, GRIPPER_MIN, GRIPPER_MAX)

    def _current_pose_cb(self, msg: Twist):
        """Callback for receiving current end-effector pose from the robot."""
        self.curr_linear = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.curr_angular = [msg.angular.x, msg.angular.y, msg.angular.z]

    def keyboard_loop(self):
        """Main loop: read keyboard inputs and publish corresponding commands."""
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                for _ in rlist:
                    c = sys.stdin.read(1)
                    # Exit on Ctrl-C or ESC
                    if c == "\x03" or c == "\x1b":
                        return

                    # Cartesian motion keys
                    if c in key_bindings:
                        t, idx, d = key_bindings[c]
                        msg = Twist()
                        msg.linear.x, msg.linear.y, msg.linear.z = self.curr_linear
                        msg.angular.x, msg.angular.y, msg.angular.z = self.curr_angular
                        if t == "linear":
                            setattr(
                                msg.linear,
                                ["x", "y", "z"][idx],
                                getattr(msg.linear, ["x", "y", "z"][idx])
                                + d * LINEAR_SCALE,
                            )
                        else:
                            setattr(
                                msg.angular,
                                ["x", "y", "z"][idx],
                                getattr(msg.angular, ["x", "y", "z"][idx])
                                + d * ANGULAR_SCALE,
                            )
                        self.pub.publish(msg)

                    # Gripper close (Enter key)
                    if c == "\r" or c == "\n":
                        self.gripper -= GRIPPER_STEP
                        grip_msg = GripperCommand()
                        grip_msg.position = self.gripper
                        grip_msg.max_effort = GRIPPER_SPEED
                        self.gripper_pub.publish(grip_msg)

                    # Gripper open (Backspace key)
                    if c == "\x7f":
                        self.gripper += GRIPPER_STEP
                        grip_msg = GripperCommand()
                        grip_msg.position = self.gripper
                        grip_msg.max_effort = GRIPPER_SPEED
                        self.gripper_pub.publish(grip_msg)
        finally:
            # Restore terminal settings on exit
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)
            fcntl.fcntl(self.fd, fcntl.F_SETFL, self.old_flags)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        node.keyboard_loop()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
