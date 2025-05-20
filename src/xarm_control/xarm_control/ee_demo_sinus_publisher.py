#!/usr/bin/env python3
"""
Publishes a sinusoidal Cartesian pose command for the xArm end-effector.
Sends Twist messages on the '/xarm6/ee_pose_cmd' topic at 100 Hz.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class EESinusPublisher(Node):
    """ROS2 node that publishes a sinusoidal trajectory for the end-effector."""

    def __init__(self) -> None:
        super().__init__('ee_demo_sinus_publisher')

        # Create publisher for end-effector pose commands
        self.publisher = self.create_publisher(
            Twist,
            '/xarm6/ee_pose_cmd',
            10
        )

        # Record start time for trajectory computation
        self._start_time = time.time()

        # Timer to publish at 100 Hz
        self.create_timer(0.01, self._publish_sinus)

        self.get_logger().info(
            'Sinusoidal end-effector publisher initialized.'
        )

    def _publish_sinus(self) -> None:
        """
        Compute sinusoidal offsets and publish a Twist message.
        Linear X oscillates around 500 mm, Y oscillates around 0.
        Z and orientation remain fixed.
        """
        elapsed = time.time() - self._start_time
        
        # Create and populate Twist message
        msg = Twist()
        msg.linear.x = 500.0 + 200.0 * math.cos(2 * math.pi * elapsed / 10.0)
        msg.linear.y = 200.0 * math.sin(2 * math.pi * elapsed / 10.0)
        msg.linear.z = 200.0
        msg.angular.x = -179.95
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # Publish and log command
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published EE command: '
            f'[{msg.linear.x:.2f}, {msg.linear.y:.2f}, '
            f'{msg.linear.z:.2f}, {msg.angular.x:.2f}, '
            f'{msg.angular.y:.2f}, {msg.angular.z:.2f}]'
        )


def main(args=None) -> None:
    """Initialize ROS2 node and spin until shutdown."""
    rclpy.init(args=args)
    node = EESinusPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
