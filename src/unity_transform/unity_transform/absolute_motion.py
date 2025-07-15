from rclpy.node import Node
from geometry_msgs.msg import Twist

from .utils import convert_pose, vector_to_twist


class AbsoluteMotion:
    """Compute absolute pose commands from unity controller data."""

    def __init__(self, node: Node) -> None:
        self.node = node

    def handle_pose(self, msg: Twist) -> None:
        vector = convert_pose(
            self.node.R_matrix,
            msg,
            offsets=self.node.offsets,
        )
        new_msg = vector_to_twist(vector)
        self.node.pub.publish(new_msg)
        self.node.get_logger().info(
            f"Published xArm6 pose: linear=({new_msg.linear.x:.1f}, {new_msg.linear.y:.1f}, {new_msg.linear.z:.1f}), "
            f"angular=({new_msg.angular.x:.1f}, {new_msg.angular.y:.1f}, {new_msg.angular.z:.1f})"
        )
