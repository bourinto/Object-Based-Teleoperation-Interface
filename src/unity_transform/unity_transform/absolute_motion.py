class AbsoluteMotion:
    """Compute absolute pose commands from Vive controller data."""
    def __init__(self, node):
        self.node = node

    def handle_pose(self, msg):
        new_msg = self.node.convert_pose(msg)
        self.node.pub.publish(new_msg)
        self.node.get_logger().info(
            f"Published xArm6 pose: linear=({new_msg.linear.x:.1f}, {new_msg.linear.y:.1f}, {new_msg.linear.z:.1f}), "
            f"angular=({new_msg.angular.x:.1f}, {new_msg.angular.y:.1f}, {new_msg.angular.z:.1f})"
        )
