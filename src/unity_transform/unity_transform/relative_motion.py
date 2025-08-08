from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

import numpy as np

from .utils import (
    convert_pose,
    sawtooth,
    twist_to_vector,
    vector_to_twist,
)


def rot_offset(u: np.ndarray, r: float, p: float, y: float) -> np.ndarray:
    """Apply Euler rotation offsets to a vector."""

    return R.from_euler("XYZ", (r, -p, -y), degrees=True).apply(u)


class RelativeMotion:
    """Handle relative motion mode for unity to xArm."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.trigger = False
        self.xarm_origin_pose = None
        self.unity_origin_pose = None
        self.xarm_current_pose = None
        self.right_sphere = None

        self.trig_sub = node.create_subscription(
            Bool, "/unity/trigger", self.trigger_cb, 10
        )
        self.xarm_sub = node.create_subscription(
            Twist, "/xarm6/ee_pose_current", self.xarm_cb, 10
        )

        self.rs_sub = node.create_subscription(
            Twist, "/unity/right_sphere", self.right_sphere_cb, 10
        )

        self.u = np.array([0, 0, 130])

    def xarm_cb(self, msg: Twist) -> None:
        """Store the latest end-effector pose from the robot."""

        self.xarm_current_pose = twist_to_vector(msg)

    def right_sphere_cb(self, msg: Twist) -> None:
        """Update the reference vector from the right sphere marker."""

        self.right_sphere = twist_to_vector(msg)
        self.u = np.array([self.right_sphere[1] * 100 * 5, 0, 130])

    def trigger_cb(self, msg: Bool) -> None:
        """Handle trigger press to start or stop relative motion."""

        if msg.data and not self.trigger:
            if self.xarm_current_pose is None:
                self.node.get_logger().warning(
                    "xArm current pose not yet received; cannot set origin."
                )
                return
            self.trigger = True
            self.xarm_origin_pose = self.xarm_current_pose.copy()
            self.unity_origin_pose = None
            self.node.get_logger().info(
                "Trigger ON: captured xArm origin pose and waiting for controller origin."
            )
        elif not msg.data and self.trigger:
            self.trigger = False
            self.xarm_origin_pose = None
            self.unity_origin_pose = None
            self.u = np.array([0, 0, 130])
            self.node.get_logger().info("Trigger OFF: stopping relative motion.")

    def handle_pose(self, msg: Twist) -> None:
        """Compute and publish pose commands while the trigger is held."""

        controller_pose = convert_pose(
            self.node.R_matrix,
            msg,
            offsets=self.node.offsets,
        )
        if self.trigger:
            if self.unity_origin_pose is None:
                self.unity_origin_pose = controller_pose.copy()
                self.node.get_logger().info("Captured controller origin.")
                return

            delta_vec = controller_pose.copy()
            delta_vec[0:3] = controller_pose[0:3] - self.unity_origin_pose[0:3]
            delta_vec[3] = 0.0
            delta_vec[4] = -sawtooth(controller_pose[4] - self.unity_origin_pose[4])
            delta_vec[5] = sawtooth(controller_pose[5] - self.unity_origin_pose[5])

            target_vec = self.xarm_origin_pose.copy()
            for i in range(3, 6):
                target_vec[i] = sawtooth(self.xarm_origin_pose[i] + delta_vec[i])

            target_vec[0:3] += rot_offset(self.u, *self.xarm_origin_pose[3:]) - rot_offset(
                self.u, *target_vec[3:]) + delta_vec[0:3] / 1.1

            target = vector_to_twist(target_vec)

            self.node.pub.publish(target)
            self.node.get_logger().info(
                f"Published xArm6 pose: linear=({target.linear.x:.1f}, {target.linear.y:.1f}, {target.linear.z:.1f}), "
                f"angular=({target.angular.x:.1f}, {target.angular.y:.1f}, {target.angular.z:.1f})"
            )
