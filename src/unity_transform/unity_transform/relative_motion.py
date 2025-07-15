from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R

import numpy as np
import time

from .utils import (
    convert_pose,
    sawtooth,
    twist_to_vector,
    vector_to_twist,
)


def rot_offset(u, r, p, y):
    return R.from_euler("XYZ", (r, -p, -y), degrees=True).apply(u)


class RelativeMotion:
    """Handle relative motion mode for Vive to xArm."""

    def __init__(self, node):
        self.node = node
        self.trigger = False
        self.xarm_origin_pose = None
        self.vive_origin_pose = None
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

    def xarm_cb(self, msg):
        self.xarm_current_pose = msg

    def right_sphere_cb(self, msg):
        self.right_sphere = twist_to_vector(msg)
        self.u = np.array([self.right_sphere[1]*100*5, 0, 130])

    def trigger_cb(self, msg: Bool):
        if msg.data and not self.trigger:
            if self.xarm_current_pose is None:
                self.node.get_logger().warning(
                    "xArm current pose not yet received; cannot set origin."
                )
                return
            self.trigger = True
            self.xarm_origin_pose = self.node.TwistType()#why store a twist msg ? an array would be better
            self.xarm_origin_pose.linear.x = self.xarm_current_pose.linear.x
            self.xarm_origin_pose.linear.y = self.xarm_current_pose.linear.y
            self.xarm_origin_pose.linear.z = self.xarm_current_pose.linear.z
            self.xarm_origin_pose.angular.x = self.xarm_current_pose.angular.x
            self.xarm_origin_pose.angular.y = self.xarm_current_pose.angular.y
            self.xarm_origin_pose.angular.z = self.xarm_current_pose.angular.z
            self.vive_origin_pose = None
            self.node.get_logger().info(
                "Trigger ON: captured xArm origin pose and waiting for controller origin."
            )
        elif not msg.data and self.trigger:
            self.trigger = False
            self.xarm_origin_pose = None
            self.vive_origin_pose = None
            self.node.get_logger().info("Trigger OFF: stopping relative motion.")

    def handle_pose(self, msg):
        converted = convert_pose( # refactor converted_pose, and should return an array
            self.node.R_matrix,
            msg,
            offsets=self.node.offsets,
        )
        if self.trigger:
            if self.vive_origin_pose is None:
                self.vive_origin_pose = self.node.TwistType()
                self.vive_origin_pose.linear.x = converted.linear.x
                self.vive_origin_pose.linear.y = converted.linear.y
                self.vive_origin_pose.linear.z = converted.linear.z
                self.vive_origin_pose.angular.x = converted.angular.x
                self.vive_origin_pose.angular.y = converted.angular.y
                self.vive_origin_pose.angular.z = converted.angular.z
                self.node.get_logger().info("Captured controller origin.")
                return
            
            # instead of storing a twist msg and convert each loop, just do it once and store vect
            vec_conv = twist_to_vector(converted) #refactor vect_ctrl
            vec_vive = twist_to_vector(self.vive_origin_pose) #refactor vect_origin
            vec_xarm = twist_to_vector(self.xarm_origin_pose) #refactor vect_unity

            delta_vec = vec_conv.copy()
            delta_vec[0:3] = vec_conv[0:3] - vec_vive[0:3]
            delta_vec[3] = 0.0
            delta_vec[4] = -sawtooth(vec_conv[4] - vec_vive[4])
            delta_vec[5] = sawtooth(vec_conv[5] - vec_vive[5])

            target_vec = vec_xarm.copy()
            for i in range(3, 6):
                target_vec[i] = sawtooth(vec_xarm[i] + delta_vec[i])

            target_vec[0:3] += rot_offset(self.u, *vec_xarm[3:]) - rot_offset(
                self.u, *target_vec[3:]) + delta_vec[0:3] / 1.1

            target = vector_to_twist(target_vec)

            self.node.pub.publish(target)
            self.node.get_logger().info(
                f"Published xArm6 pose: linear=({target.linear.x:.1f}, {target.linear.y:.1f}, {target.linear.z:.1f}), "
                f"angular=({target.angular.x:.1f}, {target.angular.y:.1f}, {target.angular.z:.1f})"
            )
