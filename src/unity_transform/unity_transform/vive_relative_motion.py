#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from control_msgs.msg import GripperCommand
import numpy as np


class ViveToXarm(Node):
    def __init__(self):
        super().__init__("vive_relative_motion")
        self.get_logger().info("Starting Vive RELATIVE motion node")

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

        # Trigger state: when True, start relative following
        self.trigger = False

        # Store the pose of the xArm at the moment the trigger switched on
        self.xarm_origin_pose = None
        # Store the pose of the Vive controller at the moment the trigger switched on
        self.vive_origin_pose = None

        # Latest xArm end-effector pose (updated by xarm_cb)
        self.xarm_current_pose = None

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

        # Subscribe to the Unity controller pose
        self.sub = self.create_subscription(
            Twist, "/unity/controller_pose", self.cb, 10
        )

        # Subscribe to the touchpad input for gripper control
        self.grip_sub = self.create_subscription(
            Float32, "/unity/touchpad", self.touchpad_cb, 10
        )

        # Subscribe to the trigger boolean
        self.trig_sub = self.create_subscription(
            Bool, "/unity/trigger", self.trigger_cb, 10
        )

        # Subscribe to the xArm's current end-effector pose
        self.xarm_sub = self.create_subscription(
            Twist, "/xarm6/ee_pose_current", self.xarm_cb, 10
        )

        # Publisher for xArm pose commands
        self.pub = self.create_publisher(Twist, "/xarm6/ee_pose_cmd", 10)

        # Publisher for xArm gripper commands
        self.grip_pub = self.create_publisher(GripperCommand, "/xarm6/gripper_cmd", 10)

        time.sleep(1)

    @staticmethod
    def _twist_to_vector(t: Twist) -> np.ndarray:
        """
        Convert a Twist message into a 6-element NumPy vector:
        [lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]
        """
        return np.array([
            t.linear.x,
            t.linear.y,
            t.linear.z,
            t.angular.x,
            t.angular.y,
            t.angular.z,
        ], dtype=float)

    @staticmethod
    def _vector_to_twist(vec: np.ndarray) -> Twist:
        """
        Convert a 6-element NumPy vector back into a Twist message.
        """
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = vec[0], vec[1], vec[2]
        t.angular.x, t.angular.y, t.angular.z = vec[3], vec[4], vec[5]
        return t

    @staticmethod
    def _sawtooth(angle: float) -> float:
        """
        Wrap an angle (degrees) into [-180, 180].
        """
        return (angle + 180.0) % 360.0 - 180.0

    def xarm_cb(self, msg: Twist):
        """
        Callback to update the latest xArm end-effector pose.
        This runs continuously, storing the current pose in self.xarm_current_pose.
        """
        self.xarm_current_pose = msg

    def trigger_cb(self, msg: Bool):
        """
        Callback for the Vive trigger button.
        When trigger transitions from False to True:
          - Record the current xArm pose as xarm_origin_pose.
          - Reset vive_origin_pose to None so that the next controller callback
            establishes the reference pose.
          - Set self.trigger to True to begin relative motion.
        When trigger transitions from True to False:
          - Set self.trigger to False to stop publishing relative commands.
          - Clear origin poses.
        """
        if msg.data and not self.trigger:
            # Trigger pressed: begin relative motion
            if self.xarm_current_pose is None:
                self.get_logger().warning("xArm current pose not yet received; cannot set origin.")
                return

            self.trigger = True
            # Store the xArm pose at the moment trigger goes True
            self.xarm_origin_pose = Twist()
            self.xarm_origin_pose.linear.x = self.xarm_current_pose.linear.x
            self.xarm_origin_pose.linear.y = self.xarm_current_pose.linear.y
            self.xarm_origin_pose.linear.z = self.xarm_current_pose.linear.z
            self.xarm_origin_pose.angular.x = self.xarm_current_pose.angular.x
            self.xarm_origin_pose.angular.y = self.xarm_current_pose.angular.y
            self.xarm_origin_pose.angular.z = self.xarm_current_pose.angular.z

            # Reset vive origin so that next controller reading sets the reference
            self.vive_origin_pose = None
            self.get_logger().info("Trigger ON: captured xArm origin pose and waiting for controller origin.")
        elif not msg.data and self.trigger:
            # Trigger released: stop relative motion
            self.trigger = False
            self.xarm_origin_pose = None
            self.vive_origin_pose = None
            self.get_logger().info("Trigger OFF: stopping relative motion.")
            self.pub.publish(self.xarm_current_pose)

    def cb(self, msg: Twist):
        """
        Callback for the Unity controller pose.
        Transforms the Vive controller pose into xArm coordinates.
        If self.trigger is True, publishes a relative xArm command:
          xarm_target = xarm_origin + (current_converted - vive_origin)
        Otherwise, does nothing.
        """
        # Convert raw Unity controller pose into xArm coordinate frame:
        XYpose = self.R_matrix @ np.array(
            [
                -(msg.linear.z) * 1000.0,
                (msg.linear.x) * 1000.0,
            ]
        )
        converted = Twist()
        converted.linear.x = XYpose[0] + self.offset_x
        converted.linear.y = XYpose[1] + self.offset_y
        converted.linear.z = (msg.linear.y * 1000.0) + self.offset_z

        converted.angular.x = 180.0
        converted.angular.y = self._sawtooth(msg.angular.x + self.offset_pitch)
        converted.angular.z = self._sawtooth(-msg.angular.y + self.offset_yaw)
        if self.trigger:
            # First time: set vive_origin_pose to the first converted reading after trigger
            if self.vive_origin_pose is None:
                self.vive_origin_pose = Twist()
                self.vive_origin_pose.linear.x = converted.linear.x
                self.vive_origin_pose.linear.y = converted.linear.y
                self.vive_origin_pose.linear.z = converted.linear.z
                self.vive_origin_pose.angular.x = converted.angular.x
                self.vive_origin_pose.angular.y = converted.angular.y
                self.vive_origin_pose.angular.z = converted.angular.z

                self.get_logger().info("Captured controller origin.")
            else:
                # Convert all three poses into NumPy vectors:
                vec_conv = self._twist_to_vector(converted)
                vec_vive = self._twist_to_vector(self.vive_origin_pose)
                vec_xarm = self._twist_to_vector(self.xarm_origin_pose)

                # Linear deltas are simple subtraction:
                delta_vec = vec_conv.copy()
                delta_vec[0:3] = vec_conv[0:3] - vec_vive[0:3]

                # Angular deltas use sawtooth wrap-around:
                delta_vec[3] = 0.
                delta_vec[4] = - self._sawtooth(vec_conv[4] - vec_vive[4])
                delta_vec[5] = self._sawtooth(vec_conv[5] - vec_vive[5])

                # Apply delta to xArm origin:
                target_vec = vec_xarm.copy()
                target_vec[0:3] += delta_vec[0:3]

                # For angular, add delta and wrap:
                for i in range(3, 6):
                    target_vec[i] = self._sawtooth(vec_xarm[i] + delta_vec[i])

                # Convert back to Twist for publishing:
                target = self._vector_to_twist(target_vec)

                self.pub.publish(target)
                self.get_logger().info(
                    f"Published xArm6 pose: linear=("
                    f"{target.linear.x:.1f}, {target.linear.y:.1f}, {target.linear.z:.1f}), "
                    f"angular=({target.angular.x:.1f}, {target.angular.y:.1f}, {target.angular.z:.1f})"
                )

    def touchpad_cb(self, msg: Float32):
        """
        Callback for the Vive touchpad input.
        Adjusts the gripper command based on touchpad movement,
        independent of trigger state.
        """
        self.get_logger().debug(f"Touchpad input: {msg.data:.3f}")
        prev_cmd = self.grip_cmd
        self.grip_cmd = np.clip(self.grip_cmd + self.grip_kp * msg.data, -10, 850)
        gripper_msg = GripperCommand()
        gripper_msg.position = self.grip_cmd
        gripper_msg.max_effort = 800.0
        self.grip_pub.publish(gripper_msg)


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
