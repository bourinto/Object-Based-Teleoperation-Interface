import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import numpy as np
from scipy.spatial.transform import Rotation

def quaternion_to_euler_xyz(q):
    x, y, z, w = q
    m00 = 1 - 2*(y**2 + z**2)
    m01 = 2*(x*y - z*w)
    m02 = 2*(x*z + y*w)
    m10 = 2*(x*y + z*w)
    m11 = 1 - 2*(x**2 + z**2)
    m12 = 2*(y*z - x*w)
    m20 = 2*(x*z - y*w)
    m21 = 2*(y*z + x*w)
    m22 = 1 - 2*(x**2 + y**2)

    alpha  = np.arcsin(-m20)
    beta = np.arctan2(m10, m00)
    gamma = np.arctan2(m21, m22)

    return np.degrees([alpha, beta, gamma])

class ViveToXarm(Node):
    def __init__(self):
        super().__init__("unity_to_xarm")

        self.declare_parameter("offset_x", 200)
        self.declare_parameter("offset_y", -412)
        self.declare_parameter("offset_z", -611)
        self.declare_parameter("offset_roll", 0)
        self.declare_parameter("offset_pitch", 0)
        self.declare_parameter("offset_yaw", 0)

        self.offset_x = self.get_parameter("offset_x").value
        self.offset_y = self.get_parameter("offset_y").value
        self.offset_z = self.get_parameter("offset_z").value
        self.offset_roll = self.get_parameter("offset_roll").value
        self.offset_pitch = self.get_parameter("offset_pitch").value
        self.offset_yaw = self.get_parameter("offset_yaw").value

        theta = np.radians(self.offset_yaw)
        self.R2D_matrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

        self.sub = self.create_subscription(Pose, "/unity/controller_pose", self.cb, 10)
        self.pub = self.create_publisher(Twist, "/xarm6/ee_pose_cmd", 10)

    def cb(self, msg):
        new_msg = Twist()

        # Cartesian coords
        pos_xy = np.array([-msg.position.z * 1000.0, msg.position.x * 1000.0])
        pose = self.R2D_matrix @ pos_xy

        new_msg.linear.x = np.floor(pose[0] + self.offset_x)
        new_msg.linear.y = np.floor(pose[1] + self.offset_y)
        new_msg.linear.z = np.floor(msg.position.y * 1000.0 + self.offset_z)

        # Euler angle X Y Z coords
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        roll, pitch, yaw = quaternion_to_euler_xyz(q)

        new_msg.angular.x = np.floor(roll)
        new_msg.angular.y = np.floor(pitch)
        new_msg.angular.z = np.floor(yaw)

        self.pub.publish(new_msg)

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
