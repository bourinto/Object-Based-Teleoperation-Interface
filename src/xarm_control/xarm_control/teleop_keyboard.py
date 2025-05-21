#!/usr/bin/env python3
import sys, termios, tty, threading, select, time, fcntl, os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from control_msgs.msg import GripperCommand

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

LINEAR_SCALE = 5
ANGULAR_SCALE = 1
GRIPPER_SPEED = 800.0  # r/min
GRIPPER_STEP = 10.0  # 10 mm per keypress
GRIPPER_MIN = -10.0  # fully closed
GRIPPER_MAX = 850.0  # 85 mm fully open


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.pub = self.create_publisher(Twist, "/xarm6/ee_pose_cmd", 10)
        self.gripper_pub = self.create_publisher(
            GripperCommand, "/xarm6/gripper_cmd", 10
        )

        self.initial_linear = [210.0, 0.0, 115.0]
        self.initial_angular = [-180.0, 0.0, 0.0]
        self.curr_linear = list(self.initial_linear)
        self.curr_angular = list(self.initial_angular)
        self.curr_gripper = 0.0
        self.gripper_step = GRIPPER_STEP

        self.exit_event = threading.Event()
        self.lock = threading.Lock()

        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        self.old_flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.old_flags | os.O_NONBLOCK)

        t = threading.Thread(target=self.keyboard_loop, daemon=True)
        t.start()

    def keyboard_loop(self):
        try:
            while not self.exit_event.is_set():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                for _ in rlist:
                    c = sys.stdin.read(1)
                    if c == "\x03":
                        self.exit_event.set()
                        return
                    if c == "\x1b":
                        msg = Twist()
                        msg.linear.x, msg.linear.y, msg.linear.z = self.initial_linear
                        msg.angular.x, msg.angular.y, msg.angular.z = (
                            self.initial_angular
                        )
                        self.pub.publish(msg)
                        self.exit_event.set()
                        return
                    if c in key_bindings:
                        t, idx, d = key_bindings[c]
                        with self.lock:
                            if t == "linear":
                                self.curr_linear[idx] += d * LINEAR_SCALE
                            else:
                                self.curr_angular[idx] += d * ANGULAR_SCALE
                    # Enter to close gripper (decrease position)
                    if c == "\r" or c == "\n":
                        with self.lock:
                            self.curr_gripper = max(
                                GRIPPER_MIN, self.curr_gripper - self.gripper_step
                            )
                    # Backspace to open gripper (increase position)
                    if c == "\x7f":
                        with self.lock:
                            self.curr_gripper = min(
                                GRIPPER_MAX, self.curr_gripper + self.gripper_step
                            )
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)
            fcntl.fcntl(self.fd, fcntl.F_SETFL, self.old_flags)

    def run(self):
        rate_hz = 20.0
        period = 1.0 / rate_hz
        while rclpy.ok() and not self.exit_event.is_set():
            msg = Twist()
            with self.lock:
                msg.linear.x, msg.linear.y, msg.linear.z = self.curr_linear
                msg.angular.x, msg.angular.y, msg.angular.z = self.curr_angular
            self.pub.publish(msg)

            grip_msg = GripperCommand()
            with self.lock:
                grip_msg.position = self.curr_gripper
            grip_msg.max_effort = GRIPPER_SPEED
            self.gripper_pub.publish(grip_msg)

            time.sleep(period)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(node.fd, termios.TCSADRAIN, node.old_term)
        fcntl.fcntl(node.fd, fcntl.F_SETFL, node.old_flags)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
