#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import select
import termios
import tty
import math
import traceback

# ================================
# NON-BLOCKING KEYBOARD READER
# ================================
def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            return sys.stdin.read(3)
        return ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ================================
# ROS2 NODE
# ================================
class CraneKeyboard(Node):

    def __init__(self):
        super().__init__("crane_keyboard")

        # Topics and joint
        self.setpoint_topic = "/skandi_iceman/joint_setpoints"
        self.state_topic = "/skandi_iceman/joint_states"
        self.joint_name = "skandi_iceman/CraneYaw"

        # Publisher (command)
        self.pub = self.create_publisher(
            JointState, self.setpoint_topic, 10
        )

        # Subscriber (real feedback)
        self.sub = self.create_subscription(
            JointState, self.state_topic, self.state_callback, 10
        )

        # Internal state
        self.command_angle = 0.0
        self.real_angle = None
        self.step = 0.03
        self.max_angle = math.pi
        self.iteration = 0

        self.get_logger().info("===================================")
        self.get_logger().info(" Crane Keyboard Control (ROS2 Jazzy)")
        self.get_logger().info(f" Command topic : {self.setpoint_topic}")
        self.get_logger().info(f" State topic   : {self.state_topic}")
        self.get_logger().info(f" Joint name    : {self.joint_name}")
        self.get_logger().info(" Controls:")
        self.get_logger().info("   LEFT  ← : rotate left")
        self.get_logger().info("   RIGHT → : rotate right")
        self.get_logger().info("   Q         : quit")
        self.get_logger().info("===================================")

        self.timer = self.create_timer(0.02, self.loop)

    # ======================================
    # RECEIVE REAL JOINT STATE FROM SIM
    # ======================================
    def state_callback(self, msg: JointState):
        if self.joint_name in msg.name:
            idx = msg.name.index(self.joint_name)
            if len(msg.position) > idx:
                self.real_angle = msg.position[idx]

    # ======================================
    # MAIN LOOP
    # ======================================
    def loop(self):
        self.iteration += 1

        key = get_key()
        angle_changed = False

        if key == '\x1b[D':          # LEFT
            self.command_angle += self.step
            angle_changed = True

        elif key == '\x1b[C':        # RIGHT
            self.command_angle -= self.step
            angle_changed = True

        elif key and key.lower()[0] == 'q':
            self.get_logger().info("Shutting down.")
            rclpy.shutdown()
            sys.exit(0)

        # Clamp
        self.command_angle = max(
            -self.max_angle,
            min(self.max_angle, self.command_angle)
        )

        # Publish command
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [float(self.command_angle)]
        self.pub.publish(msg)

        # Print when command changes
        if angle_changed:
            self.get_logger().info(
                f"COMMAND = {self.command_angle:.3f} rad "
                f"({math.degrees(self.command_angle):.2f} deg)"
            )

        # Print REAL angle from simulator every 0.5 s
        if self.iteration % 25 == 0:
            if self.real_angle is not None:
                self.get_logger().info(
                    f"REAL ANGLE = {self.real_angle:.3f} rad "
                    f"({math.degrees(self.real_angle):.2f} deg)"
                )
            else:
                self.get_logger().warning(
                    "Waiting for joint_states feedback..."
                )


def main():
    try:
        rclpy.init()
        node = CraneKeyboard()
        rclpy.spin(node)
    except Exception as e:
        print("======== EXCEPTION ========")
        print(e)
        traceback.print_exc()
        print("===========================")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
