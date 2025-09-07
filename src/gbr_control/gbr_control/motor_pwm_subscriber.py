#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Try to import the real navigator library
try:
    from bluerobotics_navigator import set_pwm_channel_duty_cycle
    REAL_NAVIGATOR = True
except ImportError:
    REAL_NAVIGATOR = False

    # Mock version of the function
    def set_pwm_channel_duty_cycle(channel, duty_cycle):
        print(f"[MOCK] Channel {channel} → Duty Cycle {duty_cycle:.2f}")

class ThrusterPWMMapper(Node):
    def __init__(self):
        super().__init__('thruster_pwm_mapper')

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/gbr/thrusters',
            self.listener_callback,
            10
        )

        if REAL_NAVIGATOR:
            self.get_logger().info("Using real BlueRobotics Navigator library.")
        else:
            self.get_logger().warn("Navigator library not found — using mock implementation.")

    def listener_callback(self, msg: Float64MultiArray):
        data = msg.data

        if len(data) != 8:
            self.get_logger().warn(f"Expected 8 values, got {len(data)}. Skipping.")
            return

        for channel, raw_value in enumerate(data):
            pwm = self.map_to_pwm(raw_value)
            try:
                set_pwm_channel_duty_cycle(channel, pwm)
                self.get_logger().info(f"Channel {channel}: Value {raw_value:.2f} → PWM {pwm:.2f}")
            except Exception as e:
                self.get_logger().error(f"Error setting PWM for channel {channel}: {e}")

    @staticmethod
    def map_to_pwm(value):
        """
        Maps value from [-20.0, 20.0] to [0.0, 1.0]
        """
        clamped = max(-20.0, min(20.0, value))
        return (clamped + 20.0) / 40.0


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterPWMMapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

