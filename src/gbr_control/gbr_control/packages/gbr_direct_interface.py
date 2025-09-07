#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3, Quaternion
import math

class GBRDirectInterface:
    
    def __init__(self, node: Node):
        self._node = node
        self._publisher = node.create_publisher(
            Float64MultiArray,
            '/gbr/thrusters',
            10
        )

        self._position = Point()
        self._orientation = Quaternion()
        self._linear_velocity = Vector3()
        self._angular_velocity = Vector3()

        self._odom_subscriber = node.create_subscription(
            Odometry,
            '/gbr/odom',
            self._odom_callback,
            10
        )
        
    def _odom_callback(self, msg: Odometry):
        self._position = msg.pose.pose.position
        self._orientation = msg.pose.pose.orientation
        self._linear_velocity = msg.twist.twist.linear
        self._angular_velocity = msg.twist.twist.angular
    
    def get_pose(self):
        position = (self._position.x, self._position.y, self._position.z)
        q = self._orientation  # Assuming q has attributes w, x, y, z

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        pitch = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation) with full -pi to pi range
        sinp = 2 * (q.w * q.y - q.z * q.x)
        cosp = 1 - 2 * (q.y * q.y + q.x * q.x)
        roll = math.atan2(sinp, cosp)  # <- this allows full -π to π

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        angle = (-roll, pitch, yaw)

        return position, angle
    
    def get_velocity(self):
        lin_vel = (self._linear_velocity.x, 
                  -self._linear_velocity.y, 
                  self._linear_velocity.z)
        
        ang_vel = (-self._angular_velocity.y,
                  self._angular_velocity.x,
                  self._angular_velocity.z)
        return lin_vel, ang_vel
    
    def print_state(self):
        position, orientation = self.get_pose()
        lin_vel, ang_vel = self.get_velocity()
        self._node.get_logger().info(
            f'\nPosition (x,y,z) [m]: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})\n'
            f'Orientation (r,p,y) [rad]: ({orientation[0]:.2f}, {orientation[1]:.2f}, {orientation[2]:.2f})\n'
            f'Linear velocity (x,y,z) [m/s]: ({lin_vel[0]:.2f}, {lin_vel[1]:.2f}, {lin_vel[2]:.2f})\n'
            f'Angular velocity (r,p,y) [rad/s]: ({ang_vel[0]:.2f}, {ang_vel[1]:.2f}, {ang_vel[2]:.2f})'
        )
    
    def set_thrusters(self, values):
        if len(values) != 8:
            self._node.get_logger().error('Must provide 8 thruster values')
            return
        msg = Float64MultiArray()
        msg.data = values
        self._publisher.publish(msg)
    
    def stop(self):
        self.set_thrusters([0.0] * 8)
