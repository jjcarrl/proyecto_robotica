#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.node import Node

class rc_control_node(Node):
    def __init__(self):
        super().__init__("rc_control_node")
        self.gamepad_capture_ = self.create_subscription(
            Joy, "/joy", self.gamepad_callback, 10
        )
        self.motor_pub_ = self.create_publisher(
            Twist, "/motor_cmd", 10
        )
        self.latest_joy_ = None  # ← None until first message arrives
        self.timer_ = self.create_timer(0.02, self.main_loop)

    def gamepad_callback(self, msg: Joy):
        self.latest_joy_ = msg

    def linear_remap(self, value, a=-1.0, b=1.0, c=0.0, d=1.0):
        return (d - c) * (value - a) / (b - a) + c  # ← corrected formula

    def main_loop(self):
        # Guard: skip until at least one Joy message has been received
        if self.latest_joy_ is None or len(self.latest_joy_.axes) < 6:
            return

        # Speed magnitude
        speed = 0.0
        forward = self.linear_remap(self.latest_joy_.axes[5], a=1, b=-1, c=0, d=5)
        speed += forward

        backward = self.linear_remap(self.latest_joy_.axes[2], a=1, b=-1, c=0, d=5)
        speed -= backward

        # Angle
        angle = self.linear_remap(self.latest_joy_.axes[0], a=-1, b=1, c=-3, d=3)

        vel_msg = Twist()
        vel_msg.angular.z = angle
        vel_msg.linear.x = speed
        self.motor_pub_.publish(vel_msg)

        #print(f"input: {list(self.latest_joy_.axes)} | speed: {speed:.2f} | angle: {angle:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = rc_control_node()
    rclpy.spin(node)
    rclpy.shutdown()