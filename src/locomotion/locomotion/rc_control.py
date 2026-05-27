#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# PS5 DualSense axes
AX_STEER  = 0   # joystick izquierdo horizontal
AX_RT     = 5   # gatillo derecho (avanzar)
AX_LT     = 2   # gatillo izquierdo (retroceder)


class rc_control_node(Node):
    def __init__(self):
        super().__init__("rc_control_node")
        self.create_subscription(Joy, "/joy", self.gamepad_callback, 10)
        self.motor_pub_ = self.create_publisher(Twist, "/motor_cmd", 10)

        self.latest_joy_ = None
        self.timer_ = self.create_timer(0.02, self.main_loop)
        self.get_logger().info('rc_control_node listo')

    def gamepad_callback(self, msg: Joy):
        self.latest_joy_ = msg

    def linear_remap(self, value, a=-1.0, b=1.0, c=0.0, d=1.0):
        return (d - c) * (value - a) / (b - a) + c

    def main_loop(self):
        if self.latest_joy_ is None or len(self.latest_joy_.axes) < 6:
            return

        joy = self.latest_joy_
        speed  = self.linear_remap(joy.axes[AX_RT], a=1, b=-1, c=0, d=5)
        speed -= self.linear_remap(joy.axes[AX_LT], a=1, b=-1, c=0, d=5)
        angle  = self.linear_remap(joy.axes[AX_STEER], a=-1, b=1, c=-3, d=3)

        msg = Twist()
        msg.linear.x  = speed
        msg.angular.z = angle
        self.motor_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = rc_control_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
