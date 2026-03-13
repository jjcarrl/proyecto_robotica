#!/usr/bin/env python3

import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from gpiozero import Motor

class MotorCommand(Node):
    def __init__(self):
        super().__init__("motor_command")

        self.gamepad_capture_ = self.create_subscription(
            Twist,
            "/motor_vel",
            self.gamepad_callback,
            10
        )
        
        self.timer_ = self.create_timer(0.02, self.main_loop)  # 50 Hz
        
        

def main(args=None):
    rclpy.init(args=args)
    node = RCControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.motor_l.stop()
        node.motor_r.stop()
        node.destroy_node()
        rclpy.shutdown()