#!/usr/bin/env python3

import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor


# =========================
# Pines GPIO (BCM)
# =========================
PIN_FORWARD_L = 5   # IN1 5
PIN_BACKWARD_L = 6  # IN2 6

PIN_FORWARD_R = 17    # IN3 17
PIN_BACKWARD_R = 27   # IN4 27


class MotorCommand(Node):
    def __init__(self):
        super().__init__("motor_command")

        self.cmd_subscription_ = self.create_subscription(
            Twist,
            "/motor_vel",
            self.cmd_callback,
            10
        )

        self.latest_cmd_ = Twist()
        self.timer_ = self.create_timer(0.02, self.main_loop)  # 50 Hz

        self.motor_l = Motor(
            forward=PIN_FORWARD_L,
            backward=PIN_BACKWARD_L,
            pwm=True
        )

        self.motor_r = Motor(
            forward=PIN_FORWARD_R,
            backward=PIN_BACKWARD_R,
            pwm=True
        )

        self.get_logger().info("Nodo motor_command iniciado.")
        self.get_logger().info("Suscrito a /motor_vel")
        self.get_logger().info("Motor izquierdo: GPIO 5/6")
        self.get_logger().info("Motor derecho: GPIO 17/27")

    def cmd_callback(self, msg: Twist):
        self.latest_cmd_ = msg

    def clamp(self, value, vmin=-1.0, vmax=1.0):
        return max(vmin, min(vmax, value))

    def main_loop(self):
        speed = self.latest_cmd_.linear.x
        turn = self.latest_cmd_.angular.z

        speed = self.clamp(speed)
        turn = self.clamp(turn)

        # Igual que antes
        turn_gain = 0.7

        left_cmd = speed - turn_gain * turn
        right_cmd = speed + turn_gain * turn

        left_cmd = self.clamp(left_cmd)
        right_cmd = self.clamp(right_cmd)

        self.motor_l.value = left_cmd
        self.motor_r.value = right_cmd

    def destroy_node(self):
        self.motor_l.stop()
        self.motor_r.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommand()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.motor_l.stop()
        node.motor_r.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()