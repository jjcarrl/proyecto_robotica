#!/usr/bin/env python3

import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from gpiozero import Motor


# =========================
# Pines GPIO (BCM)
# =========================
PIN_FORWARD_L = 17   # IN1
PIN_BACKWARD_L = 27  # IN2

PIN_FORWARD_R = 5    # IN3
PIN_BACKWARD_R = 6   # IN4


class RCControlNode(Node):
    def __init__(self):
        super().__init__("rc_control_node")

        self.gamepad_capture_ = self.create_subscription(
            Joy,
            "/joy",
            self.gamepad_callback,
            10
        )

        self.mov_publisher = self.create_publisher(
            Twist,
            "/turtle1/cmd_vel",
            10
        )

        self.latest_joy_ = Joy()
        self.timer_ = self.create_timer(0.02, self.main_loop)  # 50 Hz

        # Motores
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

        self.get_logger().info("Nodo rc_control_node iniciado.")
        self.get_logger().info("Motor izquierdo: GPIO 17/27")
        self.get_logger().info("Motor derecho: GPIO 5/6")

    def gamepad_callback(self, msg: Joy):
        self.latest_joy_ = msg

    def clamp(self, value, vmin=-1.0, vmax=1.0):
        return max(vmin, min(vmax, value))

    def linear_remap(self, value, a=-1.0, b=1.0, c=0.0, d=1.0):
        return c + (value - a) * (d - c) / (b - a)

    def apply_deadzone(self, value, deadzone=0.05):
        if abs(value) < deadzone:
            return 0.0
        return value

    def main_loop(self):
        if len(self.latest_joy_.axes) < 6:
            self.motor_l.stop()
            self.motor_r.stop()
            return

        # Gatillo derecho: adelante
        forward = self.latest_joy_.axes[5]
        forward = self.linear_remap(forward, a=1.0, b=-1.0, c=0.0, d=1.0)

        # Gatillo izquierdo: atrás
        backward = self.latest_joy_.axes[2]
        backward = self.linear_remap(backward, a=1.0, b=-1.0, c=0.0, d=1.0)

        # Stick izquierdo horizontal: giro
        turn = self.latest_joy_.axes[0]
        turn = self.apply_deadzone(turn, deadzone=0.05)

        # Velocidad base
        speed = forward - backward
        speed = self.apply_deadzone(speed, deadzone=0.05)
        speed = self.clamp(speed)

        # Mezcla diferencial
        turn_gain = 0.7
        left_cmd = speed + turn_gain * turn
        right_cmd = speed - turn_gain * turn

        left_cmd = self.clamp(left_cmd)
        right_cmd = self.clamp(right_cmd)

        # Mandar a motores
        self.motor_l.value = left_cmd
        self.motor_r.value = right_cmd

        # Publicar Twist
        vel_msg = Twist()
        vel_msg.linear.x = float(speed)
        vel_msg.angular.z = float(turn)
        self.mov_publisher.publish(vel_msg)

    def destroy_node(self):
        self.motor_l.stop()
        self.motor_r.stop()
        super().destroy_node()


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


if __name__ == "__main__":
    main()