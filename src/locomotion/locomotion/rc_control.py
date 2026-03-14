#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


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
            "/motor_vel",
            10
        )

        self.latest_joy_ = Joy()
        self.timer_ = self.create_timer(0.02, self.main_loop)  # 50 Hz

        self.get_logger().info("Nodo rc_control_node iniciado.")
        self.get_logger().info("Publicando Twist en /motor_vel")

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
        vel_msg = Twist()

        if len(self.latest_joy_.axes) < 6:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.mov_publisher.publish(vel_msg)
            return

        # Gatillo derecho: adelante
        forward = self.latest_joy_.axes[5]
        forward = self.linear_remap(forward, a=1.0, b=-1.0, c=0.0, d=1.0)

        # Gatillo izquierdo: atrás
        backward = self.latest_joy_.axes[2]
        backward = self.linear_remap(backward, a=1.0, b=-1.0, c=0.0, d=1.0)

        # Stick derecho horizontal: giro
        turn = self.latest_joy_.axes[0]
        turn = self.apply_deadzone(turn, deadzone=0.05)

        # Velocidad lineal
        speed = forward - backward
        speed = self.apply_deadzone(speed, deadzone=0.05)

        vel_msg.linear.x = float(speed)
        vel_msg.angular.z = float(turn)
        self.mov_publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RCControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()