#!/usr/bin/env python3
import csv
from time import time
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, RotaryEncoder
Device.pin_factory = LGPIOFactory(chip=0)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros


class SystemIdentNode(Node):
    def __init__(self):
        super().__init__('system_ident_node')

        self.cmd_subscriber = self.create_subscription(Twist, '/motor_cmd', self.cmd_callback, 10)
        self.odom_subscriber = self.create_subscription(Twist, '/omega', self.odom_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.latest_cmd_ = Twist()
        self.latest_speed = Twist()

        self.r_input_ = []
        self.r_output_ = []
        self.l_input_ = []
        self.l_output_ = []
        self.time_vec_ = []

        self.start_time_ = None
        self.done_ = False

        TIMER_PERIOD = 0.02  # 50 Hz
        self.timer_ = self.create_timer(TIMER_PERIOD, self.main_loop)
        self.get_logger().info('SystemIdentNode iniciado')

    def cmd_callback(self, msg: Twist):
        self.latest_cmd_ = msg

    def odom_callback(self, msg: Twist):
        self.latest_speed = msg

    def cmd_to_pwm(self):
        speed = self.latest_cmd_.linear.x
        turn  = self.latest_cmd_.angular.z

        turn_gain = 0.7
        left_cmd  = speed - turn_gain * turn
        right_cmd = speed + turn_gain * turn

        max_val = max(abs(left_cmd), abs(right_cmd), 1.0)
        left_cmd  /= max_val
        right_cmd /= max_val

        self.r_input_.append(right_cmd)
        self.l_input_.append(left_cmd)

    def clamp(self, value, vmin=-1.0, vmax=1.0):
        return max(vmin, min(vmax, value))

    def main_loop(self):
        if self.done_:
            return

        self.cmd_to_pwm()

        self.r_output_.append(self.latest_speed.linear.x)
        self.l_output_.append(self.latest_speed.angular.z)

        if self.start_time_ is None:
            self.start_time_ = time()
            elapsed = 0.0
        else:
            elapsed = time() - self.start_time_

        self.time_vec_.append(elapsed)

        self.get_logger().info(
            f'[{len(self.time_vec_)}/1000] t={elapsed:.2f}s '
            f'l_cmd={self.l_input_[-1]:.3f} r_cmd={self.r_input_[-1]:.3f} '
            f'l_out={self.l_output_[-1]:.3f} r_out={self.r_output_[-1]:.3f}',
            throttle_duration_sec=1.0
        )

        if len(self.r_input_) >= 1000:
            self._save_csv(filename=f"system_id_{int(time())}.csv")
            self.done_ = True

    def _save_csv(self, filename=None):
        filename = filename or f"system_id_{int(time())}.csv"
        fieldnames = ["timestamp", "left_cmd", "right_cmd", "left_out", "right_out"]

        with open(filename, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for i, t in enumerate(self.time_vec_):
                writer.writerow({
                    "timestamp":  t,
                    "left_cmd":   self.l_input_[i],
                    "right_cmd":  self.r_input_[i],
                    "left_out":   self.l_output_[i],
                    "right_out":  self.r_output_[i],
                })

        self.get_logger().info(f'CSV guardado: {filename}')


def main(args=None):
    rclpy.init(args=args)
    node = SystemIdentNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.done_:
                break
    except KeyboardInterrupt:
        node.get_logger().info('Interrumpido por usuario')
    finally:
        if not node.done_:  # Si salió antes de completar, guarda lo que haya
            node._save_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()