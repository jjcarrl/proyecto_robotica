#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32
from rclpy.node import Node

from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import AngularServo

# PS5 DualSense button indices (ros2 joy)
BTN_X  = 0   # X (Cruz) → toggle servo
BTN_L1 = 4   # L1       → bajar motor de pasos (elevador)
BTN_R1 = 5   # R1       → subir motor de pasos (elevador)

STEPPER_STEP_MM = 5.0   # mm por pulsación de L1/R1
SERVO_GPIO_PIN  = 18

POS_0  = 0
POS_90 = 90


class rc_control_node(Node):
    def __init__(self):
        super().__init__("rc_control_node")
        self.gamepad_capture_ = self.create_subscription(
            Joy, "/joy", self.gamepad_callback, 10
        )
        self.motor_pub_   = self.create_publisher(Twist,   "/motor_cmd",   10)
        self.stepper_pub_ = self.create_publisher(Float32, "/z_axis/move", 10)
        self.servo_state_pub_ = self.create_publisher(Int32, "/servo/state", 10)

        factory = LGPIOFactory(chip=4)
        self.servo = AngularServo(
            SERVO_GPIO_PIN,
            min_angle=POS_0,
            max_angle=POS_90,
            min_pulse_width=0.001,
            max_pulse_width=0.002,
            pin_factory=factory,
        )
        self._servo_angle = POS_0
        self.servo.angle  = POS_0

        self.latest_joy_ = None
        self._prev_btns  = []

        self.timer_ = self.create_timer(0.02, self.main_loop)
        self.get_logger().info(f'rc_control_node listo — servo GPIO{SERVO_GPIO_PIN}')

    def gamepad_callback(self, msg: Joy):
        self.latest_joy_ = msg

    def linear_remap(self, value, a=-1.0, b=1.0, c=0.0, d=1.0):
        return (d - c) * (value - a) / (b - a) + c

    def _btn_pressed(self, idx: int, btns: list) -> bool:
        cur  = len(btns) > idx and bool(btns[idx])
        prev = len(self._prev_btns) > idx and bool(self._prev_btns[idx])
        return cur and not prev

    def _set_servo(self, angle: int):
        self.servo.angle = angle
        self._servo_angle = angle
        msg = Int32(); msg.data = angle
        self.servo_state_pub_.publish(msg)
        self.get_logger().info(f'Servo → {angle}°')

    def main_loop(self):
        if self.latest_joy_ is None or len(self.latest_joy_.axes) < 6:
            return

        btns = list(self.latest_joy_.buttons)

        # ── Motor de pasos (eje Z / elevador) ────────────────────────────────
        if self._btn_pressed(BTN_R1, btns):
            msg = Float32(); msg.data = +STEPPER_STEP_MM
            self.stepper_pub_.publish(msg)
            self.get_logger().info(f'Z +{STEPPER_STEP_MM} mm')

        if self._btn_pressed(BTN_L1, btns):
            msg = Float32(); msg.data = -STEPPER_STEP_MM
            self.stepper_pub_.publish(msg)
            self.get_logger().info(f'Z -{STEPPER_STEP_MM} mm')

        # ── Servo ─────────────────────────────────────────────────────────────
        if self._btn_pressed(BTN_X, btns):
            target = POS_0 if self._servo_angle == POS_90 else POS_90
            self._set_servo(target)

        self._prev_btns = btns

        # ── Movimiento ────────────────────────────────────────────────────────
        speed = 0.0
        speed += self.linear_remap(self.latest_joy_.axes[5], a=1, b=-1, c=0, d=5)
        speed -= self.linear_remap(self.latest_joy_.axes[2], a=1, b=-1, c=0, d=5)
        angle  = self.linear_remap(self.latest_joy_.axes[0], a=-1, b=1, c=-3, d=3)

        vel_msg = Twist()
        vel_msg.angular.z = angle
        vel_msg.linear.x  = speed
        self.motor_pub_.publish(vel_msg)

    def destroy_node(self):
        self.servo.detach()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = rc_control_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
