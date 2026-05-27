#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32
from rclpy.node import Node

# PS5 DualSense — ejes
AX_STEER = 0   # joystick izquierdo horizontal
AX_RT    = 5   # gatillo derecho (avanzar)
AX_LT    = 2   # gatillo izquierdo (retroceder)

# PS5 DualSense — botones
BTN_X  = 0    # Cruz  → toggle servo (0° ↔ 90°)
BTN_R1 = 5    # R1    → stepper sube +5 mm
BTN_L1 = 4    # L1    → stepper baja -5 mm

STEP_MM   = 5.0
SERVO_0   = 0
SERVO_90  = 90


class rc_control_node(Node):
    def __init__(self):
        super().__init__("rc_control_node")
        self.create_subscription(Joy, "/joy", self.gamepad_callback, 10)

        self.motor_pub_  = self.create_publisher(Twist,   "/motor_cmd",    10)
        self.stepper_pub = self.create_publisher(Float32, "/z_axis/move",  10)
        self.servo_pub   = self.create_publisher(Int32,   "/servo/angle",  10)

        self.latest_joy_  = None
        self._prev_btns   = []
        self._servo_angle = SERVO_0

        self.timer_ = self.create_timer(0.02, self.main_loop)
        self.get_logger().info(
            'rc_control_node listo\n'
            '  Motores : LT=retroceder  RT=avanzar  JoyIzq=girar\n'
            '  Stepper : R1=+5mm        L1=-5mm\n'
            '  Servo   : X=toggle 0°/90°'
        )

    def gamepad_callback(self, msg: Joy):
        btns = list(msg.buttons)

        if self._btn_pressed(BTN_R1, btns):
            self._pub_stepper(+STEP_MM)
        if self._btn_pressed(BTN_L1, btns):
            self._pub_stepper(-STEP_MM)
        if self._btn_pressed(BTN_X, btns):
            self._servo_angle = SERVO_0 if self._servo_angle == SERVO_90 else SERVO_90
            self._pub_servo(self._servo_angle)

        self._prev_btns = btns
        self.latest_joy_ = msg

    def _btn_pressed(self, idx: int, btns: list) -> bool:
        cur  = len(btns) > idx and bool(btns[idx])
        prev = len(self._prev_btns) > idx and bool(self._prev_btns[idx])
        return cur and not prev

    def _pub_stepper(self, mm: float):
        msg = Float32()
        msg.data = mm
        self.stepper_pub.publish(msg)
        self.get_logger().info(f'Stepper {mm:+.1f} mm')

    def _pub_servo(self, angle: int):
        msg = Int32()
        msg.data = angle
        self.servo_pub.publish(msg)
        self.get_logger().info(f'Servo → {angle}°')

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
