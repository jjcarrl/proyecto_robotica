from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import AngularServo

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

BTN_X  = 0   # X (Cruz) — toggle servo
POS_0  = 0
POS_90 = 90


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        self.declare_parameter('gpio_pin', 18)
        gpio_pin = self.get_parameter('gpio_pin').value

        factory = LGPIOFactory(chip=4)
        self.servo = AngularServo(
            gpio_pin,
            min_angle=POS_0,
            max_angle=POS_90,
            min_pulse_width=0.001,
            max_pulse_width=0.002,
            pin_factory=factory,
        )
        self._angle    = POS_0
        self._prev_btn = False
        self.servo.angle = POS_0

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.pub = self.create_publisher(Int32, '/servo/state', 10)

        self.get_logger().info(f'ServoNode listo — GPIO{gpio_pin}')

    def _joy_cb(self, msg: Joy):
        cur = len(msg.buttons) > BTN_X and bool(msg.buttons[BTN_X])
        if cur and not self._prev_btn:
            target = POS_0 if self._angle == POS_90 else POS_90
            self.servo.angle = target
            self._angle = target
            out = Int32(); out.data = target
            self.pub.publish(out)
            self.get_logger().info(f'Servo → {target}°')
        self._prev_btn = cur

    def destroy_node(self):
        self.servo.detach()
        super().destroy_node()


def main():
    rclpy.init()
    node = ServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
