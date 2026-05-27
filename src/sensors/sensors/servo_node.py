from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import AngularServo

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

# Posiciones disponibles
POS_0  = 0
POS_90 = 90


class ServoNode(Node):
    """
    Controla un servo con dos posiciones fijas: 0° y 90°.

    Suscripciones:
      /servo/cmd (Bool) — False → 0°,  True → 90°

    Publicaciones:
      /servo/state (Int32) — posicion actual en grados (0 o 90)
    """

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

        self._angle = POS_0
        self.servo.angle = POS_0

        self.sub = self.create_subscription(Bool, '/servo/cmd', self._cmd_cb, 10)
        self.pub = self.create_publisher(Int32, '/servo/state', 10)

        self.create_timer(1.0, self._publish_state)

        self.get_logger().info(f'ServoNode listo — GPIO{gpio_pin}, posicion inicial: {POS_0}°')

    def _cmd_cb(self, msg: Bool):
        target = POS_90 if msg.data else POS_0
        if target == self._angle:
            return
        self.servo.angle = target
        self._angle = target
        self.get_logger().info(f'Servo → {target}°')
        self._publish_state()

    def _publish_state(self):
        out = Int32()
        out.data = self._angle
        self.pub.publish(out)

    def destroy_node(self):
        self.servo.detach()
        super().destroy_node()


def main():
    rclpy.init()
    node = ServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
