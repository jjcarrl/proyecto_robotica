import lgpio

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

BTN_X  = 0   # X (Cruz) — toggle servo
POS_0  = 0
POS_90 = 90

# Ancho de pulso en microsegundos.
# 1000 μs → 0°,  2000 μs → 90°.
# Si el servo no llega a los extremos, ajusta estos valores.
PULSO_MIN_US = 1000
PULSO_MAX_US = 2000


def angulo_a_us(angle: int) -> int:
    t = (angle - POS_0) / (POS_90 - POS_0)
    return int(PULSO_MIN_US + t * (PULSO_MAX_US - PULSO_MIN_US))


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        self.declare_parameter('gpio_pin',  26)
        self.declare_parameter('gpio_chip', 4)
        gpio_pin  = self.get_parameter('gpio_pin').value
        gpio_chip = self.get_parameter('gpio_chip').value

        self._pin      = gpio_pin
        self._angle    = POS_0
        self._prev_btn = False
        self._chip     = None

        try:
            self._chip = lgpio.gpiochip_open(gpio_chip)
            lgpio.tx_servo(self._chip, self._pin, angulo_a_us(POS_0))
            self.get_logger().info(
                f'GPIO{gpio_pin} (chip {gpio_chip}) listo — 0° ({angulo_a_us(POS_0)} μs)'
            )
        except Exception as e:
            self._chip = None
            self.get_logger().error(f'No se pudo inicializar GPIO{gpio_pin}: {e}')

        self.create_subscription(Joy,   '/joy',         self._joy_cb,   10)
        self.create_subscription(Int32, '/servo/angle', self._angle_cb, 10)
        self.pub = self.create_publisher(Int32, '/servo/state', 10)

        self.get_logger().info(f'ServoNode listo — GPIO{gpio_pin}  |  escucha: /servo/angle')

    def _mover(self, angle: int):
        angle = max(POS_0, min(POS_90, angle))
        us    = angulo_a_us(angle)

        if self._chip is not None:
            try:
                lgpio.tx_servo(self._chip, self._pin, us)
            except Exception as e:
                self.get_logger().error(f'Error al mover servo: {e}')
                return

        self._angle = angle
        out = Int32()
        out.data = angle
        self.pub.publish(out)
        self.get_logger().info(f'Servo → {angle}°  ({us} μs)')

    def _angle_cb(self, msg: Int32):
        self.get_logger().info(f'Comando recibido /servo/angle: {msg.data}°')
        self._mover(msg.data)

    def _joy_cb(self, msg: Joy):
        cur = len(msg.buttons) > BTN_X and bool(msg.buttons[BTN_X])
        if cur and not self._prev_btn:
            target = POS_0 if self._angle == POS_90 else POS_90
            self._mover(target)
        self._prev_btn = cur

    def destroy_node(self):
        if self._chip is not None:
            lgpio.tx_servo(self._chip, self._pin, 0)
            lgpio.gpiochip_close(self._chip)
        super().destroy_node()


def main():
    rclpy.init()
    node = ServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
