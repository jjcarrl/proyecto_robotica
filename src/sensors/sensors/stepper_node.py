import threading
import time

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String

BTN_L1 = 4   # L1 → bajar eje Z
BTN_R1 = 5   # R1 → subir eje Z
STEP_MM = 5.0


class StepperNode(Node):
    def __init__(self):
        super().__init__('stepper_node')

        self.declare_parameter('serial_port', '/dev/arduino')
        self.declare_parameter('baudrate', 115200)

        serial_port = self.get_parameter('serial_port').value
        baudrate    = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(serial_port, baudrate, timeout=1)
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Serial abierto: {serial_port} @ {baudrate}')
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f'No se pudo abrir serial ({serial_port}): {e}')

        self._prev_btns = []

        self.create_subscription(Joy,    '/joy',        self._joy_cb,  10)
        self.create_subscription(Float32,'/z_axis/move',self._move_cb, 10)
        self.create_subscription(String, '/z_axis/cmd', self._cmd_cb,  10)
        self.pub_pos = self.create_publisher(Float32, '/z_axis/position', 10)

        self.create_timer(1.0, self._query_position)
        threading.Thread(target=self._serial_reader, daemon=True).start()

        self.get_logger().info('StepperNode listo')

    def _btn_pressed(self, idx, btns):
        cur  = len(btns) > idx and bool(btns[idx])
        prev = len(self._prev_btns) > idx and bool(self._prev_btns[idx])
        return cur and not prev

    def _joy_cb(self, msg: Joy):
        btns = list(msg.buttons)
        if self._btn_pressed(BTN_R1, btns):
            self._send_mm(+STEP_MM)
            self.get_logger().info(f'Z +{STEP_MM} mm')
        if self._btn_pressed(BTN_L1, btns):
            self._send_mm(-STEP_MM)
            self.get_logger().info(f'Z -{STEP_MM} mm')
        self._prev_btns = btns

    def _move_cb(self, msg: Float32):
        self._send_mm(msg.data)

    def _cmd_cb(self, msg: String):
        self._send(msg.data.strip())

    def _send_mm(self, mm: float):
        cmd = f'+{mm:.4f}' if mm >= 0 else f'{mm:.4f}'
        self._send(cmd)

    def _query_position(self):
        self._send('p')

    def _send(self, cmd: str):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn('Serial no disponible')
            return
        try:
            self.ser.write((cmd if cmd.endswith('\n') else cmd + '\n').encode())
        except Exception as e:
            self.get_logger().error(f'Error serial write: {e}')

    def _serial_reader(self):
        while rclpy.ok():
            if not self.ser or not self.ser.is_open:
                time.sleep(0.1)
                continue
            try:
                line = self.ser.readline().decode(errors='replace').strip()
                if not line:
                    continue
                if 'Posicion actual:' in line:
                    pos_mm = float(line.split()[2])
                    out = Float32(); out.data = pos_mm
                    self.pub_pos.publish(out)
            except Exception as e:
                self.get_logger().error(f'Error serial read: {e}')
                time.sleep(0.1)

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = StepperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
