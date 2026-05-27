import threading
import time

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class StepperNode(Node):
    """
    Puente ROS2 ↔ Arduino Uno (motor paso a paso eje Z via AccelStepper).

    Suscripciones:
      /z_axis/move  (Float32) — movimiento relativo en mm (+/-)
      /z_axis/cmd   (String)  — comando raw: 's' parar, 'h' home, 'p' consultar

    Publicaciones:
      /z_axis/position (Float32) — posicion actual en mm (actualizada a 1 Hz)
    """

    def __init__(self):
        super().__init__('stepper_node')

        self.declare_parameter('serial_port', '/dev/arduino')
        self.declare_parameter('baudrate', 115200)

        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(serial_port, baudrate, timeout=1)
            time.sleep(2.0)  # esperar reset del Arduino al abrir serial
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Serial abierto: {serial_port} @ {baudrate}')
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f'No se pudo abrir serial ({serial_port}): {e}')

        self.sub_move = self.create_subscription(
            Float32, '/z_axis/move', self._move_cb, 10)
        self.sub_cmd = self.create_subscription(
            String, '/z_axis/cmd', self._cmd_cb, 10)

        self.pub_pos = self.create_publisher(Float32, '/z_axis/position', 10)

        self.create_timer(1.0, self._query_position)

        threading.Thread(target=self._serial_reader, daemon=True).start()

        self.get_logger().info('StepperNode listo')

    # ---------------------------------------------------------------- callbacks

    def _move_cb(self, msg: Float32):
        mm = msg.data
        cmd = f'+{mm:.4f}' if mm >= 0 else f'{mm:.4f}'
        self._send(cmd)
        self.get_logger().info(f'Moviendo: {mm:.4f} mm')

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip()
        self._send(cmd)
        self.get_logger().info(f'Comando: {cmd}')

    def _query_position(self):
        self._send('p')

    # ------------------------------------------------------------------ serial

    def _send(self, cmd: str):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn('Serial no disponible')
            return
        try:
            data = cmd if cmd.endswith('\n') else cmd + '\n'
            self.ser.write(data.encode())
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
                self.get_logger().debug(f'Arduino: {line}')
                # "Posicion actual: X.XX mm"
                if 'Posicion actual:' in line:
                    parts = line.split()
                    pos_mm = float(parts[2])
                    out = Float32()
                    out.data = pos_mm
                    self.pub_pos.publish(out)
            except Exception as e:
                self.get_logger().error(f'Error serial read: {e}')
                time.sleep(0.1)

    # --------------------------------------------------------------- cleanup

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
