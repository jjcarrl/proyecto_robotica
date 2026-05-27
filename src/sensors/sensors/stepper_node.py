import threading
import time

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String

BTN_L1  = 4     # L1 → bajar eje Z
BTN_R1  = 5     # R1 → subir eje Z
STEP_MM = 5.0


class StepperNode(Node):
    def __init__(self):
        super().__init__('stepper_node')

        self.declare_parameter('serial_port', '/dev/arduino')
        self.declare_parameter('baudrate', 115200)

        serial_port = self.get_parameter('serial_port').value
        baudrate    = self.get_parameter('baudrate').value

        self.ser = None
        try:
            self.ser = serial.Serial(serial_port, baudrate, timeout=1)
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Serial abierto: {serial_port} @ {baudrate}')
        except Exception as e:
            self.ser = None
            self.get_logger().error(
                f'No se pudo abrir serial ({serial_port}): {e}\n'
                f'  → Verifica con:  ls -la /dev/arduino  o  ls /dev/ttyUSB*'
            )

        self._prev_btns = []

        self.create_subscription(Joy,     '/joy',         self._joy_cb,  10)
        self.create_subscription(Float32, '/z_axis/move', self._move_cb, 10)
        # /z_axis/cmd acepta comandos raw en el formato exacto del firmware Arduino
        self.create_subscription(String,  '/z_axis/cmd',  self._cmd_cb,  10)
        self.pub_pos = self.create_publisher(Float32, '/z_axis/position', 10)

        # Consulta de posición cada 5 s para no saturar el serial
        self.create_timer(5.0, self._query_position)
        threading.Thread(target=self._serial_reader, daemon=True).start()

        self.get_logger().info(
            f'StepperNode listo\n'
            f'  Serial: {"OK" if self.ser else "NO DISPONIBLE"}\n'
            f'  Botones joystick: L1=bajar  R1=subir  (paso {STEP_MM} mm)\n'
            f'  Formato de comando enviado: "+5.0000" / "-5.0000" / "p"\n'
            f'  Para probar: ros2 topic pub --once /z_axis/cmd std_msgs/msg/String "{{data: \'+5.0000\'}}"'
        )

    # ------------------------------------------------------------------ helpers

    def _btn_pressed(self, idx: int, btns: list) -> bool:
        cur  = len(btns) > idx and bool(btns[idx])
        prev = len(self._prev_btns) > idx and bool(self._prev_btns[idx])
        return cur and not prev

    # ------------------------------------------------------------------ callbacks

    def _joy_cb(self, msg: Joy):
        btns = list(msg.buttons)
        if self._btn_pressed(BTN_R1, btns):
            self._send_mm(+STEP_MM)
        if self._btn_pressed(BTN_L1, btns):
            self._send_mm(-STEP_MM)
        self._prev_btns = btns

    def _move_cb(self, msg: Float32):
        self._send_mm(msg.data)

    def _cmd_cb(self, msg: String):
        self.get_logger().info(f'Comando raw recibido: "{msg.data.strip()}"')
        self._send(msg.data.strip())

    # ------------------------------------------------------------------ serial TX

    def _send_mm(self, mm: float):
        cmd = f'+{mm:.4f}' if mm >= 0 else f'{mm:.4f}'
        self.get_logger().info(f'Movimiento Z {mm:+.1f} mm → enviando: "{cmd}"')
        self._send(cmd)

    def _query_position(self):
        self._send('p')

    def _send(self, cmd: str):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn('Serial no disponible — el motor NO recibe el comando')
            return
        try:
            raw = (cmd if cmd.endswith('\n') else cmd + '\n').encode()
            self.ser.write(raw)
            self.get_logger().debug(f'→ Arduino (bytes): {raw}')
        except Exception as e:
            self.get_logger().error(f'Error serial write: {e}')

    # ------------------------------------------------------------------ serial RX

    def _serial_reader(self):
        while rclpy.ok():
            if not self.ser or not self.ser.is_open:
                time.sleep(0.1)
                continue
            try:
                line = self.ser.readline().decode(errors='replace').strip()
                if not line:
                    continue

                # Muestra TODO lo que responde el Arduino para ayudar a calibrar el protocolo
                self.get_logger().info(f'← Arduino: "{line}"')

                if 'Posicion actual:' in line:
                    try:
                        pos_mm = float(line.split()[2])
                        out = Float32()
                        out.data = pos_mm
                        self.pub_pos.publish(out)
                    except (IndexError, ValueError):
                        self.get_logger().warn(f'No se pudo parsear posición: "{line}"')

            except Exception as e:
                self.get_logger().error(f'Error serial read: {e}')
                time.sleep(0.1)

    # ------------------------------------------------------------------ cleanup

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
