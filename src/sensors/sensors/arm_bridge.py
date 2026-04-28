import asyncio
import csv
import json
import os
import threading
import time
from datetime import datetime
from pathlib import Path

import serial
import rclpy
import websockets
from rclpy.node import Node
from std_msgs.msg import String


class ArmBridgeNode(Node):
    """
    Puente entre el script DualSense (Windows) y la ESP32 via Serial USB.

    Modos (topic /arm/mode):
      live    — reenvía comandos de Windows a la ESP32 en tiempo real
      record  — igual que live pero también guarda cada frame en CSV
      replay  — ignora Windows; reproduce el CSV más reciente con su timing original
      stop    — envía posición home y detiene todo
    """

    MODES = {"live", "record", "replay", "stop"}
    HOME_CMD = "90,90,90,90\n"

    def __init__(self):
        super().__init__('arm_bridge_node')

        self.declare_parameter('ws_port',        8766)
        self.declare_parameter('serial_port',    '/dev/ttyUSB0')
        self.declare_parameter('baudrate',       115200)
        self.declare_parameter('recordings_dir', str(Path.home() / 'arm_recordings'))

        ws_port       = self.get_parameter('ws_port').value
        serial_port   = self.get_parameter('serial_port').value
        baudrate      = self.get_parameter('baudrate').value
        recordings_dir = self.get_parameter('recordings_dir').value

        self.recordings_dir = Path(recordings_dir)
        self.recordings_dir.mkdir(parents=True, exist_ok=True)

        self.mode         = 'live'
        self.recording    = []
        self.record_start = None
        self.ws_clients   = set()
        self.loop         = None

        # Serial hacia ESP32
        try:
            self.ser = serial.Serial(serial_port, baudrate, timeout=1)
            self.get_logger().info(f'Serial abierto: {serial_port} @ {baudrate}')
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f'No se pudo abrir serial ({serial_port}): {e}')

        # Topics ROS2
        self.sub_mode    = self.create_subscription(String, '/arm/mode',       self._mode_cb,       10)
        self.pub_status  = self.create_publisher(  String, '/arm/status',      10)
        self.pub_rec_list = self.create_publisher( String, '/arm/recordings',  10)

        self.create_timer(5.0, self._publish_recordings_list)

        # WebSocket server en hilo propio
        threading.Thread(target=self._run_ws_server, args=(ws_port,), daemon=True).start()

        self.get_logger().info(f'Arm bridge listo — ws://0.0.0.0:{ws_port}')
        self._publish_status()

    # ----------------------------------------------------------------- modos

    def _mode_cb(self, msg: String):
        new_mode = msg.data.strip().lower()
        if new_mode not in self.MODES:
            self.get_logger().warn(f'Modo desconocido: "{new_mode}". Opciones: {self.MODES}')
            return

        # Cerrar modo actual
        if self.mode == 'record' and new_mode != 'record':
            self._save_recording()

        # Abrir nuevo modo
        if new_mode == 'record':
            self.recording    = []
            self.record_start = time.time()
            self.get_logger().info('Grabacion iniciada')

        elif new_mode == 'replay':
            self._start_replay()
            return  # _start_replay ya cambia self.mode

        elif new_mode == 'stop':
            self._send_to_esp32(self.HOME_CMD)

        self.mode = new_mode
        self.get_logger().info(f'Modo: {new_mode}')
        self._publish_status()

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({'mode': self.mode, 'frames': len(self.recording)})
        self.pub_status.publish(msg)

    def _publish_recordings_list(self):
        files = sorted(self.recordings_dir.glob('arm_*.csv'))
        msg = String()
        msg.data = json.dumps([f.name for f in files])
        self.pub_rec_list.publish(msg)

    # -------------------------------------------------------------- grabacion

    def _save_recording(self):
        if not self.recording:
            self.get_logger().warn('Grabacion vacía, nada guardado')
            return
        ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = self.recordings_dir / f'arm_{ts}.csv'
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'd23', 'd15', 'd13', 'd25'])
            for row in self.recording:
                writer.writerow(row)
        self.get_logger().info(
            f'Grabacion guardada: {path.name}  ({len(self.recording)} frames)'
        )

    # -------------------------------------------------------------- replay

    def _start_replay(self):
        files = sorted(self.recordings_dir.glob('arm_*.csv'))
        if not files:
            self.get_logger().warn('No hay grabaciones disponibles')
            return
        latest = files[-1]
        self.mode = 'replay'
        self._publish_status()
        self.get_logger().info(f'Reproduciendo: {latest.name}')
        threading.Thread(target=self._replay_worker, args=(latest,), daemon=True).start()

    def _replay_worker(self, path: Path):
        rows = []
        with open(path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append({
                    't':   float(row['t']),
                    'cmd': f"{row['d23']},{row['d15']},{row['d13']},{row['d25']}\n"
                })

        t0 = time.time()
        for row in rows:
            if self.mode != 'replay':
                break
            wait = row['t'] - (time.time() - t0)
            if wait > 0:
                time.sleep(wait)
            self._send_to_esp32(row['cmd'])

        if self.mode == 'replay':
            self.mode = 'live'
            self._publish_status()
            self.get_logger().info('Reproduccion terminada → modo live')

    # ---------------------------------------------------------------- serial

    def _send_to_esp32(self, cmd: str):
        if not self.ser or not self.ser.is_open:
            return
        try:
            data = cmd if cmd.endswith('\n') else cmd + '\n'
            self.ser.write(data.encode())
        except Exception as e:
            self.get_logger().error(f'Error serial: {e}')

    # ------------------------------------------------------------- WebSocket

    async def _handler(self, websocket):
        self.ws_clients.add(websocket)
        self.get_logger().info(f'Windows conectado: {websocket.remote_address}')
        try:
            async for raw in websocket:
                cmd = raw if isinstance(raw, str) else raw.decode()
                cmd = cmd.strip()

                if self.mode == 'live':
                    self._send_to_esp32(cmd)

                elif self.mode == 'record':
                    self._send_to_esp32(cmd)
                    t    = round(time.time() - self.record_start, 4)
                    vals = cmd.split(',')
                    if len(vals) == 4:
                        self.recording.append([t] + [int(v) for v in vals])

                # replay/stop: ignorar comandos de Windows

        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.ws_clients.discard(websocket)
            self.get_logger().info('Windows desconectado')

    def _run_ws_server(self, port: int):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        async def _serve():
            async with websockets.serve(self._handler, '0.0.0.0', port):
                await asyncio.Future()

        self.loop.run_until_complete(_serve())

    # --------------------------------------------------------------- cleanup

    def destroy_node(self):
        if self.mode == 'record':
            self._save_recording()
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ArmBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
