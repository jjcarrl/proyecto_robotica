import asyncio
import json
import threading

import cv2
import rclpy
import websockets
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class VisionBridgeNode(Node):
    """
    Puente entre el topic /camera/image_raw y un cliente Windows via WebSocket.
    Comprime frames a JPEG y los transmite; recibe resultados JSON de vuelta
    y los publica en /vision/detections.
    """

    def __init__(self):
        super().__init__('vision_bridge_node')

        self.declare_parameter('port', 8765)
        self.declare_parameter('jpeg_quality', 60)
        self.declare_parameter('stream_fps', 15.0)

        port = self.get_parameter('port').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        stream_fps = self.get_parameter('stream_fps').value

        self.bridge = CvBridge()
        self.ws_clients: set = set()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.loop: asyncio.AbstractEventLoop | None = None

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 10
        )
        self.pub = self.create_publisher(String, '/vision/detections', 10)

        self.create_timer(1.0 / stream_fps, self._stream_tick)

        ws_thread = threading.Thread(
            target=self._run_ws_server, args=(port,), daemon=True
        )
        ws_thread.start()

        self.get_logger().info(
            f'Vision bridge listo — ws://0.0.0.0:{port}  '
            f'(JPEG q={self.jpeg_quality}, {stream_fps:.0f} fps)'
        )

    # ------------------------------------------------------------------ ROS2

    def _image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.frame_lock:
            self.latest_frame = frame

    def _stream_tick(self):
        if self.loop is None or not self.ws_clients:
            return
        with self.frame_lock:
            frame = self.latest_frame
        if frame is None:
            return
        ok, buf = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            return
        data = buf.tobytes()
        asyncio.run_coroutine_threadsafe(self._broadcast(data), self.loop)

    # --------------------------------------------------------------- WebSocket

    async def _broadcast(self, data: bytes):
        dead = set()
        for ws in list(self.ws_clients):
            try:
                await ws.send(data)
            except Exception:
                dead.add(ws)
        self.ws_clients -= dead

    async def _handler(self, websocket):
        self.ws_clients.add(websocket)
        addr = websocket.remote_address
        self.get_logger().info(f'Cliente conectado: {addr}')
        try:
            async for raw in websocket:
                try:
                    result = json.loads(raw)
                    msg = String()
                    msg.data = json.dumps(result)
                    self.pub.publish(msg)
                    self.get_logger().info(f'Deteccion recibida: {result}')
                except (json.JSONDecodeError, Exception) as e:
                    self.get_logger().warn(f'Mensaje invalido: {e}')
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.ws_clients.discard(websocket)
            self.get_logger().info(f'Cliente desconectado: {addr}')

    def _run_ws_server(self, port: int):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        async def _serve():
            async with websockets.serve(self._handler, '0.0.0.0', port):
                await asyncio.Future()  # corre indefinidamente

        self.loop.run_until_complete(_serve())


def main():
    rclpy.init()
    node = VisionBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
