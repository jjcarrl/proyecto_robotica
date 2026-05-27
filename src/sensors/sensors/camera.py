import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

try:
    from picamera2 import Picamera2
    _HAS_PICAMERA = True
except ImportError:
    _HAS_PICAMERA = False


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('device_index', 0)
        device_index = self.get_parameter('device_index').value

        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        if _HAS_PICAMERA:
            self.cam = Picamera2()
            config = self.cam.create_video_configuration(
                main={"size": (820, 640), "format": "RGB888"}
            )
            self.cam.configure(config)
            self.cam.start()
            self._capture = self._capture_picamera
            self.get_logger().info('Usando Picamera2')
        else:
            self.cam = cv2.VideoCapture(device_index)
            self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 820)
            self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
            if not self.cam.isOpened():
                self.get_logger().error(f'No se pudo abrir camara {device_index}')
            self._capture = self._capture_opencv
            self.get_logger().info(f'Usando OpenCV — device {device_index}')

        self.timer_ = self.create_timer(0.02, self.publish_frame)

    def _capture_picamera(self):
        return self.cam.capture_array()

    def _capture_opencv(self):
        ret, frame = self.cam.read()
        return frame if ret else None

    def publish_frame(self):
        frame = self._capture()
        if frame is None:
            return
        ros2_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        ros2_img.header.stamp = self.get_clock().now().to_msg()
        ros2_img.header.frame_id = 'camera_link'
        self.pub.publish(ros2_img)

    def destroy_node(self):
        if _HAS_PICAMERA:
            self.cam.stop()
        else:
            self.cam.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
