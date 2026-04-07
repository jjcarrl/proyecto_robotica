import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cam = Picamera2()
        self.cam.configure(self.cam.create_preview_configuration())
        self.cam.start()
        self.timer = self.create_timer(0.033, self.publish_frame)  # ~30 FPS

    def publish_frame(self):
        frame = self.cam.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()