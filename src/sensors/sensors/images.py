import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
import cv2

class ImagesNode(Node):
    def __init__(self):
        super().__init__('images_node')
        self.sub = self.create_subscription(Image, '/camera/image_raw', 10, self.image_callback)
        self.timer = self.create_timer(0.02, self.publish_frame)  # ~50 FPS

    def image_callback(self, msg: Image):
        self.get_logger().info(f"Received image with timestamp: {msg.header.stamp}")

def main():
    rclpy.init()
    node = ImagesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()