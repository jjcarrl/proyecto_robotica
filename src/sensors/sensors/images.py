import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagesNode(Node):
    def __init__(self):
        super().__init__('images_node')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg: Image):
        self.get_logger().info(f"Received image with timestamp: {msg.header.stamp}")
        ocvimg = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Video", ocvimg)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ImagesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()