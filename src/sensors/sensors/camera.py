import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from picamera2 import Picamera2


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)

        self.bridge = CvBridge()

        self.cam = Picamera2()
        config = self.cam.create_video_configuration(
            main={"size": (820, 640), "format": "RGB888"}
        )
        self.cam.configure(config)
        self.cam.start()

        self.timer_period_ = 0.02  # 50 Hz
        self.timer_ = self.create_timer(self.timer_period_, self.publish_frame)
        self.img_counter_ = 0

    def publish_frame(self):
        frame = self.cam.capture_array()
        ros2_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        ros2_img.header.stamp = self.get_clock().now().to_msg()
        ros2_img.header.frame_id = 'camera_link'
        self.pub.publish(ros2_img)
        self.img_counter_ += 1

    def destroy_node(self):
        self.cam.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
