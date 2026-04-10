import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        
        self.cameraDeviceNumber=0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)

        self.bridgeObject = CvBridge() #Imagenes de opencv a topicos ros

        self.timer_period_ = 0.02
        self.timer_ = self.create_timer(self.timer_period_,self.publish_frame)

        self.img_counter_ = 0

    def publish_frame(self):

        success, frame = self.camera.read()
        frame = cv2.resize(frame, (820,640), interpolation=cv2.INTER_CUBIC)

        if success:
            ros2_img = self.bridgeObject.cv2_to_imgmsg(frame, encoding="bgr8")
            self.pub.publish(ros2_img)
            self.img_counter_ += 1

def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()