#!/usr/bin/env python3
# Imports
import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.node import Node
class rc_control_node(Node):
      def __init__(self):
         super().__init__("rc_control_node")
         self.gamepad_capture_ = self.create_subscription(
            Joy,"/joy",self.gamepad_callback,10
        )
         self.mov_publisher = self.create_publisher(
             Twist,"/turtle1/cmd_vel",10
         )
         self.latest_joy_ = Joy()
         
         self.timer_ = self.create_timer(0.02,self.main_loop)
      def gamepad_callback(self, msg: Joy):
         self.latest_joy_ = msg
      
      def linear_remap(self,value,a=-1.0,b=1.0,c = 0.0, d=1.0):
         value = value
         value = (c-d)*(value-a)/(b-a) + c
         return value
      
      def main_loop(self):
         # Speed magnitude
         speed = 0
         forward = self.latest_joy_.axes[5]
         forward = self.linear_remap(forward,a=1,b=-1,c=0,d=5)
         speed += forward
         backward = self.latest_joy_.axes[2]
         backward = self.linear_remap(backward,a=1,b=-1,c=0,d=5)
         speed -= backward
         # Angle
         angle = 0
         angle = self.latest_joy_.axes[0]
         angle = self.linear_remap(angle,a=-1,b=1,c=-3,d=3)
         vel_msg = Twist()
         vel_msg.angular.z = angle
         vel_msg.linear.x = speed
         self.mov_publisher.publish(vel_msg)
         
def main(args=None):
    rclpy.init(args=args)
    node = rc_control_node()
    rclpy.spin(node)
    rclpy.shutdown()