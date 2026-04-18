import os
import csv
import time
import math
import threading
import socket
import json

from numpy import roll
from geometry_msgs import msg
import mpu6050

from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, RotaryEncoder
Device.pin_factory = LGPIOFactory(chip=0)

from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf2_ros



class MPUNode(Node):
    def __init__(self):
        super().__init__('mpu_node')
        self.get_logger().info("MPU Node has been started.")

        self.imu = mpu6050.mpu6050(0x68)

        self.wheel_radius = 0.08
        self.encoder_resolution = self.get_parameter('encoder_resolution').get_parameter_value().integer_value        
        # Initialize encoder
        self.encoder = RotaryEncoder(a=17, b=27, max_steps=self.encoder_resolution)
        
        # Initialize publishers and TF broadcaster
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer_period_ = 0.02  # 20 ms -> 50 Hz
        self.timer_ = self.create_timer(self.timer_period_, self.main_loop)


    def read_sensor_data(self):
        # Read the accelerometer values
        accelerometer_data = self.imu.get_accel_data()

        # Read the gyroscope values
        gyroscope_data = self.imu.get_gyro_data()

        # Read temp
        temperature = self.imu.get_temp()

        return accelerometer_data, gyroscope_data, temperature
    

    def main_loop(self):
        
        acc,gyro,temp = self.read_sensor_data()

        # Create and publish the odometry message

        # Depends on IMU orientation, check axes if moved
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'

        imu_msg.linear_acceleration.x = acc['x']  # robot forward
        imu_msg.linear_acceleration.y =  acc['y']  # robot left
        imu_msg.linear_acceleration.z = acc['z']  # robot up

        imu_msg.angular_velocity.x =  gyro['x']*math.pi/180   # robot roll
        imu_msg.angular_velocity.y = gyro['y']*math.pi/180   # robot pitch
        imu_msg.angular_velocity.z = gyro['z']*math.pi/180   # robot yaw rate ← key one


        gyro_noise = (0.005 * math.pi/180) ** 2 * 100  # ~7.6e-7
        imu_msg.angular_velocity_covariance = [
            gyro_noise, 0, 0,
            0, gyro_noise, 0,
            0, 0, gyro_noise]
        accel_noise = (400e-6 * 9.81) ** 2 * 100  # ~1.54e-4
        imu_msg.linear_acceleration_covariance = [
            accel_noise, 0, 0,
            0, accel_noise, 0,
            0, 0, accel_noise
        ]

        self.imu_pub.publish(imu_msg)
