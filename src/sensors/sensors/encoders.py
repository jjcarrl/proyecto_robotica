#!/usr/bin/env python3

import os
import csv
import time
import math
import threading
import socket
import json

from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, RotaryEncoder

from sensors.odometry import RecorderPlayer
Device.pin_factory = LGPIOFactory(chip=0)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf2_ros

# =========================
# Pines GPIO encoders (BCM)
# =========================
PIN_ENC_L_A = 20
PIN_ENC_L_B = 21
PIN_ENC_R_A = 23
PIN_ENC_R_B = 24

# =========================
# Parámetros del robot
# =========================
ENCODER_PPR  = 562
WHEEL_DIAM_M = 0.08
WHEEL_BASE   = 0.20
DIST_PER_TICK = (math.pi * WHEEL_DIAM_M) / ENCODER_PPR


class EncodersNode(Node):
    def __init__(self):
        super().__init__("encoder_sensor")

        # ── Encoders ─────────────────────────────────────────────
        self.enc_l = RotaryEncoder(PIN_ENC_L_A, PIN_ENC_L_B, max_steps=0)
        self.enc_r = RotaryEncoder(PIN_ENC_R_A, PIN_ENC_R_B, max_steps=0)

        # ── Odometría ────────────────────────────────────────────
        self.x_   = 0.0
        self.y_   = 0.0
        self.omega_r = 0.0
        self.omega_l = 0.0
        self.yaw_ = 0.0
        self.prev_ticks_l_ = 0
        self.prev_ticks_r_ = 0
        self.v_linear = 0.0
        self.omega    = 0.0

        self.left_wheel_angle_ = 0.0
        self.right_wheel_angle_ = 0.0

        # ── Publicadores ─────────────────────────────────────────
        self.speed_pub_ = self.create_publisher(Twist,    "/omega", 10)
        self.odom_pub_ = self.create_publisher(Odometry, "/odom",      10)
        self.joint_pub_ = self.create_publisher(JointState, '/joint_states', 10)

        # ── Socket UDP para dashboard web ───────────────────────
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ── TF broadcaster ───────────────────────────────────────
        self.tf_broadcaster_ = tf2_ros.TransformBroadcaster(self)

        # ── Timer principal 50 Hz ────────────────────────────────
        self.timer_period_ = 0.02  # 20 ms -> 50 Hz
        self.timer_ = self.create_timer(self.timer_period_, self.main_loop)

    # Loop principal: actualiza odometría y publica datos a ROS y dashboard web

    def main_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._update_odometry()
        self._publish_odometry()

    # Actualiza la odometría a partir de los encoders, usando un modelo de robot diferencial

    def _update_odometry(self):
        ticks_l = self.enc_l.steps
        ticks_r = self.enc_r.steps

        d_l = (ticks_l - self.prev_ticks_l_) * DIST_PER_TICK
        d_r = (ticks_r - self.prev_ticks_r_) * DIST_PER_TICK

        speed_r = d_r / self.timer_period_
        speed_l = d_l / self.timer_period_

        self.omega_r = speed_r / 2 * WHEEL_DIAM_M  # rad/s
        self.omega_l = speed_l / 2 * WHEEL_DIAM_M  # rad/s

        self.prev_ticks_l_ = ticks_l
        self.prev_ticks_r_ = ticks_r

        d_center = (d_l + d_r) / 2.0
        d_yaw    = (d_r - d_l) / WHEEL_BASE

        self.x_   += d_center * math.cos(self.yaw_ + d_yaw / 2.0)
        self.y_   += d_center * math.sin(self.yaw_ + d_yaw / 2.0)
        self.yaw_ += d_yaw

        self.left_wheel_angle_  += d_l / (WHEEL_DIAM_M / 2)  
        self.right_wheel_angle_ += d_r / (WHEEL_DIAM_M / 2)

        self.v_linear = d_center / self.timer_period_
        self.omega    = d_yaw    / self.timer_period_

    # Publica la odometria y velocidad a ROS y TF, y también a un dashboard web vía UDP    

    def _publish_odometry(self):
        now_stamp = self.get_clock().now().to_msg()

        # Odometría ROS
        odom = Odometry()
        odom.header.stamp    = now_stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.pose.pose.position.x = self.x_
        odom.pose.pose.position.y = self.y_
        odom.pose.pose.orientation.z = math.sin(self.yaw_ / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw_ / 2.0)
        odom.twist.twist.linear.x  = self.v_linear   # (d_l + d_r) / 2 / dt
        odom.twist.twist.angular.z = self.omega       # d_yaw / dt

        # Covariances
        odom.pose.covariance[0]  = 0.002
        odom.pose.covariance[7]  = 0.026
        odom.pose.covariance[35] = 0.05

        odom.twist.covariance[0]  = 0.002
        odom.twist.covariance[7]  = 0.026
        odom.twist.covariance[35] = 0.05


        self.odom_pub_.publish(odom)
        omega = Twist()
        omega.linear.x  = self.omega_r
        omega.angular.z = self.omega_l
        self.speed_pub_.publish(omega)

        # Publica estado de las ruedas como JointState
        js = JointState()
        js.header.stamp = now_stamp
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_angle_, self.right_wheel_angle_] 
        self.joint_pub_.publish(js)

        t = TransformStamped()
        t.header.stamp            = now_stamp
        t.header.frame_id         = "odom"
        t.child_frame_id          = "base_link"
        t.transform.translation.x = self.x_
        t.transform.translation.y = self.y_
        t.transform.rotation.z    = math.sin(self.yaw_ / 2.0)
        t.transform.rotation.w    = math.cos(self.yaw_ / 2.0)
        #self.tf_broadcaster_.sendTransform(t)

        self._push_to_dashboard(self.x_, self.y_, math.degrees(self.yaw_), (self.omega_r + self.omega_l)/2, now_stamp.sec + now_stamp.nanosec * 1e-9)   

    # Publica datos a dashboard (para visualización en web)

    def _push_to_dashboard(self, x, y, yaw_deg, vel, timestamp):
        payload = json.dumps({"x": x, "y": y, "yaw_deg": yaw_deg,
                              "vel": vel, "timestamp": timestamp}).encode()
        self._udp_sock.sendto(payload, ("127.0.0.1", 5001))

# ─────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = EncodersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()