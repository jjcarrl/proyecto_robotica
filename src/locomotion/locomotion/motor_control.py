#!/usr/bin/env python3

import math
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor
from simple_pid import PID


# =========================
# Pines GPIO (BCM)
# =========================
PIN_FORWARD_L = 5   # IN1 5
PIN_BACKWARD_L = 6  # IN2 6

PIN_FORWARD_R = 17    # IN3 17
PIN_BACKWARD_R = 27   # IN4 27


class MotorControlNode(Node):
    def __init__(self):
        super().__init__("motor_control_node")
        self.odom_sub_ = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.ref_sub_ = self.create_subscription(
            Odometry, "/ref", self.ref_callback, 10
        )
        self.enc_sub_ = self.create_subscription(
            Twist, "/omega", self.enc_callback, 10
        )

        self.motor_l = Motor(
            forward=PIN_FORWARD_L,
            backward=PIN_BACKWARD_L,
            pwm=True
        )

        self.motor_r = Motor(
            forward=PIN_FORWARD_R,
            backward=PIN_BACKWARD_R,
            pwm=True
        )
        # Pose y referencia
        self.curr_pose_ = Odometry()
        self.curr_ref_ = Odometry()
        self.enc_speed_ = Twist()
        
        # Referencia inicial
        self.curr_ref_.pose.pose.position.x = 2.0
        self.curr_ref_.pose.pose.position.y = 0.0

        # Parámetros del robot
        self.r = 0.08 # Radio de las ruedas (m)
        self.L = 0.2  # Distancia entre ruedas (m)

        # Ganancias de control
        self.Ks = 0.3
        self.Kth = 2.4
        self.P = 0.774983013313723
        self.I = 28.0089367298732
        self.D = 0.00203814996847987

        # Setup PID
        self.pid_R = PID(self.P, self.I, self.D, setpoint=0.0)
        self.pid_L = PID(self.P, self.I, self.D, setpoint=0.0)

        self.pid_R.output_limits = (-1.0, 1.0)
        self.pid_L.output_limits = (-1.0, 1.0)

        self.pid_R.sample_time = 0.02
        self.pid_L.sample_time = 0.02

        # Log de inicio
        self.get_logger().info("Nodo motor_command iniciado.")
        self.get_logger().info("Suscrito a /motor_vel")
        self.get_logger().info("Motor izquierdo: GPIO 5/6")
        self.get_logger().info("Motor derecho: GPIO 17/27")


        # Timer
        self.create_timer(0.02, self.main_loop)  # 50 Hz

    def odom_callback(self, msg: Odometry):
        self.curr_pose_ = msg
    
    def ref_callback(self, msg: Odometry):
        self.curr_ref_ = msg
        

    def enc_callback(self, msg: Twist):
        self.enc_speed_ = msg

    def main_loop(self):
        # Aquí podrías implementar lógica de control basada en la odometría
        

        # Control de velocidad
        qz = self.curr_pose_.pose.pose.orientation.z
        qw = self.curr_pose_.pose.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)
        x = self.curr_pose_.pose.pose.position.x
        y = self.curr_pose_.pose.pose.position.y
        x_ref = self.curr_ref_.pose.pose.position.x
        y_ref = self.curr_ref_.pose.pose.position.y

        #Error de posicion
        dx = x_ref - x
        dy = y_ref - y

        # Angulo hacia el objetivo y errores
        phi = math.atan2(dy, dx)
        es = math.sqrt(dx**2 + dy**2)*math.cos(phi-theta)
        eth = phi - theta

        # Normalizar eth a [-pi, pi]
        eth = math.atan2(math.sin(eth), math.cos(eth))

        # Control proporcional de velocidad y giro
        v = self.Ks*es
        w = self.Kth*eth

        # Control de motores
        w_L = (v-w*self.L)/self.r
        w_R = (v+w*self.L)/self.r

        # Control PID para cada rueda
        
        self.pid_R.setpoint = w_R
        self.pid_L.setpoint = w_L

        enc_r = self.enc_speed_.linear.x
        enc_l = self.enc_speed_.angular.z

        control_r = self.pid_R(enc_r)
        control_l = self.pid_L(enc_l)

        if es < 0.05:
            es=0.0
            eth=0.0
            control_r = 0.0
            control_l = 0.0

        self.motor_l.value = control_l
        self.motor_r.value = control_r

    def destroy_node(self):
        self.motor_l.stop()
        self.motor_r.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.motor_l.stop()
        node.motor_r.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
