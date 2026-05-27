#!/usr/bin/env python3
"""
bringup.launch.py  —  Proyecto Final Grupo 5, IELE3338L 2026-10
=================================================================
Arranca el sistema completo:
  1. robot_state_publisher + EKF   (my_robot_description)
  2. Sensores: encoders, BNO055, YDLidar X4 Pro, Pi Camera
  3. Vision bridge WebSocket       (streaming Pi → PC externo)
  4. SLAM Toolbox online-async     (mapeo con LiDAR)
  5. Nav2 navigation stack         (planeación + control)
  6. Motor control con PID         (cinemática + lazo de velocidad)
  7. Brazo manipulador (ESP32)     (arm_bridge)
  8. Eje Z stepper (Arduino)       (stepper_node)

Uso:
  ros2 launch proyecto_final_grupo5 bringup.launch.py
  ros2 launch proyecto_final_grupo5 bringup.launch.py lidar_port:=/dev/ttyUSB1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Directorios de paquetes ───────────────────────────────────────────────
    desc_dir         = get_package_share_directory('my_robot_description')
    nav2_config_dir  = get_package_share_directory('nav2_config')
    sensors_dir      = get_package_share_directory('sensors')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(nav2_config_dir, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(nav2_config_dir, 'config', 'slam_params.yaml')

    # ── Argumentos ───────────────────────────────────────────────────────────
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/lidar',
        description='Puerto serial del YDLidar X4 Pro'
    )

    # ── 1. URDF + EKF ────────────────────────────────────────────────────────
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_dir, 'launch', 'description.launch.py')
        )
    )

    # ── 2. Sensores base (encoders + IMU + LiDAR) ────────────────────────────
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_dir, 'launch', 'sensors.py')
        ),
        launch_arguments={
            'lidar_port': LaunchConfiguration('lidar_port'),
        }.items()
    )

    # ── 3. Cámara Pi ─────────────────────────────────────────────────────────
    camera_node = Node(
        package='sensors',
        executable='camera_node',
        name='camera_node',
        output='screen',
    )

    # ── 4. Vision bridge (streaming JPEG hacia PC externo via WebSocket) ─────
    vision_bridge_node = Node(
        package='sensors',
        executable='vision_bridge_node',
        name='vision_bridge_node',
        output='screen',
        parameters=[{
            'port': 8765,
            'jpeg_quality': 60,
            'stream_fps': 15.0,
        }],
    )

    # ── 5. SLAM Toolbox ───────────────────────────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false',
        }.items()
    )

    # ── 6. Nav2 ───────────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'false',
        }.items()
    )

    # ── 7. Control de motores con PID ─────────────────────────────────────────
    # Suscribe /odometry/filtered y /ref, controla GPIO directamente.
    # NOTA: no lanzar motor_com (motor_command) simultáneamente — conflicto GPIO.
    motor_control_node = Node(
        package='locomotion',
        executable='motor_control',
        name='motor_control_node',
        output='screen',
    )

    # ── 8. Brazo manipulador (ESP32 via serial) ───────────────────────────────
    arm_bridge_node = Node(
        package='sensors',
        executable='arm_bridge_node',
        name='arm_bridge_node',
        output='screen',
        parameters=[{
            'ws_port': 8766,
            'serial_port': '/dev/ttyUSB0',
            'baudrate': 115200,
        }],
    )

    # ── 9. Eje Z — motor paso a paso (Arduino via serial) ────────────────────
    stepper_node = Node(
        package='sensors',
        executable='stepper_node',
        name='stepper_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/arduino',
            'baudrate': 115200,
        }],
    )

    # ── 10. Dashboard de trayectoria (Flask, accesible en :5000) ─────────────
    dashboard_node = Node(
        package='locomotion',
        executable='graphical_odom',
        name='trajectory_dashboard',
        output='screen',
    )

    return LaunchDescription([
        lidar_port_arg,
        description_launch,
        sensors_launch,
        camera_node,
        vision_bridge_node,
        slam_launch,
        nav2_launch,
        motor_control_node,
        arm_bridge_node,
        stepper_node,
        dashboard_node,
    ])
