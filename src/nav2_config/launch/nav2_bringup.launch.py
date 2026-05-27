#!/usr/bin/env python3
"""
nav2_bringup.launch.py
Arranca toda la pila de navegación:
  1. robot_state_publisher  (URDF → TF estático)
  2. robot_localization EKF (odom + imu → /odometry/filtered, TF odom→base_link)
  3. Sensores: encoders, MPU-6050, YDLidar
  4. SLAM Toolbox online-async  (map → odom TF)
  5. Nav2 navigation stack      (planner + controller + behaviours)
  6. motor_command              (suscribe /cmd_vel → PWM GPIO)

Uso:
  ros2 launch nav2_config nav2_bringup.launch.py
  ros2 launch nav2_config nav2_bringup.launch.py lidar_port:=/dev/ttyUSB1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Paths ─────────────────────────────────────────────────────────────────
    nav2_config_dir   = get_package_share_directory('nav2_config')
    desc_dir          = get_package_share_directory('my_robot_description')
    sensors_dir       = get_package_share_directory('sensors')
    slam_toolbox_dir  = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir  = get_package_share_directory('nav2_bringup')

    nav2_params_file  = os.path.join(nav2_config_dir, 'config', 'nav2_params.yaml')
    slam_params_file  = os.path.join(nav2_config_dir, 'config', 'slam_params.yaml')

    # ── Arguments ─────────────────────────────────────────────────────────────
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port del YDLidar X4 Pro'
    )

    # ── 1. Robot description + EKF ────────────────────────────────────────────
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_dir, 'launch', 'description.launch.py')
        )
    )

    # ── 2. Sensores ───────────────────────────────────────────────────────────
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_dir, 'launch', 'sensors.py')
        ),
        launch_arguments={
            'lidar_port': LaunchConfiguration('lidar_port'),
        }.items()
    )

    # ── 3. SLAM Toolbox (online async) ────────────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false',
        }.items()
    )

    # ── 4. Nav2 navigation stack ──────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'false',
        }.items()
    )

    # ── 5. Motor command ──────────────────────────────────────────────────────
    # Nav2 publica /cmd_vel; lo remapeamos a /motor_cmd que espera motor_command.
    motor_command_node = Node(
        package='locomotion',
        executable='motor_com',
        name='motor_command_node',
        output='screen',
        remappings=[
            ('/motor_cmd', '/cmd_vel'),
        ],
    )

    return LaunchDescription([
        lidar_port_arg,
        description_launch,
        sensors_launch,
        slam_launch,
        nav2_launch,
        motor_command_node,
    ])
