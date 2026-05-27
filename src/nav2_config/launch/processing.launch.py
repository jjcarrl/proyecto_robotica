#!/usr/bin/env python3
"""
processing.launch.py  —  capa de procesamiento (corre en el PC)

Nodos:
  1. robot_state_publisher  (URDF → TF estático)
  2. joint_state_publisher
  3. EKF  (odom + imu → /odometry/filtered, TF odom→base_link)
  4. SLAM Toolbox online-async  (map → odom TF)
  5. Nav2 navigation stack
  6. rc_control_node  (joy → /cmd_vel)

Prerequisito: la Raspberry debe tener corriendo sensors.launch.py
  ros2 launch sensors sensors.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    nav2_config_dir  = get_package_share_directory('nav2_config')
    desc_dir         = get_package_share_directory('my_robot_description')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(nav2_config_dir, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(nav2_config_dir, 'config', 'slam_params.yaml')
    ekf_params_file  = os.path.join(desc_dir, 'config', 'ekf.yaml')

    urdf_path = os.path.join(desc_dir, 'urdf', 'robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
    )

    # ── 1. Robot description ──────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # ── 2. EKF ────────────────────────────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
    )

    # ── 3. SLAM Toolbox ───────────────────────────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false',
        }.items(),
    )

    # ── 4. Nav2 ───────────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'false',
        }.items(),
    )

    # ── 5. RC control (joy → /cmd_vel) ────────────────────────────────────────
    rc_control_node = Node(
        package='locomotion',
        executable='rc_control',
        name='rc_control_node',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        ekf_node,
        slam_launch,
        nav2_launch,
        rc_control_node,
    ])
