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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
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

    # ── 2. rf2o lidar odometry (/scan → /odom_rf2o) ───────────────────────────
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic':     '/scan',
            'odom_topic':           '/odom_rf2o',
            'publish_tf':           False,
            'base_frame_id':        'base_link',
            'odom_frame_id':        'odom',
            'freq':                 10.0,
            'init_pose_from_topic': '',
        }],
    )

    # ── 3. EKF ────────────────────────────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
    )

    # ── 4. SLAM Toolbox (delay 5s para que EKF publique TF primero) ───────────
    slam_launch = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': 'false',
            }.items(),
        )]
    )

    # ── 5. Nav2 — solo nodos necesarios (delay 8s) ───────────────────────────
    nav2_nodes = [
        Node(package='nav2_controller',        executable='controller_server',   name='controller_server',   output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_smoother',          executable='smoother_server',     name='smoother_server',     output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_planner',           executable='planner_server',      name='planner_server',      output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_behaviors',         executable='behavior_server',     name='behavior_server',     output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_bt_navigator',      executable='bt_navigator',        name='bt_navigator',        output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',   name='waypoint_follower',   output='screen', parameters=[nav2_params_file]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',   name='velocity_smoother',   output='screen', parameters=[nav2_params_file]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart':    True,
                'node_names':   [
                    'controller_server', 'smoother_server', 'planner_server',
                    'behavior_server',   'bt_navigator',    'waypoint_follower',
                    'velocity_smoother',
                ],
            }],
        ),
    ]
    nav2_launch = TimerAction(period=8.0, actions=nav2_nodes)

    # ── 6. RC control (joy → /cmd_vel) ────────────────────────────────────────
    rc_control_node = Node(
        package='locomotion',
        executable='rc_control',
        name='rc_control_node',
        output='screen',
    )

    # ── 7. Joy ────────────────────────────────────────────────────────────────
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'device_id': 0}],
    )

    nodes = [
        robot_state_publisher,
        joint_state_publisher,
        ekf_node,
        slam_launch,
        nav2_launch,
        rc_control_node,
        joy_node,
    ]
    if rf2o_node is not None:
        nodes.insert(2, rf2o_node)

    return LaunchDescription(nodes)
