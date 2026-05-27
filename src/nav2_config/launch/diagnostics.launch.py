#!/usr/bin/env python3
"""
diagnostics.launch.py  —  launch mínimo para calibración de sensores

Corre solo: robot_state_publisher (TF) + rf2o (lidar odometry)
SIN EKF, SIN SLAM, SIN Nav2

Prerequisito: Pi corriendo sensors.launch.py (lidar + encoders)

Uso:
    ros2 launch nav2_config diagnostics.launch.py

Topics a monitorear:
    /odom_rf2o   → odometría de lidar (referencia)
    /odom        → odometría de encoders (a calibrar)
    /imu         → IMU (a verificar signo/escala)

Procedimiento:
  1. Estático 30s:
       ros2 topic echo /odom_rf2o --field pose.pose.position

  2. Avanzar 1.00 m exacto con cinta:
       ros2 topic echo /odom_rf2o --field pose.pose.position.x
       ros2 topic echo /odom      --field pose.pose.position.x
       → ratio_lineal = x_rf2o / x_enc
       → WHEEL_DIAM_M_correcto = 0.08 * ratio_lineal
         ó ENCODER_PPR_correcto = 562 / ratio_lineal

  3. Girar exactamente 360° en sitio (marcas en el suelo):
       ros2 topic echo /odom_rf2o --field pose.pose.orientation
       ros2 topic echo /odom      --field pose.pose.orientation
       → yaw_rf2o, yaw_enc: deben ser ~0 (o exactamente 2π ± error)
       → ratio_angular = yaw_rf2o / yaw_enc
       → WHEEL_BASE_correcto = 0.20 * ratio_angular

  4. Verificar signo del IMU:
       Girar el robot lentamente a la izquierda (CCW vista desde arriba).
       ros2 topic echo /imu --field angular_velocity.z
       → Debe ser POSITIVO (convenio ROS: CCW = +z)
       Si es negativo, hay que negar gyro[2] en mpu.py.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    desc_dir = get_package_share_directory('my_robot_description')
    urdf_path = os.path.join(desc_dir, 'urdf', 'robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

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

    return LaunchDescription([robot_state_publisher, rf2o_node])
