from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar'),

        Node(
            package='sensors',
            executable='mpu_node',
            name='mpu_node',
            output='screen',
        ),
        Node(
            package='sensors',
            executable='encoders_node',
            name='encoders_node',
            output='screen',
        ),
        Node(
            package='sensors',
            executable='lidar_node',
            name='ydlidar_node',
            output='screen',
            parameters=[{
                'port':     LaunchConfiguration('lidar_port'),
                'frame_id': 'laser',
            }],
        ),
        Node(
            package='locomotion',
            executable='motor_com',
            name='motor_command_node',
            output='screen',
            remappings=[('/motor_cmd', '/cmd_vel')],
        ),
    ])