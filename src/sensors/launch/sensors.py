from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
])