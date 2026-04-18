from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='locomotion',
            executable='graphical_odom',
            name='trajectory_plotter',
            output='screen',
        ),
        Node(
            package='locomotion',
            executable='motor_control',
            name='motor_control_node',
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
            executable='mpu_node',
            name='mpu_node',
            output='screen',
        )
    ])
