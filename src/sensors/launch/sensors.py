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
        ),
        Node(
            package='sensors',
            executable='servo_node',
            name='servo_node',
            output='screen',
            parameters=[{'gpio_pin': 18}],
        ),
        Node(
            package='sensors',
            executable='stepper_node',
            name='stepper_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/arduino',
                'baudrate': 115200,
            }],
        ),
    ])