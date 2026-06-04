from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='terrain_mapping_drone_control',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='log',
            arguments=['--ros-args', '--log-level', 'warn'],
        ),

        Node(
            package='terrain_mapping_drone_control',
            executable='auto_detect_land',
            name='auto_detect_land',
            output='screen',
        ),
    ])