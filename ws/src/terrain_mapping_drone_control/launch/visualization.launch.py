#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch RViz2 and the drone pose visualizer."""

    pkg_share = get_package_share_directory('terrain_mapping_drone_control')
    rviz_config = os.path.join(pkg_share, 'config', 'drone_viz.rviz')

    pose_visualizer = Node(
        package='terrain_mapping_drone_control',
        executable='pose_visualizer',
        name='pose_visualizer',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        pose_visualizer,
        rviz_node
    ])