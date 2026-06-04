#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('terrain_mapping_drone_control')

    cylinder_landing_launch = os.path.join(
        pkg_share,
        'launch',
        'cylinder_landing.launch.py'
    )

    rtabmap_launch = os.path.join(
        pkg_share,
        'launch',
        'rtabmap.launch.py'
    )

    px4_autopilot_path = LaunchConfiguration('px4_autopilot_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_autopilot_path',
            default_value=os.path.join(
                os.environ.get('HOME', '/home/' + os.environ.get('USER', 'user')),
                'PX4-Autopilot'
            ),
            description='Path to PX4-Autopilot directory'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cylinder_landing_launch),
            launch_arguments={
                'px4_autopilot_path': px4_autopilot_path,
                'use_sim_time': use_sim_time,
            }.items()
        ),

        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rtabmap_launch),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                    }.items()
                )
            ]
        )
    ])