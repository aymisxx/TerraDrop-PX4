from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_link',
            arguments=[
                '0.1', '0', '0.05',
                '0', '0', '0',
                'base_link',
                'camera_link'
            ],
            output='screen'
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),

                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'approx_sync': True,
                'queue_size': 10,

                'odom_frame_id': 'odom',
                'subscribe_odom_info': False,
                'odom_tf_angular_variance': 0.01,
                'odom_tf_linear_variance': 0.001,

                'visual_odometry': False,

                'grid_cell_size': 0.05,
                'grid_size': 20.0,
                'optimize_from_graph_end': True,
                'optimizer_iterations': 100,

                'loop_closure_activated': True,
                'loop_closure_restriction_type': 0,
                'loop_closure_min_inliers': 20,

                'memory_management': True,
                'max_cloud_size': 50000,
                'min_cluster_size': 100
            }],
            remappings=[
                ('rgb/image', '/drone/front_rgb'),
                ('depth/image', '/drone/front_depth'),
                ('rgb/camera_info', '/drone/front_rgb/camera_info'),

                ('odom', '/fmu/out/vehicle_odometry'),

                ('grid_map', 'map'),
                ('mapData', 'mapData'),
                ('mapPath', 'mapPath'),
                ('cloud_map', 'cloud_map')
            ]
        ),

        Node(
            package='rtabmap_util',
            executable='point_cloud_xyz',
            name='point_cloud_xyz',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'decimation': 4,
                'voxel_size': 0.02,
                'max_depth': 4.0,
                'min_depth': 0.4
            }],
            remappings=[
                ('depth/image', '/drone/front_depth'),
                ('depth/camera_info', '/drone/front_depth/camera_info'),
                ('cloud', 'cloud_xyz')
            ]
        ),

        LogInfo(
            msg='RTAB-Map launched with TerraDrop-PX4 drone configuration'
        )
    ])