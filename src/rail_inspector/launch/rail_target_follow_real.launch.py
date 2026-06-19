import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rail_inspector_share = get_package_share_directory('rail_inspector')

    return LaunchDescription([
        Node(
            package='rail_inspector',
            executable='relay_node',
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            # arguments=["0.2", "0", "0.38", "0.0", "0.38397", "0.0", "base_link", "livox_frame"],
            arguments=["0", "0", "0", "0", "0", "0.0", "base_link", "livox_frame"]
        ),
        DeclareLaunchArgument(
            'track_gauge',
            default_value='0.9',
            description='Expected distance between the two rails in meters',
        ),
        DeclareLaunchArgument(
            'rail_width',
            default_value='0.06',
            description='Expected lateral width of one rail in meters',
        ),
        DeclareLaunchArgument(
            'num_slices',
            default_value='30',
            description='Number of cross-sections sampled across the forward span',
        ),
        DeclareLaunchArgument(
            'lateral_search_width',
            default_value='0.7',
            description='Half-width of each sampled cross-section in meters',
        ),
        DeclareLaunchArgument(
            'follow_mode',
            default_value='teleop',
            description='Control mode: "auto" or "teleop"',
        ),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/livox/lidar',
            description='Point cloud topic fed to the local heightmap node',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rail_inspector_share, 'launch', 'rail_target_follow.launch.py')
            ),
            launch_arguments={
                'track_gauge': LaunchConfiguration('track_gauge'),
                'rail_width': LaunchConfiguration('rail_width'),
                'num_slices': LaunchConfiguration('num_slices'),
                'lateral_search_width': LaunchConfiguration('lateral_search_width'),
                'follow_mode': LaunchConfiguration('follow_mode'),
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'use_sim_time': 'false',
            }.items()
        ),
    ])
