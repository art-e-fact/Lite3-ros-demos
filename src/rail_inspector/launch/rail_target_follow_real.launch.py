import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rail_inspector_share = get_package_share_directory('rail_inspector')

    livox_pkg = get_package_share_directory("livox_ros_driver2")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([f"{livox_pkg}/launch_ROS2/msg_MID360s_launch.py"]),
        ),   
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
            default_value='0.54',
            description='Expected distance between the two rails in meters',
        ),
        DeclareLaunchArgument(
            'rail_width',
            default_value='0.028',
            description='Expected lateral width of one rail in meters',
        ),
        DeclareLaunchArgument(
            'forward_span',
            default_value='2.6',
            description='Forward distance ahead of the robot covered by the sampled rail slices',
        ),
        DeclareLaunchArgument(
            'backward_span',
            default_value='0.0',
            description='Backward distance behind the robot covered by the sampled rail slices',
        ),
        DeclareLaunchArgument(
            'num_slices',
            default_value='30',
            description='Number of cross-sections sampled from backward_span behind to forward_span ahead',
        ),
        DeclareLaunchArgument(
            'lateral_search_width',
            default_value='0.6',
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
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/leg_odom2',
            description='Odometry topic used by the rail detector and follower nodes',
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
                'forward_span': LaunchConfiguration('forward_span'),
                'backward_span': LaunchConfiguration('backward_span'),
                'follow_mode': LaunchConfiguration('follow_mode'),
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'use_sim_time': 'false',
                'use_rl_deploy_controller': 'false'
            }.items()
        ),
    ])
