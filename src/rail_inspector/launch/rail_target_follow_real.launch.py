import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rail_inspector_share = get_package_share_directory('rail_inspector')
    livox_pkg = get_package_share_directory('livox_ros_driver2')
    default_params = os.path.join(rail_inspector_share, 'config', 'rail_follow_office_real.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([f'{livox_pkg}/launch_ROS2/msg_MID360s_launch.py']),
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
            # arguments=['0.2', '0', '0.1', '0', '0', '0.0', 'base_link', 'livox_frame'],
            arguments=['0', '0', '0.1', '0.', '0', '0.0', 'base_link', 'livox_frame'],
            # arguments=['0', '0', '0', '0.', '0', '0.0', 'base_link', 'livox_frame'],
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with parameters for local_heightmap_node, rail_detector_node, and rail_target_follower_node',
        ),
        DeclareLaunchArgument(
            'follow_mode',
            default_value='',
            description='Override follow_mode from params_file (auto or teleop). Leave unset to use the yaml value.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rail_inspector_share, 'launch', 'rail_target_follow.launch.py')
            ),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'follow_mode': LaunchConfiguration('follow_mode'),
                'use_rl_deploy_controller': 'false',
            }.items(),
        ),
    ])

