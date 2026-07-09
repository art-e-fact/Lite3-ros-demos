import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    enable_heightmap = LaunchConfiguration('enable_heightmap').perform(context).strip().lower() == 'true'
    use_rl_deploy_controller = (
        LaunchConfiguration('use_rl_deploy_controller').perform(context).strip().lower() == 'true'
    )
    enable_drive_watchdog = (
        LaunchConfiguration('enable_drive_watchdog').perform(context).strip().lower() == 'true'
    )
    drive_label = LaunchConfiguration('drive_label').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    follow_mode = LaunchConfiguration('follow_mode').perform(context).strip()

    follower_overrides = {}
    if follow_mode:
        follower_overrides['follow_mode'] = follow_mode

    actions = []

    if enable_heightmap:
        actions.extend([
            Node(
                package='simple_local_heightmap',
                executable='local_heightmap_node',
                name='local_heightmap_node',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='rail_inspector',
                executable='rail_detector_node',
                name='rail_detector_node',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='rail_inspector',
                executable='rail_target_follower_node',
                name='rail_target_follower_node',
                output='screen',
                parameters=[params_file, follower_overrides],
            ),
        ])

    if use_rl_deploy_controller:
        actions.append(
            Node(
                package='lite3_sdk_deploy',
                executable='rl_deploy',
                name='rl_deploy',
                output='screen',
                arguments=['--twist'],
            )
        )

    if enable_drive_watchdog:
        actions.append(
            Node(
                package='rail_inspector',
                executable='drive_watchdog_node',
                name='drive_watchdog_node',
                output='screen',
                parameters=[{'drive_label': drive_label}],
            )
        )

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('rail_inspector')
    default_params = os.path.join(pkg_share, 'config', 'rail_follow_sim.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_heightmap',
            default_value='true',
            description='Launch the heightmap, rail detector, and rail follower nodes',
        ),
        DeclareLaunchArgument(
            'use_rl_deploy_controller',
            default_value='true',
            description='Launch lite3_sdk_deploy rl_deploy to convert /cmd_vel into joint commands',
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
        DeclareLaunchArgument(
            'enable_drive_watchdog',
            default_value='true',
            description='Launch drive_watchdog_node, which publishes /emergency_stop when the watched drive is removed.',
        ),
        DeclareLaunchArgument(
            'drive_label',
            default_value='NO NAME',
            description='Volume label of the USB drive to watch (works on both macOS sim and Linux robot).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
