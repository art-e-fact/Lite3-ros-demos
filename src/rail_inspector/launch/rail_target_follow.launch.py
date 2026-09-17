import os
import shlex

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
    params_file = LaunchConfiguration('params_file').perform(context)
    follow_mode = LaunchConfiguration('follow_mode').perform(context).strip()
    deploy_package = LaunchConfiguration('deploy_package').perform(context).strip()
    deploy_executable = LaunchConfiguration('deploy_executable').perform(context).strip()
    deploy_args_raw = LaunchConfiguration('deploy_args').perform(context).strip()

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
        deploy_args = shlex.split(deploy_args_raw) if deploy_args_raw else []
        actions.append(
            Node(
                package=deploy_package,
                executable=deploy_executable,
                name='rl_deploy',
                output='screen',
                arguments=deploy_args,
                parameters=[{'use_sim_time': True}],
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
            description='Launch rl_deploy to convert /cmd_vel into joint commands',
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
            'deploy_package',
            default_value='lite3_sdk_deploy',
            description='ROS package providing rl_deploy (e.g. lite3_sdk_deploy or m20_sdk_deploy)',
        ),
        DeclareLaunchArgument(
            'deploy_executable',
            default_value='rl_deploy',
            description='Executable name for the low-level RL deploy controller',
        ),
        DeclareLaunchArgument(
            'deploy_args',
            default_value='--twist',
            description='Arguments passed to rl_deploy (e.g. --twist for /cmd_vel input)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
