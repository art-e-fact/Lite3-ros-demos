import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    enable_heightmap = LaunchConfiguration('enable_heightmap').perform(context).strip().lower() == 'true'
    use_elevation_mapping = LaunchConfiguration('use_elevation_mapping').perform(context).strip().lower() == 'true'
    use_rl_deploy_controller = (
        LaunchConfiguration('use_rl_deploy_controller').perform(context).strip().lower() == 'true'
    )
    params_file = LaunchConfiguration('params_file').perform(context)
    follow_mode = LaunchConfiguration('follow_mode').perform(context).strip()

    follower_overrides = {}
    if follow_mode:
        follower_overrides['follow_mode'] = follow_mode

    actions = []

    if enable_heightmap:
        if use_elevation_mapping:
            core_param_path = os.path.join(
                get_package_share_directory('elevation_mapping_cupy'),
                'config', 'core', 'core_param.yaml',
            )
            actions.append(
                Node(
                    package='elevation_mapping_cupy',
                    executable='elevation_mapping_node.py',
                    name='elevation_mapping_node',
                    output='screen',
                    parameters=[core_param_path, params_file],
                    remappings=[('/elevation_mapping_node/local_heightmap', '/local_heightmap')],
                )
            )
            detector_params = [params_file, {'heightmap_layout': 'column_major'}]
        else:
            actions.append(
                Node(
                    package='simple_local_heightmap',
                    executable='local_heightmap_node',
                    name='local_heightmap_node',
                    output='screen',
                    parameters=[params_file],
                )
            )
            detector_params = [params_file]

        actions.extend([
            Node(
                package='rail_inspector',
                executable='rail_detector_node',
                name='rail_detector_node',
                output='screen',
                parameters=detector_params,
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
            'use_elevation_mapping',
            default_value='false',
            description=(
                'Use elevation_mapping_cupy as the heightmap source instead of '
                'simple_local_heightmap. Requires an NVIDIA GPU with CUDA, cupy, and torch.'
            ),
        ),
        DeclareLaunchArgument(
            'use_rl_deploy_controller',
            default_value='true',
            description='Launch lite3_sdk_deploy rl_deploy to convert /cmd_vel into joint commands',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with parameters for heightmap, rail_detector_node, and rail_target_follower_node',
        ),
        DeclareLaunchArgument(
            'follow_mode',
            default_value='',
            description='Override follow_mode from params_file (auto or teleop). Leave unset to use the yaml value.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
