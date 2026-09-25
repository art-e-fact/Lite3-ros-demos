"""Nav2 on a recorded route's map, plus the quadruped's /cmd_vel interface.

The nav side of the demo, launched by test/test_route.py once the simulator
(which runs in its own pixi environment) is up:

    ros2 launch nav2_demo nav2_route.launch.py route:=routes/lab.yaml

The route (recorded with `artefacts-route record`) names the map, so Nav2's
map server, AMCL and the navigation stack come up on it. rl_deploy turns
Nav2's /cmd_vel into joint commands for the robot, as in the rail demo.
"""

import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from artefacts_toolkit_navigation import load_route, route_launch_args


def launch_setup(context, *args, **kwargs):
    route = load_route(LaunchConfiguration('route').perform(context) or None)
    params_file = LaunchConfiguration('params_file').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context).strip().lower() == 'true'
    deploy_package = LaunchConfiguration('deploy_package').perform(context).strip()
    deploy_executable = LaunchConfiguration('deploy_executable').perform(context).strip()
    deploy_args_raw = LaunchConfiguration('deploy_args').perform(context).strip()
    nav2_delay = LaunchConfiguration('nav2_delay').perform(context).strip() or '0'

    # Only the map is taken from the route here: the robot is spawned at the
    # route's start by the simulator (robot.start_pose in its config).
    nav2_args = route_launch_args(route, map_arg='map')
    # The route records the map path as given when recording, relative to the
    # repository root; Nav2's map server wants it absolute.
    nav2_args['map'] = os.path.abspath(nav2_args['map'])
    nav2_args.update({
        'params_file': params_file,
        'use_sim_time': 'True',
        'autostart': 'True',
        'use_composition': 'False',
    })
    nav2_share = get_package_share_directory('nav2_bringup')

    actions = [
        Node(
            package=deploy_package,
            executable=deploy_executable,
            name='rl_deploy',
            output='screen',
            arguments=shlex.split(deploy_args_raw) if deploy_args_raw else [],
            parameters=[{'use_sim_time': True}],
        ),
        # rl_deploy stands the robot up over its first ~6 s, during which the lidar
        # sweeps the ground; Nav2 is brought up after that so AMCL and the costmaps
        # only ever see real scans.
        TimerAction(period=float(nav2_delay), actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={k: v for k, v in nav2_args.items()
                              if k in ('map', 'params_file', 'use_sim_time', 'autostart', 'use_composition')}.items(),
        )]),
    ]
    if use_rviz:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'rviz_launch.py')),
            launch_arguments={'use_sim_time': 'True'}.items(),
        ))
    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_demo')
    return LaunchDescription([
        DeclareLaunchArgument(
            'route', default_value='',
            description='A route file recorded with artefacts-route; empty for the route recorded last',
        ),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg_share, 'config', 'nav2_lite3.yaml'),
            description='Nav2 parameters',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false', description="Open Nav2's RViz view",
        ),
        DeclareLaunchArgument(
            'nav2_delay', default_value='10.0',
            description='Seconds to wait for rl_deploy to stand the robot up before starting Nav2',
        ),
        DeclareLaunchArgument(
            'deploy_package', default_value='lite3_sdk_deploy',
            description='ROS package providing rl_deploy (lite3_sdk_deploy or m20_sdk_deploy)',
        ),
        DeclareLaunchArgument(
            'deploy_executable', default_value='rl_deploy',
            description='Executable name for the low-level RL deploy controller',
        ),
        DeclareLaunchArgument(
            'deploy_args', default_value='--twist',
            description='Arguments passed to rl_deploy (--twist takes /cmd_vel)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
