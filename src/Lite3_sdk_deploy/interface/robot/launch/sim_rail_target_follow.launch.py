import os
import platform
import shutil

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _load_simulation_config(path: str) -> dict:
    if not path:
        return {}
    try:
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except OSError:
        return {}
    return data if isinstance(data, dict) else {}


def _resolve_simulator_script() -> str:
    """Locate the installed start_simulation.py inside the lite3_sdk_deploy package."""
    share_dir = get_package_share_directory('lite3_sdk_deploy')
    # Installed via `install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME})`,
    # which lives at <prefix>/lib/lite3_sdk_deploy/start_simulation.py while
    # the share dir is at <prefix>/share/lite3_sdk_deploy. Walk up to <prefix>.
    prefix = os.path.dirname(os.path.dirname(share_dir))
    return os.path.join(prefix, 'lib', 'lite3_sdk_deploy', 'start_simulation.py')


def _simulator_action(simulation_config_path: str, headless: bool):
    """Spawn the simulator.

    On macOS, MuJoCo's interactive viewer (`mujoco.viewer.launch_passive`)
    must run under the `mjpython` interpreter shipped with the `mujoco`
    PyPI/conda package, because Apple requires Cocoa UI to live on the
    main thread. When the simulator is configured headless, the plain
    Python interpreter works fine and we keep the standard ROS `Node`
    action so ros2 lifecycle/composition behaves identically to Linux.
    """
    needs_mjpython = (platform.system() == 'Darwin') and (not headless)
    if not needs_mjpython:
        return Node(
            package='lite3_sdk_deploy',
            executable='start_simulation.py',
            output='screen',
            arguments=['--config', simulation_config_path],
        )

    mjpython = shutil.which('mjpython')
    if mjpython is None:
        # Fall back with a loud warning; the viewer will fail to open but
        # at least the user gets a clear hint instead of a cryptic Cocoa
        # error.
        return LogInfo(
            msg=(
                "[sim_rail_target_follow] WARNING: headless=false on macOS but "
                "`mjpython` is not on PATH. Install the `mujoco` package "
                "into the active env (`pixi add mujoco`) or set headless=true "
                "in your simulation_config YAML."
            )
        )

    script = _resolve_simulator_script()
    return ExecuteProcess(
        cmd=[mjpython, script, '--config', simulation_config_path],
        output='screen',
        # Ensure ros2 Python entrypoints are importable when running under
        # mjpython rather than via the ROS `Node` action wrapper.
        additional_env={'PYTHONUNBUFFERED': '1'},
    )


def launch_setup(context, *args, **kwargs):
    simulation_config_path = LaunchConfiguration('simulation_config').perform(context).strip()
    simulation_config = _load_simulation_config(simulation_config_path)
    headless = _as_bool(simulation_config.get('headless'), True)
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_heightmap = LaunchConfiguration('enable_heightmap')
    enable_heightmap_value = enable_heightmap.perform(context).lower() == 'true'

    sensors = simulation_config.get('sensors') or {}
    realsense = sensors.get('realsense') or {}
    enable_mid360_value = _as_bool((sensors.get('mid360') or {}).get('enabled'), False)
    enable_depth_value = _as_bool(realsense.get('enable_depth'), False)
    enable_pointcloud_value = _as_bool(realsense.get('enable_pointcloud'), False)

    use_keyboard_teleop = LaunchConfiguration('use_keyboard_teleop')
    use_joy_teleop = LaunchConfiguration('use_joy_teleop')
    follow_distance = LaunchConfiguration('follow_distance')
    min_linear_x = LaunchConfiguration('min_linear_x')
    max_linear_x = LaunchConfiguration('max_linear_x')
    distance_error_for_max_speed = LaunchConfiguration('distance_error_for_max_speed')
    max_linear_y = LaunchConfiguration('max_linear_y')
    max_angular_z = LaunchConfiguration('max_angular_z')
    k_center = LaunchConfiguration('k_center')
    k_heading = LaunchConfiguration('k_heading')
    stale_timeout_sec = LaunchConfiguration('stale_timeout_sec')

    joy_config = PathJoinSubstitution([
        FindPackageShare('lite3_sdk_deploy'), 'config', 'f310_holonomic.yaml'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('lite3_sdk_deploy'), 'config', 'teleop.rviz'
    ])

    heightmap_cloud_topic = None
    if enable_mid360_value:
        heightmap_cloud_topic = '/mid360/points'
    elif enable_depth_value and enable_pointcloud_value:
        heightmap_cloud_topic = '/camera/depth/color/points'

    actions = [
        _simulator_action(simulation_config_path, headless),
    ]

    if enable_heightmap_value and heightmap_cloud_topic is not None:
        actions.append(
            Node(
                package='simple_local_heightmap',
                executable='local_heightmap_node',
                name='local_heightmap_node',
                output='screen',
                parameters=[{
                    'cloud_topic': heightmap_cloud_topic,
                    'map_frame': 'odom',
                    'robot_frame': 'base_link',
                    'use_sim_time': use_sim_time,
                    'resolution': 0.025,
                    'length_x': 8.0,
                    'length_y': 8.0,
                    'front_clear_enabled': True,
                    'front_clear_length': 2.5,
                    'front_clear_width': 1.0,
                    'front_clear_offset_x': 0.25,
                    'front_stale_time_sec': 0.75,
                }],
            )
        )
        actions.append(
            Node(
                package='rail_inspector',
                executable='rail_detector_node',
                name='rail_detector_node',
                output='screen',
                parameters=[{
                    'heightmap_topic': '/local_heightmap',
                    'odom_topic': '/odom',
                    'marker_topic': '/rail_detector/markers',
                    'center_offset_topic': '/rail_detector/center_offset',
                    'tangent_yaw_topic': '/rail_detector/tangent_yaw',
                    'target_distance_topic': '/rail_detector/target_distance',
                    'use_sim_time': use_sim_time,
                    'track_gauge': 1.067,
                }],
            )
        )
        actions.append(
            Node(
                package='rail_inspector',
                executable='rail_target_follower_node',
                name='rail_target_follower_node',
                output='screen',
                parameters=[{
                    'cmd_vel_topic': '/cmd_vel',
                    'odom_topic': '/odom',
                    'center_offset_topic': '/rail_detector/center_offset',
                    'tangent_yaw_topic': '/rail_detector/tangent_yaw',
                    'target_distance_topic': '/rail_detector/target_distance',
                    'use_sim_time': use_sim_time,
                    'follow_distance': follow_distance,
                    'min_linear_x': min_linear_x,
                    'max_linear_x': max_linear_x,
                    'distance_error_for_max_speed': distance_error_for_max_speed,
                    'max_linear_y': max_linear_y,
                    'max_angular_z': max_angular_z,
                    'k_center': k_center,
                    'k_heading': k_heading,
                    'stale_timeout_sec': stale_timeout_sec,
                }],
            )
        )
    elif enable_heightmap_value:
        actions.append(
            LogInfo(
                msg=(
                    'rail heightmap pipeline not started: enable_mid360:=true or '
                    'enable_depth:=true enable_pointcloud:=true is required'
                )
            )
        )

    actions.extend([
        Node(
            package='lite3_sdk_deploy',
            executable='rl_deploy',
            output='screen',
            arguments=['--twist'],
        ),
    ])

    if use_keyboard_teleop.perform(context).lower() == 'true' and not headless:
        actions.append(
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_keyboard',
                output='screen',
                remappings=[('cmd_vel', '/cmd_vel')],
                emulate_tty=True,
            )
        )

    if use_joy_teleop.perform(context).lower() == 'true':
        actions.extend([
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
            ),
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                output='screen',
                parameters=[joy_config],
                remappings=[('cmd_vel', '/cmd_vel')],
            ),
        ])

    if use_rviz.perform(context).lower() == 'true' and not headless:
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('lite3_sdk_deploy'), 'config', 'simulations', 'mujoco_railroad_mid360.yaml'
            ]),
            description='Path to the Lite3 simulation YAML config to run',
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock published on /clock',
        ),
        DeclareLaunchArgument(
            'enable_heightmap',
            default_value='true',
            description='Launch the simple local heightmap, rail detector, and rail follower nodes',
        ),
        DeclareLaunchArgument(
            'follow_distance',
            default_value='1.5',
            description='Desired stand-off distance to the detected target in meters',
        ),
        DeclareLaunchArgument(
            'min_linear_x',
            default_value='0.35',
            description='Minimum forward command that reliably starts locomotion',
        ),
        DeclareLaunchArgument(
            'max_linear_x',
            default_value='0.45',
            description='Maximum forward body-frame speed command in meters per second',
        ),
        DeclareLaunchArgument(
            'distance_error_for_max_speed',
            default_value='1.5',
            description='Distance error at which forward speed reaches max_linear_x',
        ),
        DeclareLaunchArgument(
            'max_linear_y',
            default_value='0.4',
            description='Maximum lateral body-frame speed command in meters per second',
        ),
        DeclareLaunchArgument(
            'max_angular_z',
            default_value='0.5',
            description='Maximum yaw-rate command in radians per second',
        ),
        DeclareLaunchArgument(
            'k_center',
            default_value='1.0',
            description='Gain that converts rail center offset into lateral correction speed',
        ),
        DeclareLaunchArgument(
            'k_heading',
            default_value='1.2',
            description='Gain that converts rail tangent yaw error into angular speed',
        ),
        DeclareLaunchArgument(
            'stale_timeout_sec',
            default_value='0.5',
            description='Maximum wall-time age accepted for detector and odometry inputs',
        ),
        DeclareLaunchArgument(
            'use_keyboard_teleop',
            default_value='false',
            description='Launch teleop_twist_keyboard (interactive terminal required)',
        ),
        DeclareLaunchArgument(
            'use_joy_teleop',
            default_value='false',
            description='Launch joy_node + teleop_twist_joy',
        ),
        OpaqueFunction(function=launch_setup),
    ])