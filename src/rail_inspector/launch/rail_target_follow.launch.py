from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    enable_heightmap = LaunchConfiguration('enable_heightmap').perform(context).strip().lower() == 'true'
    use_rl_deploy_controller = (
        LaunchConfiguration('use_rl_deploy_controller').perform(context).strip().lower() == 'true'
    )
    cloud_topic = LaunchConfiguration('cloud_topic').perform(context).strip()
    use_sim_time = LaunchConfiguration('use_sim_time')
    follow_mode = LaunchConfiguration('follow_mode')
    follow_distance = LaunchConfiguration('follow_distance')
    min_linear_x = LaunchConfiguration('min_linear_x')
    max_linear_x = LaunchConfiguration('max_linear_x')
    distance_error_for_max_speed = LaunchConfiguration('distance_error_for_max_speed')
    max_linear_y = LaunchConfiguration('max_linear_y')
    max_angular_z = LaunchConfiguration('max_angular_z')
    k_center = LaunchConfiguration('k_center')
    k_heading = LaunchConfiguration('k_heading')
    stale_timeout_sec = LaunchConfiguration('stale_timeout_sec')
    visualize_with_rviz = LaunchConfiguration('visualize_with_rviz')
    visualize_with_rerun = LaunchConfiguration('visualize_with_rerun')
    rerun_save_path = LaunchConfiguration('rerun_save_path')
    rerun_recording_id = LaunchConfiguration('rerun_recording_id')

    actions = []

    if enable_heightmap and cloud_topic:
        actions.extend([
            Node(
                package='simple_local_heightmap',
                executable='local_heightmap_node',
                name='local_heightmap_node',
                output='screen',
                parameters=[{
                    'cloud_topic': cloud_topic,
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
            ),
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
                    'visualize_with_rviz': visualize_with_rviz,
                    'visualize_with_rerun': visualize_with_rerun,
                    'rerun_save_path': rerun_save_path,
                    'rerun_recording_id': rerun_recording_id,
                }],
            ),
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
                    'follow_mode': follow_mode,
                }],
            ),
        ])
    elif enable_heightmap:
        actions.append(
            LogInfo(
                msg=(
                    'rail heightmap pipeline not started: '
                    'cloud_topic must be non-empty when enable_heightmap:=true'
                )
            )
        )

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
            'follow_mode',
            default_value='auto',
            description='Control mode: "auto" (follows target automatically) or "teleop" (uses follow_rail_speed)',
        ),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/mid360/points',
            description='Point cloud topic fed to the local heightmap node',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock published on /clock',
        ),
        DeclareLaunchArgument(
            'follow_distance',
            default_value='1.5',
            description='Desired stand-off distance to the detected target in metres',
        ),
        DeclareLaunchArgument(
            'min_linear_x',
            default_value='0.35',
            description='Minimum forward command that reliably starts locomotion',
        ),
        DeclareLaunchArgument(
            'max_linear_x',
            default_value='0.45',
            description='Maximum forward speed command in metres per second',
        ),
        DeclareLaunchArgument(
            'distance_error_for_max_speed',
            default_value='1.5',
            description='Distance error at which forward speed reaches max_linear_x',
        ),
        DeclareLaunchArgument(
            'max_linear_y',
            default_value='0.4',
            description='Maximum lateral speed command in metres per second',
        ),
        DeclareLaunchArgument(
            'max_angular_z',
            default_value='0.5',
            description='Maximum yaw-rate command in radians per second',
        ),
        DeclareLaunchArgument(
            'k_center',
            default_value='1.0',
            description='Gain for rail centre offset to lateral correction speed',
        ),
        DeclareLaunchArgument(
            'k_heading',
            default_value='1.2',
            description='Gain for rail tangent yaw error to angular speed',
        ),
        DeclareLaunchArgument(
            'stale_timeout_sec',
            default_value='0.5',
            description='Maximum wall-time age accepted for detector and odometry inputs',
        ),
        DeclareLaunchArgument(
            'visualize_with_rviz',
            default_value='false',
            description='Whether to publish RViz debug markers',
        ),
        DeclareLaunchArgument(
            'visualize_with_rerun',
            default_value='false',
            description='Whether to visualize detector states with Rerun.io',
        ),
        DeclareLaunchArgument(
            'rerun_save_path',
            default_value='',
            description='Optional file path to save the Rerun recording (.rrd)',
        ),
        DeclareLaunchArgument(
            'rerun_recording_id',
            default_value='',
            description='Optional custom recording ID for Rerun',
        ),
        OpaqueFunction(function=launch_setup),
    ])
