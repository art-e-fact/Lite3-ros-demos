from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _mode_simulation_config(package_share: str, mode: int) -> str:
    if mode == 0:
        return f"{package_share}/config/simulations/mujoco_procedural_lidar.yaml"
    if mode == 1:
        return f"{package_share}/config/simulations/mujoco_procedural_rgbd.yaml"
    return f"{package_share}/config/simulations/mujoco_procedural_rgbd_lidar.yaml"


def launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    database_path = LaunchConfiguration("database_path")
    simulation_config = LaunchConfiguration("simulation_config").perform(context).strip()

    mode = int(LaunchConfiguration('mode').perform(context))
    localization = LaunchConfiguration('localization').perform(context)
    localization = localization.lower() == 'true'
    control_type = int(LaunchConfiguration('control_type').perform(context))

    package_share = FindPackageShare("lite3_sdk_deploy").perform(context)
    headless_value = headless.perform(context).strip().lower()

    ## rtabmap modes
    if mode == 0:
        rtabmap_mode = "lidar"
        rviz_filepath = f"{package_share}/config/mapping_lidar.rviz"
    elif mode == 1:
        rtabmap_mode = "rgbd"
        rviz_filepath = f"{package_share}/config/mapping_rgbd.rviz"
    else:
        rtabmap_mode = "rgbd_lidar"
        rviz_filepath = f"{package_share}/config/mapping_rgbd_lidar.rviz"
    selected_config = simulation_config or _mode_simulation_config(package_share, mode)

    ## rl_deploy
    rl_deploy_prefix = ''
    if control_type == 0:
        rl_deploy_args = ["--twist"]
    elif control_type == 1:
        rl_deploy_args = []
        rl_deploy_prefix = 'xterm -e'
    else:
        rl_deploy_args = ["--gamepad"]
    
    return [
        # MuJoCo simulation
        Node(
            package="lite3_sdk_deploy",
            executable="start_simulation.py",
            output="screen",
            arguments=["--config", selected_config, *(["--set", f"headless={headless_value}"] if headless_value in {"true", "false"} else [])],
        ),

        # RL controller
        Node(
            package="lite3_sdk_deploy",
            executable="rl_deploy",
            output="screen",
            arguments=rl_deploy_args,
            prefix=rl_deploy_prefix,
        ),

        # Auto navigation
        Node(
            package="lite3_sdk_deploy",
            executable="auto_waypoint_navigator.py",
            output="screen",
        ),
        
        # Rviz
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_filepath],
            condition=IfCondition(
                PythonExpression(["'", use_rviz, "' == 'true' and '", headless, "' != 'true'"])
            ),
        ),

        # RTAB-Map launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{package_share}/launch/rtabmap_{rtabmap_mode}.launch.py"),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "localization": str(localization).lower(),
                "database_path": database_path,
            }.items(),
        )
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("database_path", default_value="~/.ros/rtabmap.db"),
            DeclareLaunchArgument("simulation_config", default_value=""),

            DeclareLaunchArgument(
                'mode', default_value='2',
                description='RTAB-Map mode: 0 (lidar), 1 (rgbd), 2 (lidar+rgbd)'
            ),

            DeclareLaunchArgument(
                'localization', 
                default_value='false',
                description='Launch in localization mode.'
            ),

            DeclareLaunchArgument(
                'control_type', default_value='0',
                description='Joints control type: 0 (twist), 1 (keyboard), 2 (gamepad)'
            ),

            DeclareLaunchArgument(
                'nav2_rviz_filepath',
                default_value=PathJoinSubstitution([
                    FindPackageShare("lite3_sdk_deploy"), 'config', 'mapping2.rviz'
                ]),
            ),

            OpaqueFunction(function=launch_setup),
        ]
    )
