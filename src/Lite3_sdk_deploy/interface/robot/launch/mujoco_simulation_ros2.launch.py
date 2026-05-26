from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import yaml


def _mode_simulation_config(package_share: str, mode: int) -> str:
    if mode == 0:
        return f"{package_share}/config/simulations/mujoco_lidar.yaml"
    if mode == 1:
        return f"{package_share}/config/simulations/mujoco_rgbd.yaml"
    return f"{package_share}/config/simulations/mujoco_rgbd_lidar.yaml"


def _uses_procedural_scene(config_path: str) -> bool:
    try:
        with open(config_path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except OSError:
        return False
    return bool(data.get("use_procedural_scene", False))


def launch_setup(context, *args, **kwargs):
    mode = int(LaunchConfiguration('mode').perform(context))
    localization = LaunchConfiguration('localization').perform(context)
    localization = localization.lower() == 'true'
    control_type = int(LaunchConfiguration('control_type').perform(context))
    simulation_config = LaunchConfiguration('simulation_config').perform(context).strip()
    nav2_params_filepath_launch_arg = LaunchConfiguration('nav2_params_filepath')
    use_sim_time = LaunchConfiguration("use_sim_time")
    database_path = LaunchConfiguration("database_path")

    nav2_package_share = FindPackageShare("nav2_bringup").perform(context)
    lite3_package_share = FindPackageShare("lite3_sdk_deploy").perform(context)

    nav2_launch = (
        f"{nav2_package_share}"
        "/launch/navigation_launch.py"
    )
    rviz_launch = (
        f"{nav2_package_share}"
        "/launch/rviz_launch.py"
    )

    ## rtabmap modes
    if mode == 0:
        rtabmap_mode = "lidar"
        rviz_filepath = f"{lite3_package_share}/config/mapping_lidar_costmaps.rviz"
    elif mode == 1:
        rtabmap_mode = "rgbd"
        rviz_filepath = f"{lite3_package_share}/config/mapping_rgbd_costmaps.rviz"
    else:
        rtabmap_mode = "rgbd_lidar"
        rviz_filepath = f"{lite3_package_share}/config/mapping_rgbd_lidar_costmaps.rviz"


    rtabmap_args = {
        "use_sim_time": use_sim_time,
        "localization": str(localization).lower(),
        "database_path": database_path,
    }

    ## rl_deploy
    rl_deploy_prefix = ''
    if control_type == 0:
        rl_deploy_args = ["--twist"]
    elif control_type == 1:
        rl_deploy_args = []
        rl_deploy_prefix = 'xterm -e'
    else:
        rl_deploy_args = ["--gamepad"]

    selected_config = simulation_config or _mode_simulation_config(lite3_package_share, mode)
    if not _uses_procedural_scene(selected_config):
        rtabmap_args["max_ground_height"] = '0.3'
        rtabmap_args["max_ground_angle"] = '60'

    return [
        # MuJoCo simulation
        Node(
            package='lite3_sdk_deploy', 
            executable='start_simulation.py',
            output='screen',
            arguments=['--config', selected_config],
        ),

        # RL controller
        Node(
            package='lite3_sdk_deploy', 
            executable='rl_deploy',
            output='screen',
            arguments=rl_deploy_args,
            prefix=rl_deploy_prefix,
        ),

        # Navigation (planner + controller)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch]),
            launch_arguments=[
                ('params_file', nav2_params_filepath_launch_arg)
            ]
        ),   

        # RViz2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz_launch]),
            launch_arguments=[
                ('rviz_config', rviz_filepath)
            ]
        ),

        # RTAB-Map launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{lite3_package_share}/launch/rtabmap_{rtabmap_mode}.launch.py"),
            launch_arguments=rtabmap_args.items(),
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("database_path", default_value="~/.ros/rtabmap.db"),

        DeclareLaunchArgument(
            'mode', default_value='2',
            description='RTAB-Map mode: 0 (lidar), 1 (rgbd), 2 (lidar+rgbd)'
        ),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'
        ),

        DeclareLaunchArgument(
            'control_type', default_value='0',
            description='Joints control type: 0 (twist), 1 (keyboard), 2 (gamepad)'
        ),

        DeclareLaunchArgument(
            'nav2_params_filepath',
            default_value=PathJoinSubstitution([
                get_package_share_directory('lite3_sdk_deploy'),
                'config', 'nav2_params.yaml'
            ]),
            description='the file path to Nav2 params'
        ),

        DeclareLaunchArgument(
            'nav2_rviz_filepath',
            default_value=PathJoinSubstitution([
                get_package_share_directory('lite3_sdk_deploy'),
                'config', 'mapping2.rviz'
            ]),
            description='the file path to Nav2 rviz config'
        ),

        DeclareLaunchArgument(
            'simulation_config', default_value='',
            description='Optional simulation YAML. When set, it overrides the built-in preset selection.'
        ),

        OpaqueFunction(function=launch_setup)
    ])
