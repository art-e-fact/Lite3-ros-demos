from pathlib import Path

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


def _as_bool(value, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _configured_scene(config_path: str) -> str:
    try:
        with open(config_path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except OSError:
        return ""
    if not isinstance(data, dict):
        return ""
    return str(data.get("scene", "")).strip()


def launch_setup(context, *args, **kwargs):
    mode = int(LaunchConfiguration('mode').perform(context))
    localization = LaunchConfiguration('localization').perform(context)
    localization = localization.lower() == 'true'
    control_type = int(LaunchConfiguration('control_type').perform(context))
    simulation_config = LaunchConfiguration('simulation_config').perform(context).strip()
    nav2_params_filepath_launch_arg = LaunchConfiguration('nav2_params_filepath')
    use_sim_time = LaunchConfiguration("use_sim_time")
    database_path = LaunchConfiguration("database_path")
    scene_id = int(LaunchConfiguration('scene_id').perform(context))
    scene = LaunchConfiguration('scene').perform(context).strip()
    procedural_env_seed = LaunchConfiguration('procedural_env_seed')
    xml_path = LaunchConfiguration('xml').perform(context).strip()

    nav2_package_share = FindPackageShare("nav2_bringup").perform(context)
    lite3_package_share = FindPackageShare("lite3_sdk_deploy").perform(context)
    selected_config = simulation_config or _mode_simulation_config(lite3_package_share, mode)

    if not scene:
        if xml_path:
            scene = f"src/Lite3_sdk_deploy/Lite3_description/lite3_mjcf/mjcf/{xml_path}"
        elif scene_id == 1:
            scene = "procedural://blocks"
        else:
            scene = _configured_scene(selected_config)
    is_procedural_scene = scene.startswith("procedural://")

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

    enable_lidar = mode in (0, 2)
    enable_depth = mode in (1, 2)
    enable_color = enable_depth

    enable_pointcloud = LaunchConfiguration('enable_pointcloud').perform(context).lower() == 'true'

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

    start_simulation_args = [
        '--config', selected_config,
        '--set', f'scene={scene}',
        '--set', f'sensors.lidar_2d.enabled={str(enable_lidar).lower()}',
        '--set', f'sensors.realsense.enable_depth={str(enable_depth).lower()}',
        '--set', f'sensors.realsense.enable_color={str(enable_color).lower()}',
        '--set', f'sensors.realsense.enable_pointcloud={str(enable_pointcloud).lower()}',
    ]
    procedural_env_seed_value = str(procedural_env_seed.perform(context)).strip()
    if procedural_env_seed_value:
        start_simulation_args.extend(['--set', f'procedural_env_seed={procedural_env_seed_value}'])

    if not is_procedural_scene:
        rtabmap_args["max_ground_height"] = '0.3'
        rtabmap_args["max_ground_angle"] = '60'

    return [
        # MuJoCo simulation
        Node(
            package='lite3_sdk_deploy', 
            executable='start_simulation.py',
            output='screen',
            arguments=start_simulation_args,
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
            'enable_pointcloud', default_value='false',
            description='Publish RealSense pointcloud (debug; off by default)'
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
            'scene_id', default_value='0',
            description='Legacy scene selector: 0 (authored/default scene), 1 (procedural scene).'
        ),

        DeclareLaunchArgument(
            'scene', default_value='',
            description='Override the scene path or set a procedural scene URI such as procedural://blocks.'
        ),

        DeclareLaunchArgument(
            'procedural_env_seed', default_value='-1',
            description='Seed for procedural scene generation; -1 selects a random seed.'
        ),

        DeclareLaunchArgument(
            'xml', default_value='',
            description='Top-level MuJoCo XML scene file name from src/Lite3_sdk_deploy/Lite3_description/lite3_mjcf/mjcf.'
        ),

        DeclareLaunchArgument(
            'simulation_config', default_value='',
            description='Optional simulation YAML config path. Defaults to the mode-specific config when empty.'
        ),

        OpaqueFunction(function=launch_setup)
    ])
