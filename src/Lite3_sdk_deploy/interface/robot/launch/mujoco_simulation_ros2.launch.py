from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    mode = int(LaunchConfiguration('mode').perform(context))
    localization = LaunchConfiguration('localization').perform(context)
    localization = localization.lower() == 'true'
    control_type = int(LaunchConfiguration('control_type').perform(context))
    nav2_params_filepath_launch_arg = LaunchConfiguration('nav2_params_filepath')
    use_sim_time = LaunchConfiguration("use_sim_time")
    database_path = LaunchConfiguration("database_path")
    scene_id = int(LaunchConfiguration('scene_id').perform(context))
    use_procedural_scene = LaunchConfiguration('use_procedural_scene').perform(context).lower() == 'true'
    procedural_env_seed = LaunchConfiguration('procedural_env_seed')
    xml_path = LaunchConfiguration('xml').perform(context).strip()
    effective_use_procedural_scene = use_procedural_scene or scene_id == 1

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
        enable_lidar = True
        enable_depth = False
        enable_color = False
    elif mode == 1:
        rtabmap_mode = "rgbd"
        rviz_filepath = f"{lite3_package_share}/config/mapping_rgbd_costmaps.rviz"
        enable_lidar = False
        enable_depth = True
        enable_color = True
    else:
        rtabmap_mode = "rgbd_lidar"
        rviz_filepath = f"{lite3_package_share}/config/mapping_rgbd_lidar_costmaps.rviz"
        enable_lidar = True
        enable_depth = True
        enable_color = True


    enable_pointcloud = LaunchConfiguration('enable_pointcloud').perform(context).lower() == 'true'
    enable_mid360 = LaunchConfiguration('enable_mid360').perform(context).lower() == 'true'

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

    ## scene
    mujoco_simulation_ros2_params = {
        "enable_lidar": enable_lidar,
        "enable_mid360": enable_mid360,
        "enable_depth": enable_depth,
        "enable_color": enable_color,
        "enable_pointcloud": enable_pointcloud,
        "use_procedural_scene": effective_use_procedural_scene,
        "procedural_env_seed": procedural_env_seed,
    }
    mujoco_simulation_ros2_args = []
    if xml_path:
        xml_path = f"src/Lite3_sdk_deploy/Lite3_description/lite3_mjcf/mjcf/{xml_path}"
        mujoco_simulation_ros2_args = ["--xml", xml_path]

    if not effective_use_procedural_scene:
        rtabmap_args["max_ground_height"] = '0.3'
        rtabmap_args["max_ground_angle"] = '60'

    return [
        # MuJoCo simulation
        Node(
            package='lite3_sdk_deploy', 
            executable='mujoco_simulation_ros2.py',
            output='screen',
            arguments=mujoco_simulation_ros2_args,
            parameters = [mujoco_simulation_ros2_params],
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
            'enable_mid360', default_value='false',
            description='Publish Mid360 pointcloud (off by default)'
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
            'use_procedural_scene', default_value='false',
            description='Generate the MuJoCo environment procedurally at runtime.'
        ),

        DeclareLaunchArgument(
            'procedural_env_seed', default_value='-1',
            description='Seed for procedural scene generation; -1 selects a random seed.'
        ),

        DeclareLaunchArgument(
            'xml', default_value='',
            description='Top-level MuJoCo XML scene file name from src/Lite3_sdk_deploy/Lite3_description/lite3_mjcf/mjcf.'
        ),

        OpaqueFunction(function=launch_setup)
    ])
