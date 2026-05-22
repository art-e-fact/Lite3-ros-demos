from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory


MAPPING_MODE = 0
NAVIGATION_MODE = 1

def launch_setup(context, *args, **kwargs):
    mode = int(LaunchConfiguration('mode').perform(context))
    control_type = int(LaunchConfiguration('control_type').perform(context))
    scene_id = int(LaunchConfiguration('scene_id').perform(context))
    use_procedural_scene = LaunchConfiguration('use_procedural_scene').perform(context).lower() == 'true'
    procedural_env_seed = LaunchConfiguration('procedural_env_seed')
    xml_path = LaunchConfiguration('xml').perform(context).strip()
    effective_use_procedural_scene = use_procedural_scene or scene_id == 1
    enable_pointcloud = LaunchConfiguration('enable_pointcloud').perform(context).lower() == 'true'
    enable_mid360 = LaunchConfiguration("enable_mid360")

    lite3_package_share = FindPackageShare("lite3_sdk_deploy").perform(context)

    enable_lidar_2D = False
    enable_depth = False
    enable_color = False

    if mode == MAPPING_MODE:
        rtabmap_mode = None
        rviz_filepath = f"{lite3_package_share}/config/mapping_mid360s.rviz"
        lego_loam_bor_params = [[
            f"{lite3_package_share}/config/mapping_mid360s.yaml"
        ]]        

    elif mode == NAVIGATION_MODE:
        rtabmap_mode = None
        rviz_filepath = f"{lite3_package_share}/config/navigation_mid360s.rviz"
        lego_loam_bor_params = [[
            f"{lite3_package_share}/config/navigation_mid360s.yaml"
        ]] 

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
        "enable_mid360": enable_mid360,
        "enable_lidar": enable_lidar_2D,
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


    return [
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package="lego_loam_bor",
                    executable="lego_loam",
                    output="screen",
                    parameters=lego_loam_bor_params,
                    remappings=[
                        ('/lslidar_point_cloud', '/mid360/points'),
                        ('/odom', '/odom')
                    ],
                ),

                Node(
                    package="mcl_3dl",
                    executable="mcl_3dl",
                    output="screen",
                    parameters=[
                        f"{lite3_package_share}/config/navigation_mid360s.yaml"
                    ],
                    remappings=[
                        ('/lslidar_point_cloud', '/mid360/points'),
                        ('/odom', '/odom')
                    ],
                    condition=IfCondition(str(mode == NAVIGATION_MODE))
                ),

                Node(
                    package="global_planner",
                    executable="global_planner_node",
                    output="screen",
                    parameters=[
                        f"{lite3_package_share}/config/navigation_mid360s.yaml"
                    ],
                    remappings=[
                        ('/lslidar_point_cloud', '/mid360/points'),
                        ('/odom', '/odom')
                    ],
                    condition=IfCondition(str(mode == NAVIGATION_MODE))
                ),

                Node(
                    package="p2p_move_base",
                    executable="p2p_move_base_node",
                    output="screen",
                    parameters=[
                        f"{lite3_package_share}/config/navigation_mid360s.yaml"
                    ],
                    remappings=[
                        ('/lslidar_point_cloud', '/mid360/points'),
                        ('/odom', '/odom')
                    ],
                    condition=IfCondition(str(mode == NAVIGATION_MODE))
                ),

                Node(
                    package="p2p_move_base",
                    executable="clicked2goal.py",
                    output="screen",
                    parameters=[
                        f"{lite3_package_share}/config/navigation_mid360s.yaml"
                    ],
                    remappings=[
                        ('/lslidar_point_cloud', '/mid360/points'),
                        ('/odom', '/odom')
                    ],
                    condition=IfCondition(str(mode == NAVIGATION_MODE))
                ),
            ]
        ),


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

        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_filepath]
        ),

    ]

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'mode', default_value='0',
            description='DDDMR mode: 0 (mapping), 1 (navigation)'
        ),

        DeclareLaunchArgument(
            'enable_mid360', default_value='true',
            description='Publish Mid360 pointcloud (off by default)'
        ),

        DeclareLaunchArgument(
            'enable_pointcloud', default_value='false',
            description='Publish RealSense pointcloud (debug; off by default)'
        ),

        DeclareLaunchArgument(
            'control_type', default_value='0',
            description='Joints control type: 0 (twist), 1 (keyboard), 2 (gamepad)'
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