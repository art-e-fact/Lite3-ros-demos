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


def _default_simulation_config(package_share: str) -> str:
    return f"{package_share}/config/simulations/mujoco_mid360.yaml"

def launch_setup(context, *args, **kwargs):
    mode = int(LaunchConfiguration('mode').perform(context))
    control_type = int(LaunchConfiguration('control_type').perform(context))
    simulation_config = LaunchConfiguration('simulation_config').perform(context).strip()

    lite3_package_share = FindPackageShare("lite3_sdk_deploy").perform(context)
    selected_config = simulation_config or _default_simulation_config(lite3_package_share)

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
            'control_type', default_value='0',
            description='Joints control type: 0 (twist), 1 (keyboard), 2 (gamepad)'
        ),

        DeclareLaunchArgument(
            'simulation_config', default_value='',
            description='Optional simulation YAML. When set, it overrides the built-in preset selection.'
        ),

        OpaqueFunction(function=launch_setup)
    ])