from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _default_simulation_config(package_share: str) -> str:
    return f"{package_share}/config/simulations/newton_viewer.yaml"


def launch_setup(context, *args, **kwargs):
    control_type = int(LaunchConfiguration("control_type").perform(context))
    headless_value = LaunchConfiguration("headless").perform(context).strip().lower()
    simulation_config = LaunchConfiguration("simulation_config").perform(context).strip()
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    nav2_package_share = FindPackageShare("nav2_bringup").perform(context)
    lite3_package_share = FindPackageShare("lite3_sdk_deploy").perform(context)
    rviz_launch = f"{nav2_package_share}/launch/rviz_launch.py"

    rl_deploy_prefix = ""
    if control_type == 0:
        rl_deploy_args = ["--twist"]
    elif control_type == 1:
        rl_deploy_args = []
        rl_deploy_prefix = "xterm -e"
    elif control_type == 2:
        rl_deploy_args = ["--gamepad"]
    else:
        raise ValueError("control_type must be 0 (twist), 1 (keyboard), or 2 (gamepad)")

    selected_config = simulation_config or _default_simulation_config(lite3_package_share)
    simulation_args = ["--config", selected_config]
    if headless_value in {"true", "false"}:
        simulation_args.extend(["--set", f"headless={headless_value}"])

    return [
        Node(
            package="lite3_sdk_deploy",
            executable="start_simulation.py",
            name="newton_simulation",
            output="screen",
            arguments=simulation_args,
        ),
        Node(
            package="lite3_sdk_deploy",
            executable="rl_deploy",
            name="rl_deploy",
            output="screen",
            arguments=rl_deploy_args,
            prefix=rl_deploy_prefix,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz_launch]),
            launch_arguments=[
                ("rviz_config", rviz_config),
            ],
            condition=IfCondition(
                PythonExpression([
                    "'", rviz, "' == 'true' and '", LaunchConfiguration("headless"), "' != 'true'"
                ])
            ),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "control_type",
                default_value="0",
                description="Control type: 0 (twist), 1 (keyboard), 2 (gamepad).",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="",
                description="Optional simulation override. When set to true or false, it overrides the simulation config headless field.",
            ),
            DeclareLaunchArgument(
                "simulation_config",
                default_value="",
                description="Optional simulation YAML. When set, it overrides the built-in preset selection.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution([
                    get_package_share_directory("lite3_sdk_deploy"),
                    "config",
                    "sensors.rviz",
                ]),
                description="RViz config file to load.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )