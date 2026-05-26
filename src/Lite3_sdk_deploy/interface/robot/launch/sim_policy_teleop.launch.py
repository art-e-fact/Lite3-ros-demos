from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_simulation_config(package_share: str) -> str:
    return f"{package_share}/config/simulations/mujoco_procedural.yaml"


def launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    simulation_config = LaunchConfiguration("simulation_config").perform(context).strip()

    use_keyboard_teleop = LaunchConfiguration("use_keyboard_teleop")
    use_joy_teleop = LaunchConfiguration("use_joy_teleop")

    joy_config = PathJoinSubstitution([
        FindPackageShare("lite3_sdk_deploy"), "config", "f310_holonomic.yaml"
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("lite3_sdk_deploy"), "config", "mapping_lidar.rviz"
    ])
    package_share = FindPackageShare("lite3_sdk_deploy").perform(context)
    headless_value = headless.perform(context).strip().lower()
    selected_config = simulation_config or _default_simulation_config(package_share)

    return [
        Node(
            package="lite3_sdk_deploy",
            executable="start_simulation.py",
            output="screen",
            arguments=["--config", selected_config, *(["--set", f"headless={headless_value}"] if headless_value in {"true", "false"} else [])],
        ),

        Node(
            package="lite3_sdk_deploy",
            executable="rl_deploy",
            output="screen",
            arguments=["--twist"],
        ),

        Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            name="teleop_twist_keyboard",
            output="screen",
            remappings=[("cmd_vel", "/cmd_vel")],
            emulate_tty=True,
            condition=IfCondition(
                PythonExpression([
                    "'", use_keyboard_teleop, "' == 'true' and '", headless, "' != 'true'"
                ])
            ),
        ),

        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_joy_teleop")),
        ),

        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            output="screen",
            parameters=[joy_config],
            remappings=[("cmd_vel", "/cmd_vel")],
            condition=IfCondition(LaunchConfiguration("use_joy_teleop")),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(
                PythonExpression([
                    "'", use_rviz, "' == 'true' and '", headless, "' != 'true'"
                ])
            ),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("simulation_config", default_value=""),

        DeclareLaunchArgument(
            "use_keyboard_teleop",
            default_value="false",
            description="Launch teleop_twist_keyboard (interactive terminal required)",
        ),
        DeclareLaunchArgument(
            "use_joy_teleop",
            default_value="false",
            description="Launch joy_node + teleop_twist_joy",
        ),

        OpaqueFunction(function=launch_setup),
    ])
