from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_procedural_scene = LaunchConfiguration("use_procedural_scene")
    procedural_env_seed = LaunchConfiguration("procedural_env_seed")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")

    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_mid360 = LaunchConfiguration("enable_mid360")
    enable_depth = LaunchConfiguration("enable_depth")
    enable_color = LaunchConfiguration("enable_color")
    enable_pointcloud = LaunchConfiguration("enable_pointcloud")

    use_keyboard_teleop = LaunchConfiguration("use_keyboard_teleop")
    use_joy_teleop = LaunchConfiguration("use_joy_teleop")

    joy_config = PathJoinSubstitution([
        FindPackageShare("lite3_sdk_deploy"), "config", "f310_holonomic.yaml"
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("lite3_sdk_deploy"), "config", "mapping_lidar.rviz"
    ])

    return [
        Node(
            package="lite3_sdk_deploy",
            executable="mujoco_simulation_ros2.py",
            output="screen",
            parameters=[{
                "use_procedural_scene": use_procedural_scene,
                "procedural_env_seed": procedural_env_seed,
                "headless": headless,
                "enable_lidar": enable_lidar,
                "enable_mid360": enable_mid360,
                "enable_depth": enable_depth,
                "enable_color": enable_color,
                "enable_pointcloud": enable_pointcloud,
            }],
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
        DeclareLaunchArgument("use_procedural_scene", default_value="true"),
        DeclareLaunchArgument("procedural_env_seed", default_value="-1"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),

        DeclareLaunchArgument("enable_lidar", default_value="false"),
        DeclareLaunchArgument("enable_mid360", default_value="false"),
        DeclareLaunchArgument("enable_depth", default_value="false"),
        DeclareLaunchArgument("enable_color", default_value="false"),
        DeclareLaunchArgument("enable_pointcloud", default_value="false"),

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
