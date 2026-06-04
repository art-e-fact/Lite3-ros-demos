from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _simulation_arguments(config_path: str, overrides_text: str):
    arguments = []
    if config_path:
        arguments.extend(["--config", config_path])

    for item in overrides_text.split(";"):
        override = item.strip()
        if override:
            arguments.extend(["--set", override])
    return arguments


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("config").perform(context).strip()
    simulation_overrides = LaunchConfiguration("simulation_overrides").perform(context).strip()
    arguments = _simulation_arguments(config_path, simulation_overrides)

    return [
        Node(
            package="simulation_package",
            executable="start_simulation",
            name="simulation",
            output="screen",
            arguments=arguments,
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=""),
        DeclareLaunchArgument("simulation_overrides", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])