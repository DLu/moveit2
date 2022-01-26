from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from moveit_config_core import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_launch_description():
    ld = LaunchDescription()

    # Required Argument
    ld.add_action(DeclareLaunchArgument("config_package_name",
                                        description="The name of the package containing the MoveIt config"))
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    add_debuggable_node(
        ld,
        package="moveit_setup_assistant",
        executable="moveit_setup_assistant",
        arguments=[['--config_pkg=', LaunchConfiguration("config_package_name")]],
    )

    return ld
