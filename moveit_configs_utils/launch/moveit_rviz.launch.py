from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from moveit_config_core import (
    add_debuggable_node,
    get_planning_parameters_config,
    DeclareBooleanLaunchArg,
)


def generate_launch_description():
    core_config_path = get_package_share_directory("moveit_config_core")

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("moveit_config_path"))  # Required argument
    moveit_config_path = LaunchConfiguration("moveit_config_path")

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config", default_value=str(core_config_path / "config/moveit.rviz")
        )
    )

    rviz_parameters = [
        {
            "robot_description_planning": get_planning_parameters_config(
                core_config_path
            )
        },
        str(moveit_config_path / "config/kinematics.yaml"),
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=True,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    return ld
