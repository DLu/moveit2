from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("robot_description_path"))  # Required argument
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))

    # Load the URDF
    robot_description_content = ParameterValue(
        Command(["xacro ", LaunchConfiguration("robot_description_path")]),
        value_type=str,
    )

    # Given the published joint states, publish tf for the robot links and the robot description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_content,
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            }
        ],
    )
    ld.add_action(rsp_node)

    return ld
