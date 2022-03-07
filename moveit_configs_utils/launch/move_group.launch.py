from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration

from launch_ros.parameter_descriptions import ParameterValue

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

    ld.add_action(
        DeclareLaunchArgument("relative_srdf_path")
    )  # Required arg - path relative to moveit_config_path

    ld.add_action(DeclareBooleanLaunchArg("debug"), default_value=False)

    ld.add_action(
        DeclareLaunchArgument(
            "pipeline",
            default_value="ompl",
        )
    )

    ld.add_action(
        DeclareLaunchArgument("allow_trajectory_execution", default_value="true")
    )
    ld.add_action(DeclareLaunchArgument("fake_execution", default_value="false"))
    ld.add_action(DeclareLaunchArgument("max_safe_path_cost", default_value="1"))
    ld.add_action(DeclareLaunchArgument("jiggle_fraction", default_value="0.05"))
    ld.add_action(
        DeclareLaunchArgument("publish_monitored_planning_scene", default_value="true")
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    # Load the SRDF
    srdf_contents = ParameterValue(
        Command(
            [
                "xacro ",
                moveit_config_path,
                "/",
                LaunchConfiguration("relative_srdf_path"),
            ]
        ),
        value_type=str,
    )

    kinematics_config = [moveit_config_path, "/config/kinematics.yaml"]
    sensors_config = [moveit_config_path, "/config/sensors_3d.yaml"]
    trajectory_config = [moveit_config_path, "/config/trajectory_execution.yaml"]
    joint_limits_config = [moveit_config_path, "/config/joint_limits.yaml"]
    controller_config = [moveit_config_path, "/config/ros_controllers.yaml"]

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "robot_description_semantic": srdf_contents,
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "max_safe_path_cost": LaunchConfiguration("max_safe_path_cost"),
        "jiggle_fraction": LaunchConfiguration("jiggle_fraction"),
        "capabilities": LaunchConfiguration("capabilities"),
        "disable_capabilities": LaunchConfiguration("disable_capabilities"),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
    }

    move_group_params = [
        get_planning_parameters_config(core_config_path),
        kinematics_config,
        # sensors_config,
        trajectory_config,
        joint_limits_config,
        controller_config,
        move_group_configuration,
    ]
    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )
    return ld
