from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


class DeclareBooleanLaunchArg(DeclareLaunchArgument):
    def __init__(self, name, *, default_value=False, description=None, **kwargs):
        super().__init__(
            name,
            default_value=str(default_value),
            description=description,
            choices=["true", "false", "True", "False"],
            **kwargs,
        )


def add_debuggable_node(
    ld, package, executable, condition_name="debug", commands_file=None, **kwargs
):
    standard_node = Node(
        package=package,
        executable=executable,
        condition=UnlessCondition(LaunchConfiguration(condition_name)),
        **kwargs,
    )
    ld.add_action(standard_node)

    if commands_file:
        dash_x_arg = f"-x {commands_file}"
    else:
        dash_x_arg = ""
    prefix = [f"gdb {dash_x_arg} --ex run --args"]

    debug_node = Node(
        package=package,
        executable=executable,
        condition=IfCondition(LaunchConfiguration(condition_name)),
        prefix=prefix,
        **kwargs,
    )
    ld.add_action(debug_node)
