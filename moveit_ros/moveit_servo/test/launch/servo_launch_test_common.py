import os
import yaml
import launch
from launch import LaunchDescription
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch_testing.actions
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_servo_test_description(*args,
                                    gtest_name: SomeSubstitutionsType,
                                    start_position_path: SomeSubstitutionsType = ''):

    # Get parameters using the demo config file
    servo_yaml = load_yaml('moveit_servo', 'config/panda_simulated_config.yaml')
    servo_params = { 'moveit_servo' : servo_yaml }

    # Get URDF and SRDF
    robot_description_config = xacro.process_file(os.path.join(get_package_share_directory('moveit_resources_panda_moveit_config'),
                                                               'config',
                                                               'panda.urdf.xacro'),
                                                  mappings={'initial_positions_file': os.path.join(os.path.dirname(__file__), start_position_path)})
    robot_description = {'robot_description' : robot_description_config.toxml()}

    robot_description_semantic_config = load_file('moveit_resources_panda_moveit_config', 'config/panda.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    # Fake ros2_controller system
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description,  os.path.join(get_package_share_directory("moveit_resources_panda_moveit_config"), "config", "panda_ros_controllers.yaml")],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )


    # load joint_state_controller
    load_joint_state_controller = ExecuteProcess(cmd=['ros2 control load_start_controller joint_state_controller'], shell=True, output='screen')
    load_controllers = [load_joint_state_controller]
    # load panda_arm_controller
    load_controllers += [ExecuteProcess(cmd=['ros2 control load_configure_controller panda_arm_controller'],
                                        shell=True,
                                        output='screen',
                                        on_exit=[ExecuteProcess(cmd=['ros2 control switch_controllers --start-controllers panda_arm_controller'],
                                                                shell=True,
                                                                output='screen')])]

    # Component nodes for tf and Servo
    test_container = ComposableNodeContainer(
            name='test_servo_integration_container',
            namespace='/',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='robot_state_publisher',
                    plugin='robot_state_publisher::RobotStatePublisher',
                    name='robot_state_publisher',
                    parameters=[robot_description]),
                ComposableNode(
                    package='tf2_ros',
                    plugin='tf2_ros::StaticTransformBroadcasterNode',
                    name='static_tf2_broadcaster',
                    parameters=[ {'/child_frame_id' : 'panda_link0', '/frame_id' : 'world'} ]),
                ComposableNode(
                    package='moveit_servo',
                    plugin='moveit_servo::ServoServer',
                    name='servo_server',
                    parameters=[servo_params, robot_description, robot_description_semantic],
                    extra_arguments=[{'use_intra_process_comm': True}])
            ],
            output='screen',
    )

    servo_gtest = launch_testing.actions.GTest(
        path=PathJoinSubstitution([LaunchConfiguration('test_binary_dir'), gtest_name]),
        timeout=40.0, output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='test_binary_dir',
                                             description='Binary directory of package '
                                                         'containing test executables'),
        ros2_control_node,
        test_container,
        servo_gtest,
        launch_testing.actions.ReadyToTest()
    ] + load_controllers), {'test_container': test_container, 'servo_gtest': servo_gtest, 'ros2_control_node': ros2_control_node}
