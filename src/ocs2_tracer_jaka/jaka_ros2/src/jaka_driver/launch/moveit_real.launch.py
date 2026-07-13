import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():

    device_port_arg = DeclareLaunchArgument(
        'device_port',
        default_value='/dev/ttyUSB0',
        description='Serial device port for DH AG95 gripper'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for serial communication'
    )
    gripper_id_arg = DeclareLaunchArgument(
        'gripper_id',
        default_value='1',
        description='Gripper ID for communication'
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="tracer_jaka",
            package_name="tracer_jaka_moveit_config",
        )
        .robot_description(file_path="config/tracer_jaka_zu5.urdf.xacro")
        .robot_description_semantic(file_path="config/tracer_jaka_zu5.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("tracer_jaka_moveit_config"),
        "config", "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        output="log",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("tracer_jaka_moveit_config"),
        "config", "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    jaka_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jaka_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    jaka_fts_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jaka_fts_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    dh_ag95_driver_node = Node(
        package='dh_gripper_driver',
        executable='dh_ag95_driver',
        name='dh_ag95_driver',
        parameters=[{
            'device_port': LaunchConfiguration('device_port'),
            'baudrate':    LaunchConfiguration('baudrate'),
            'gripper_id':  LaunchConfiguration('gripper_id'),
            'max_position': 100.0,
            'max_force':    100.0,
        }],
        output='screen'
    )

    return LaunchDescription([
        device_port_arg,
        baudrate_arg,
        gripper_id_arg,

        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        jaka_fts_broadcaster_spawner,
        jaka_arm_controller_spawner,

        move_group_node,
        rviz_node,

        #dh_ag95_driver_node,
    ])
