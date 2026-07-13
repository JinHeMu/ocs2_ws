import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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

    # ============ 夹爪启动参数 ============
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

    # ============ MoveIt 配置 ============
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

    # Servo 参数
    servo_yaml = load_yaml("jaka_driver", "config/jaka_servo_params.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # RViz
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

    # ros2_control
    ros2_controllers_path = os.path.join(
        get_package_share_directory("jaka_driver"),
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
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    jaka_arm_controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    jaka_servo_controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["servo_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    jaka_fts_broadcaster_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["fts_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # delay_jtc_after_admittance = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=jaka_admittance_controller_spawner,
    #         on_exit=[jaka_arm_controller_spawner],
    #     )
    # )

    # ============ DH AG95 夹爪驱动节点 ============
    # 不再单独起 robot_state_publisher / RViz，夹爪的 TF 由主 RSP 统一发布。
    # 把驱动的 /joint_states 直接发给主话题，夹爪关节名与机械臂不同，互不干扰。
    dh_ag95_driver_node = Node(
        package='dh_gripper_driver',
        executable='dh_ag95_driver',
        name='dh_ag95_driver',
        parameters=[{
            'device_port': LaunchConfiguration('device_port'),
            'baudrate':    LaunchConfiguration('baudrate'),
            'gripper_id':  LaunchConfiguration('gripper_id'),
            'max_position': 100.0,   # AG95 最大开口 100mm
            'max_force':    100.0,   # AG95 最大夹持力 100N
        }],
        # 不再 remap 到 /gripper/joint_states —— 让它直接进 /joint_states
        output='screen'
    )

    # ============ Servo 组件容器（内含手柄节点） ============
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            ComposableNode(
                package="jaka_driver",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription([
        # 启动参数
        device_port_arg,
        baudrate_arg,
        gripper_id_arg,
        # 可视化 & MoveIt
        rviz_node,
        move_group_node,
        # ros2_control
        #ros2_control_node,
        #joint_state_broadcaster_spawner,
        # jaka_fts_broadcaster_spawner,
        # jaka_admittance_controller_spawner,
        # delay_jtc_after_admittance,
        # jaka_arm_controller_spawner,
        # jaka_servo_controller_spawner,
        # Servo + 手柄
        servo_node,
        container,
        # 夹爪驱动
        #dh_ag95_driver_node,
    ])

