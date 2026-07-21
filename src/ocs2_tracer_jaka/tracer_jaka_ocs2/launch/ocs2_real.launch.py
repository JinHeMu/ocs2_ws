#!/usr/bin/env python3
# =============================================================================
#  ocs2_real.launch.py
#
#  实机 launch (替换原 ocs2_sim.launch.py 中的 Gazebo 部分):
#    1. xacro -> /tmp/.../tracer_jaka_real.urdf  (real_robot:=true 把 ros2_control
#       插件块切到 jaka_hardware_interface)
#    2. robot_state_publisher
#    3. tracer_base (CAN 节点, 接 /cmd_vel, 发 /odom 与 odom->base_link TF)
#    4. ros2_control_node + joint_state_broadcaster + jaka_arm_controller
#    5. OCS2 三件套 (mpc / mrt / target):
#         use_sim_time          : false
#         use_stamped_cmd       : false      ← 让 MRT 发 Twist 给 tracer_base
#         base_cmd_topic        : /cmd_vel
#         odom_topic            : /odom
#         base_frame            : base_footprint
#    6. RViz2
# =============================================================================

import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    pkg_ocs2     = FindPackageShare('tracer_jaka_ocs2')
    pkg_mujoco     = FindPackageShare('tracer_jaka_mujoco')   # 你放 URDF/xacro 的包
    pkg_tracer   = FindPackageShare('tracer_base')

    # --------- 参数 ---------
    use_rviz   = LaunchConfiguration('use_rviz')
    task_file  = LaunchConfiguration('task_file')
    urdf_file  = LaunchConfiguration('urdf_file')
    lib_folder = LaunchConfiguration('lib_folder')
    can_port   = LaunchConfiguration('can_port')
    robot_ip   = LaunchConfiguration('robot_ip')
    local_ip   = LaunchConfiguration('local_ip')


    declare_args = [
        DeclareLaunchArgument('use_rviz',  default_value='true'),
        DeclareLaunchArgument('can_port',  default_value='can0'),
        DeclareLaunchArgument('robot_ip',  default_value='192.168.0.100'),
        DeclareLaunchArgument('local_ip',  default_value='192.168.0.10'),
        DeclareLaunchArgument(
            'task_file',
            default_value=PathJoinSubstitution(
                [pkg_ocs2, 'config', 'task_real.info'])),
        DeclareLaunchArgument(
            # 名字沿用你原来的 xacro_file，但现在默认指向 MuJoCo 包里的 .urdf
            "urdf_file",
            default_value=PathJoinSubstitution(
                [pkg_mujoco, "urdf", "tracer_jaka_zu5_real.urdf"]
            ),
        ),
        DeclareLaunchArgument(
            'lib_folder',
            default_value='/tmp/ocs2_tracer_jaka_real/auto_generated'),
        DeclareLaunchArgument('use_joy',    default_value='true'),
        DeclareLaunchArgument('joy_device', default_value='/dev/input/js0'),
    ]


    # --------- 2. robot_state_publisher ---------
    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ',
            urdf_file, ' ',
            'sim_mode:=false ',
        ])
    }
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
    )

    # --------- 3. tracer 底盘 (CAN) ---------
    tracer_base = Node(
        package='tracer_base',
        executable='tracer_base_node',     # 视你的 CMakeLists 实际可执行名替换
        name='tracer_base_node',
        output='screen',
        parameters=[{
            'port_name':       can_port,
            'odom_frame':      'odom',
            'base_frame':      'base_footprint',  # 与 OCS2 / URDF 对齐
            'odom_topic_name': 'odom',
            'is_tracer_mini':  False,
            'simulated_robot': False,
            'control_rate':    50,
        }],
    )

    # --------- 4. controller_manager + spawners ---------
    controllers_yaml = PathJoinSubstitution(
        [pkg_ocs2, 'config', 'tracer_jaka_controllers_real.yaml'])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml,
                    {'use_sim_time': False}],
        output='screen',
        remappings=[
            # ros2_control_node 默认从 /robot_description 读 URDF, 上面已经塞参数
        ],
    )

    spawn_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )
    spawn_jtc = Node(
        package='controller_manager', executable='spawner',
        arguments=['jaka_arm_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 控制器顺序: jsb 起来后再起 jtc, 避免 jtc 找不到 state interface
    spawn_jtc_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_jtc]))

    # --------- 5. OCS2 三件套 ---------
    common_ocs2 = {
        'taskFile':     task_file,
        'urdfFile':     urdf_file,
        'libFolder':    lib_folder,
        'use_sim_time': False,
    }

    mpc_node = Node(
        package='tracer_jaka_ocs2',
        executable='tracer_jaka_mpc_node',
        name='tracer_jaka_mpc_node',
        output='screen',
        parameters=[common_ocs2],
    )

    mrt_node = Node(
        package='tracer_jaka_ocs2',
        executable='tracer_jaka_mrt_node',
        name='tracer_jaka_mrt_node',
        output='screen',
        parameters=[{
            **common_ocs2,
            'mrt_loop_rate':     100.0,
            'traj_horizon':      0.10,                 # 实机给宽一点
            'use_stamped_cmd':   False,                # *** 关键: 发 Twist ***
            'base_cmd_topic':    '/cmd_vel',           # *** tracer_base 订这个 ***
            'odom_topic':        '/odom',              # *** tracer_base 发这个 ***
            'joint_state_topic': '/joint_states',
            'arm_cmd_topic':     '/jaka_forward_controller/commands',
            'arm_joint_names':   ['joint_1','joint_2','joint_3',
                                  'joint_4','joint_5','joint_6'],
            'base_frame':        'base_footprint',
            'world_frame':       'odom',
            'ee_frame':          'tool0',
        }],
    )

    target_node = Node(
        package='tracer_jaka_ocs2',
        executable='tracer_jaka_target_node',
        name='tracer_jaka_target_node',
        output='screen',
        parameters=[{
            'robot_name':    'mobile_manipulator',
            'marker_frame':  'odom',
            'ee_frame':      'tool0',
            'marker_scale':  0.3,
            'input_dim':     8,
            'use_sim_time':  False,
        }],
    )

    # --------- 6. RViz2 ---------
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', PathJoinSubstitution(
            [pkg_ocs2, 'rviz', 'tracer_jaka_ocs2.rviz'])],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=None,   # 如果你想用 IfCondition(use_rviz), 这里加上
    )

    use_joy    = LaunchConfiguration('use_joy')
    joy_device = LaunchConfiguration('joy_device')

    # 手柄驱动: 读 /dev/input/jsX, 出 /joy
    joy_driver = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {
                "device_id": 0,
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
                "use_sim_time": False,
            }
        ],
        condition=IfCondition(use_joy),
        output="screen",
    )


    # 手柄 -> OCS2 目标位姿节点
    joy_target_node = Node(
        package='tracer_jaka_ocs2',
        executable='tracer_jaka_joy_target_node',
        name='tracer_jaka_joy_target_node',
        output='screen',
        parameters=[{
            'robot_name':    'mobile_manipulator',
            'marker_frame':  'odom',
            'ee_frame':      'tool0',
            'input_dim':     8,
            'joy_topic':     '/joy',
            'publish_rate':  50.0,
            'linear_speed':  0.15,
            'angular_speed': 0.6,
            'deadzone':      0.10,
            'use_sim_time':  False,           # sim launch 里改成 use_sim_time
            # 如果是 PS4/PS5 手柄, 这里覆盖默认的轴/键映射即可
            # 'axis_x': 1, 'axis_y': 0, 'axis_z': 4, 'axis_yaw': 3,
            # 'button_deadman': 4, 'button_reset': 0, 'button_home': 1,
        }],
        condition=IfCondition(use_joy),
    )

        # 手柄控制底盘全身轨迹 (左摇杆前进/后退, 右摇杆转向)
    # "胡萝卜" 模式: 目标 = 当前位置 + 速度 * lookahead_time
    # 松开 LB: 底盘保持位置, 手臂→home (MPC 主动对抗重力)
    joy_whole_body_node = Node(
        package="tracer_jaka_ocs2",
        executable="tracer_jaka_joy_whole_body_node",
        name="tracer_jaka_joy_whole_body_node",
        output="screen",
        parameters=[
            {
                "robot_name": "mobile_manipulator",
                "world_frame": "odom",
                "publish_rate": 50.0,
                "linear_speed_max": 0.4,       # 最大线速度 [m/s]
                "angular_speed_max": 1.0,      # 最大角速度 [rad/s]
                "deadzone": 0.10,
                "lookahead_time": 1.5,          # 胡萝卜前视距离 [s]
                "trajectory_horizon": 2.0,      # 轨迹总时长 [s]
                "lead_time": 0.05,
                "num_waypoints": 5,             # 航点数 (显式编码速度)
                "state_dim": 9,
                "input_dim": 8,
                "base_dim": 3,
                "arm_dim": 6,
                "joy_topic": "/joy",
                # 手柄映射
                "axis_linear": 1,       # 左摇杆 Y → 前进/后退
                "axis_angular": 3,      # 右摇杆 X → 转向
                "button_deadman": 4,     # LB → 安全开关
                "button_arm_home": 0,    # A → 臂归 home
                "button_arm_hold": 1,    # B → 臂保持当前构型
                # 机械臂 home 位姿
                "arm_home": [0.0, 1.5707,0.0, 1.5707, 3.14159, 0.785398],
                "use_sim_time": False,
            }
        ],
        condition=IfCondition(use_joy),
    )

    map_to_odom_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="map_to_odom_static_tf",
    arguments=[
        "-2", "0", "0",     # x y z
        "0", "0", "0",     # yaw pitch roll
        "map",
        "odom",
    ],
    output="screen",
    )

    # --------- 时序 ---------
    # 时序原则:
    #   t=0    : xacro, rsp, tracer_base, controller_manager 同时起
    #   t=2s   : spawn jsb (要等 ros2_control_node 先起来)
    #   jsb退出: spawn jtc (RegisterEventHandler 串起来)
    #   t=4s   : rviz
    #   t=10s  : 等 odom + joint_states + JTC 都活了, 再起 OCS2 三件套
    rviz_delayed = TimerAction(period=4.0, actions=[rviz_node])
    spawn_delayed = TimerAction(period=2.0, actions=[spawn_jsb])
    ocs2_delayed = TimerAction(
        period=10.0,
        actions=[mpc_node, mrt_node, joy_driver, joy_whole_body_node])

    return LaunchDescription(declare_args + [
        rsp,
        tracer_base,
        controller_manager,
        spawn_delayed,
        spawn_jtc_after_jsb,
        ocs2_delayed,
        map_to_odom_tf,
        rviz_delayed,
    ])

