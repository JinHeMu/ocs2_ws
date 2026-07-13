# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # -------------------------
    # Launch arguments
    # -------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lidar = LaunchConfiguration('use_lidar')
    global_frame = LaunchConfiguration('global_frame')
    map_clearing_frame_id = LaunchConfiguration('map_clearing_frame_id')
    log_level = LaunchConfiguration('log_level')

    # -------------------------
    # nvblox config files
    # -------------------------
    nvblox_examples_dir = get_package_share_directory('nvblox_examples_bringup')

    nvblox_base_config = os.path.join(
        nvblox_examples_dir,
        'config',
        'nvblox',
        'nvblox_base.yaml'
    )

    # -------------------------
    # Remap your robot topics to nvblox expected topics
    # -------------------------
    remappings = [
        # Depth camera
        ('camera_0/depth/image', '/camera/depth/image_raw'),
        ('camera_0/depth/camera_info', '/camera/depth/camera_info'),

        # Color camera
        ('camera_0/color/image', '/camera/color/image_raw'),
        ('camera_0/color/camera_info', '/camera/color/camera_info'),

        # 3D lidar
        ('pointcloud', '/lidar/points'),
    ]

    # -------------------------
    # nvblox node
    # -------------------------
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=[
            nvblox_base_config,

            {
                # 你的话题里有 /clock，所以默认 use_sim_time:=True
                # 如果是真实硬件且没有使用仿真时间，启动时传 use_sim_time:=False
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),

                # 单个 RGB-D 相机
                'num_cameras': 1,

                # 使用深度图和彩色图
                'use_depth': True,
                'use_color': True,

                # 是否融合 /lidar/points
                'use_lidar': ParameterValue(use_lidar, value_type=bool),

                # 使用 TF，不直接使用 /base_controller/odom topic
                # 你需要保证 TF 里存在 global_frame -> camera_depth_frame 的变换链
                'use_tf_transforms': True,
                'use_topic_transforms': False,

                # 常用设置：odom 作为建图全局坐标系
                'global_frame': global_frame,

                # 地图清理参考坐标系，一般是 base_link
                'map_clearing_frame_id': map_clearing_frame_id,

                # 静态环境建图
                'mapping_type': 'static_tsdf',

                # 建议先打开，方便看终端有没有收到数据
                'print_rates_to_console': True,
                'print_timings_to_console': False,
                'print_delays_to_console': False,
            }
        ],
    )

    # -------------------------
    # Component container
    # -------------------------
    nvblox_container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[
            nvblox_node,
        ],
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level',
            log_level,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation time. Your topic list contains /clock, so default is True.'
        ),

        DeclareLaunchArgument(
            'use_lidar',
            default_value='True',
            description='Fuse /lidar/points into nvblox. Set False to use RGB-D camera only.'
        ),

        DeclareLaunchArgument(
            'global_frame',
            default_value='odom',
            description='Global frame used by nvblox.'
        ),

        DeclareLaunchArgument(
            'map_clearing_frame_id',
            default_value='base_link',
            description='Frame around which nvblox clears the local map.'
        ),

        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level: debug, info, warn, error.'
        ),

        nvblox_container,
    ])
