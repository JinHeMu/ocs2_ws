
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def replace_package_path_in_urdf(urdf_path, package_path):
    with open(urdf_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()
    urdf_content = urdf_content.replace('package://dh_ag_95', package_path)
    temp_urdf_path = '/tmp/temp_dh_ag_95.urdf'
    with open(temp_urdf_path, 'w') as temp_urdf_file:
        temp_urdf_file.write(urdf_content)
    return temp_urdf_path



def generate_launch_description():
    robot_name_in_model = 'dh_ag_95'
    package_name = 'dh_ag_95'
    urdf_name = "dh_ag_95.urdf"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    mesh_model_path = os.path.join(pkg_share, 'meshes')  # 模型文件路径

    # 替换 URDF 中的路径为绝对路径
    temp_urdf_path = replace_package_path_in_urdf(urdf_model_path, pkg_share)

    # Start Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-file', temp_urdf_path],
        output='screen')

    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)

    return ld
