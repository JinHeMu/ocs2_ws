from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="esdf_simple_nav",
                executable="simple_nav",
                name="esdf_simple_navigator",
                output="screen",
                parameters=[
                    {
                        "map_topic": "/esdf_occ2d",
                        "goal_topic": "/goal_pose",
                        "path_topic": "/planned_path",
                        "cmd_vel_topic": "/cmd_vel",
                        "map_frame": "map",
                        "base_frame": "base_link",
                        "inflation_radius": 0.25,
                        "lookahead_distance": 0.40,
                        "max_linear_speed": 0.30,
                        "max_angular_speed": 1.00,
                    }
                ],
            )
        ]
    )
