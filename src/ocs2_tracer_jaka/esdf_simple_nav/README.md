# esdf_simple_nav

Minimal ROS 2 navigation pipeline:

- subscribes: `/esdf_occ2d` (`nav_msgs/OccupancyGrid`)
- subscribes: `/goal_pose` (`geometry_msgs/PoseStamped`)
- reads robot pose from TF: `map -> base_link`
- plans with 8-connected A*
- publishes: `/planned_path` (`nav_msgs/Path`)
- tracks path with Pure Pursuit
- publishes: `/cmd_vel` (`geometry_msgs/Twist`)

## Build

```bash
cd ~/ros2_ws/src
unzip /path/to/esdf_simple_nav.zip
cd ~/ros2_ws
colcon build --symlink-install --packages-select esdf_simple_nav
source install/setup.bash
```

## Run

First start your ESDF map publisher, robot driver, odometry/localization, and TF tree.

Then:

```bash
ros2 launch esdf_simple_nav simple_nav.launch.py
```

Check:

```bash
ros2 topic echo /esdf_occ2d --once
ros2 topic echo /goal_pose
ros2 topic echo /planned_path
ros2 topic echo /cmd_vel
ros2 run tf2_ros tf2_echo map base_link
```

In RViz:

1. Fixed Frame = `map`
2. Add `Map`, topic `/esdf_occ2d`
3. Add `Path`, topic `/planned_path`
4. Use **2D Goal Pose** and set its topic to `/goal_pose`

## Important

This is a compact educational/custom navigation stack for a mostly static map.
It does not include dynamic-obstacle avoidance, recovery behaviors, collision
monitoring, behavior trees, or localization. For production mobile-robot use,
Nav2 is generally the stronger architecture.
