# ocs2_ws

model predictive control of mobile_manipulator

## build

```bash
colcon build   --packages-up-to tracer_jaka_ocs2  tracer_jaka_mujoco --symlink-install   --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## run

```bash
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py
```
