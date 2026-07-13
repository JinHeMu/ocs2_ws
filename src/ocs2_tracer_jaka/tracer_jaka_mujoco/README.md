# tracer_jaka_mujoco

ROS2 ⟷ MuJoCo 自包含桥接：平面 3 自由度底盘 + JAKA Zu5 机械臂 + **LiDAR** + **深度/RGB 相机**。
传感器频率与 500Hz 物理步进**解耦**，按需抽帧，避免仿真卡顿。

## 1. 目录结构

```
tracer_jaka_mujoco/
├── package.xml
├── setup.py / setup.cfg
├── resource/tracer_jaka_mujoco
├── config/sensors.yaml          # 所有参数都在这里改
├── launch/bridge.launch.py
├── models/                      # 放 scene.xml / tracer_jaka_zu5_robot.xml / meshes/（自己拷进来）
├── urdf/                        # 可选：tracer_jaka_zu5.urdf（给 robot_state_publisher）
└── tracer_jaka_mujoco/
    ├── mujoco_bridge_node.py    # 主节点
    ├── lidar_sensor.py          # LiDAR（独立线程）
    ├── camera_sensor.py         # 相机（主线程渲染 + 独立线程发布）
    └── sensors_common.py        # 公用工具
```

## 2. 依赖

```bash
pip install mujoco mujoco_lidar numpy
# taichi 后端(GPU)再装: pip install taichi
```

`mujoco_lidar` 的 `lidar_site` 与相机 `d435_depth` 已在你的 `tracer_jaka_zu5_robot.xml` 里。

## 3. 放模型

把场景文件拷到 `models/`，并确保 `scene.xml` 里 `meshdir` 指向真实网格目录：

```
models/scene.xml
models/tracer_jaka_zu5_robot.xml
models/meshes/...                # 或保持 XML 中的绝对 meshdir
```

> LiDAR 要有“东西可打”，所以 **model 必须指向带墙体/桌子/障碍的 `scene.xml`**，而不是只含机器人的文件。

## 4. 编译运行

```bash
cd ~/ros2_ws && colcon build --packages-select tracer_jaka_mujoco
source install/setup.bash

ros2 launch tracer_jaka_mujoco bridge.launch.py
# 指定模型 / 无头：
ros2 launch tracer_jaka_mujoco bridge.launch.py model:=/abs/path/scene.xml viewer:=false
```

## 5. 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/lidar/points` | `sensor_msgs/PointCloud2` | 帧 `lidar_link`，雷达局部坐标 |
| `/camera/color/image_raw` | `sensor_msgs/Image` (rgb8) | |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (32FC1, m) | |
| `/camera/{color,depth}/camera_info` | `sensor_msgs/CameraInfo` | |
| `/joint_states` `/base_controller/odom` `/clock` `/tf` | | 底盘/臂状态 |

控制：

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear:{x:0.2}, angular:{z:0.3}}"
ros2 topic pub /arm_controller/commands std_msgs/Float64MultiArray "{data:[0,0.5,1.0,0,0.5,0]}"
```

## 6. 改参数 / 改频率（`config/sensors.yaml`）

- **雷达频率**：`lidar.rate`（默认 25Hz）
- **相机帧率**：`camera.rate`（默认 15Hz）
- 雷达分辨率：`num_ray_cols` / `num_ray_rows`（或 `pattern: HDL64` 等命名模式）
- 雷达测距：`cutoff_dist`；后端：`backend: cpu|taichi`
- 相机分辨率/FOV：`width` `height` `fovy`（`fovy` 要与 XML 中相机一致）
- 深度上限：`max_depth`，超出是否置 0：`clip_far_to_zero`

> 物理仍是 500Hz（由 `model.opt.timestep` 决定），但传感器按上面的 `rate` 抽帧。
> 想让物理也更轻，可在 MJCF `<option timestep="0.004"/>`（=250Hz）。

## 7. 坐标系（TF）

桥接直接发布：
```
odom -> base_footprint                       (里程)
base_footprint -> lidar_link                 (静态，雷达固连底盘)
base_footprint -> camera_depth_optical_frame (动态，相机在臂末端，随臂运动)
```
相机已做 MuJoCo 相机系→ROS 光学系（x 右 / y 下 / z 朝前）的转换，深度图可直接生成点云。
这些传感器帧是 `base_footprint` 的**直接子节点**，与 `robot_state_publisher` 发的臂 link TF 不冲突。

## 8. 关于 OpenGL（重要）

相机离屏渲染与 MuJoCo 查看器都用 OpenGL。本包把**渲染放在持有查看器上下文的主线程**里（按 `camera.rate` 抽帧），规避多线程 GL 冲突。若仍报 GL 错误：

- 纯无头高帧率：`export MUJOCO_GL=egl` 并 `viewer:=false`
- 只调底盘/臂、不要相机：`camera.enable: false`
- LiDAR 不用 GL，任何模式都安全。

## 9. 设计要点

- `physics_lock` 包住 `mj_step`；传感器线程仅在锁内**快照** `MjData`，锁外做光追/渲染，物理不被长时间阻塞。
- LiDAR 在独立线程；相机渲染在主线程、消息打包+发布在独立线程（队列只留最新帧）。
- 底盘 `exact_base=true` 为纯运动学跟随，零稳态偏差（与原方案一致）。
