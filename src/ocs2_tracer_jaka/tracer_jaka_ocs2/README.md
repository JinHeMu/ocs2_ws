# tracer_jaka_ocs2

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix tracer_jaka_gazebo)/share

OCS2 全身 MPC 与 Tracer + JAKA 移动机械臂 Gazebo Harmonic 仿真的集成功能包。

## 1. 总体架构

```
                        ┌─────────────────────────────┐
                        │    tracer_jaka_gazebo       │
                        │  (Gazebo + ros2_control)    │
                        └───────────┬─────────────────┘
                                    │
            /diff_drive_controller/odom │  /joint_states
                                    │
                         ┌──────────▼──────────┐
                         │  tracer_jaka_mrt    │
                         │   (本包) Bridge     │
                         └──┬───────────┬──────┘
       SystemObservation    │           │  optimal state/input
                            │           │
                            ▼           │
                   ┌────────────────┐   │
                   │tracer_jaka_mpc │◄──┘ updatePolicy
                   │  (本包) SLQ    │
                   └────────▲───────┘
                            │ TargetTrajectories
                            │
              ┌─────────────┴────────────┐
              │  tracer_jaka_target_node │
              │  (本包) PoseStamped→TT   │
              └─────────────▲────────────┘
                            │
                       /target_pose  (RViz "2D Goal Pose")
```

| 节点 | 职责 |
|------|------|
| `tracer_jaka_mpc_node`     | OCS2 SLQ 求解器, 监听 observation, 发布 policy |
| `tracer_jaka_mrt_node`     | 把 odom + joint_states 拼成 SystemObservation; 把 policy 拆成 `TwistStamped` (cmd_vel) + `JointTrajectory` |
| `tracer_jaka_target_node`  | 把 RViz 点的目标位姿转成 OCS2 `TargetTrajectories` |

## 2. 状态/输入向量约定

`manipulatorModelType = 1` (`wheelBasedMobileManipulator`):

\[
x = [\,x_b,\; y_b,\; \theta_b,\; q_1,\; \dots,\; q_6\,]^\top \in \mathbb{R}^{9}
\]

\[
u = [\,v,\; \omega,\; \dot q_1,\; \dots,\; \dot q_6\,]^\top \in \mathbb{R}^{8}
\]

`TargetTrajectories` 的 desiredState 编码末端目标位姿 (size 7):

\[
x_\text{des} = [\,p_x,\; p_y,\; p_z,\; q_x,\; q_y,\; q_z,\; q_w\,]^\top
\]

## 3. 编译

依赖 (按 ROS 2 OCS2 移植版安装, 推荐 [legubiao/ocs2_ros2](https://github.com/legubiao/ocs2_ros2)):

```bash
cd ~/ocs2_tracer_jaka/src
# 把本包放到 src 下:
# tracer_jaka_ocs2/
# tracer_jaka_gazebo/
# ocs2_ros2/   <- 你的 OCS2 移植包
cd ..
colcon build --symlink-install --packages-up-to tracer_jaka_ocs2
source install/setup.bash
```

## 4. 必做事项 (踩坑提醒)

### 4.1 在 `config/task.info` 里填轮子 joint 名

`manipulatorModelType=1` 时 OCS2 不会建模轮子, 必须把所有车轮 / 转向 joint 列在 `removeJoints` 里, 否则 Pinocchio 会把它们当成 "arm DoF":

```
removeJoints {
  [0] "tracer_left_wheel"
  [1] "tracer_right_wheel"
  ; ...
}
```

具体名字打开你 `tracer_jaka.urdf.xacro` 看 transmission 那一段就是。

### 4.2 自检 `selfCollision.collisionLinkPairs` 里的 link 名

`task.info` 里写了 `base_link / urbase_base_link / Link_2..6 / gripper_base_link`, 这些必须在你 URDF 里真的存在 (而且要带 `<collision>` 几何), 否则 OCS2 会段错误。可以用:

```bash
xacro tracer_jaka.urdf.xacro | grep -E '<link name='
```

把不存在的对从 `task.info` 删掉。

### 4.3 关掉原 `gazebo.launch.py` 的双 RViz

目前你的 `gazebo.launch.py` 里 `rviz_node` 写的是 `condition=None`, 这导致 `use_rviz` 形同虚设。要么修一下:

```python
from launch.conditions import IfCondition
...
condition=IfCondition(use_rviz),
```

要么直接接受双 RViz, 在我提供的 `ocs2_sim.launch.py` 里我已经给原 launch 传了 `use_rviz:=false`, 但只有你修了上面那行才会真的关掉。

### 4.4 OCS2 codegen

第一次启动会调用 CppAD 自动生成微分 (`recompileLibraries=true`), 大概要 30s ~ 2min。生成结果在 `lib_folder` (默认 `/tmp/ocs2_tracer_jaka/auto_generated`)。第二次起就秒启。如果改了 task.info 里影响动力学的参数 (e.g. baseFrame, eeFrame), 把这个目录删了重来。

## 5. 跑起来

### 一键全部启动

```bash
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py
```

### 分两个终端 (调试时方便看 log)

```bash
# T1
ros2 launch tracer_jaka_gazebo gazebo.launch.py use_rviz:=false

# T2 (等机器人 spawn 完, 控制器都加载完)
ros2 launch tracer_jaka_ocs2 ocs2_only.launch.py
```

## 6. 给 MPC 发目标

### 用 RViz 的 "2D Goal Pose" 工具

在 `tracer_jaka_ocs2.rviz` 里 `SetGoal` 工具的 topic 已经设成 `/target_pose`, 直接点击拖一下就发。注意 RViz 的 2D Goal 高度是 0, 你要让末端到 0.6m 高就需要先用脚本试:

### 用脚本

```bash
ros2 run tracer_jaka_ocs2 send_target.py 0.8 0.0 0.6 0 0 0
#                                          x   y   z   r p y
```

参数依次为 `x y z roll pitch yaw` (rad, frame=`odom`)。

### 验证

```bash
# observation 应该 ~100 Hz
ros2 topic hz /mobile_manipulator_mpc_observation

# policy 应该 ~100 Hz (mpcDesiredFrequency)
ros2 topic hz /mobile_manipulator_mpc_policy

# 输出
ros2 topic echo /diff_drive_controller/cmd_vel
ros2 topic echo /jaka_arm_controller/joint_trajectory
```

## 7. 关键参数 (在 `launch/ocs2_sim.launch.py` 里改)

| 参数 | 默认 | 说明 |
|------|------|------|
| `mrt_loop_rate`     | 100 Hz | MRT 控制环频率 (= cmd_vel 发布频率) |
| `traj_horizon`      | 0.1 s  | 给 JTC 的轨迹采样时窗 |
| `traj_num_points`   | 5      | 时窗内采样点数 |
| `odom_topic`        | `/diff_drive_controller/odom` | 底盘位姿来源 |
| `joint_state_topic` | `/joint_states`               | 关节读数来源 |
| `base_cmd_topic`    | `/diff_drive_controller/cmd_vel` | TwistStamped |
| `arm_cmd_topic`     | `/jaka_arm_controller/joint_trajectory` | JTC |

## 8. 调权重

效果不好时常改这几个 (`config/task.info`):

- `endEffector.muPosition / muOrientation` - 越大跟踪越凶, 但容易颤
- `inputCost.R.base.wheelBasedMobileManipulator.scaling` - 越大 base 越懒得动 (会让机械臂自己够)
- `inputCost.R.arm.scaling` - 越大手臂越懒得动 (会让 base 去够)

## 9. 文件清单

```
tracer_jaka_ocs2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── task.info                # OCS2 配置 (你给的, 加了 removeJoints 注释)
├── launch/
│   ├── ocs2_sim.launch.py       # Gazebo + OCS2 全套
│   └── ocs2_only.launch.py      # 仅 OCS2 (Gazebo 已在跑)
├── rviz/
│   └── tracer_jaka_ocs2.rviz
├── scripts/
│   └── send_target.py           # 命令行发目标
└── src/
    ├── TracerJakaMpcNode.cpp    # MPC 求解器
    ├── TracerJakaMrtNode.cpp    # MRT + Gazebo 桥
    └── TracerJakaTargetNode.cpp # PoseStamped → TargetTrajectories
```

## 10. 跟你现有手动指令的等价关系

| 之前你手动发的 | OCS2 接管后由谁发 |
|----|----|
| `/diff_drive_controller/cmd_vel` (TwistStamped) | `tracer_jaka_mrt_node` 自动发 |
| `/jaka_arm_controller/joint_trajectory`         | `tracer_jaka_mrt_node` 自动发 |
| (新增) 末端目标位姿                              | 你点 RViz / 调 `send_target.py` |

也就是说, 一旦 OCS2 跑起来, 你就 **不要再手动 pub `cmd_vel` / `joint_trajectory`** 了, 否则会和 MRT 抢 topic。
