# DH AG95 ROS2 驱动与描述包

本仓库为 DH Robotics AG95 电动夹爪的 ROS2 驱动与描述包，支持真实夹爪控制与 RViz 三维可视化。

---

## 主要特性

- 只需发布主关节（如 `R1_1_joint`）的角度，其他关节由 URDF mimic 自动推导
- 支持夹爪位置、力、速度控制
- 支持 RViz 实时三维显示
- 兼容 ROS2 Humble 及以上

---

## 快速使用

### 1. 构建

```bash
colcon build
source install/setup.bash
```

### 2. 启动夹爪可视化
```bash
ros2 launch dh_ag_95 display_robot.launch.py
```

----

### 3. 启动夹爪驱动与可视化
```bash
ros2 launch dh_gripper_driver dh_ag95_gripper.launch.py
```

----
### 4. 串口权限
```bash
sudo chmod 666 /dev/ttyUSB0
```

----

### 5. 控制夹爪(另开一终端)
```bash
source install/setup.bash
ros2 topic pub --once /gripper/ctrl dh_gripper_driver/msg/GripperCtrl "{initialize: false, position: 50.0, force: 50.0, speed: 20.0}"
```

### 6. 查看关节状态

```bash
ros2 topic echo /gripper/joint_states
```

---

### 7. 引用接口

```C++ Code
DhAg95GripperDriver driver = new DhAg95GripperDriver()
driver.sendGripperCommand(100.0, 50.0, 20.0, false)
```

```python Code
import driver from DhAg95GripperDriver
driver.DhAg95GripperDriver()
driver.sendGripperCommand(100.0, 50.0, 20.0, false)
```

---

## 说明

- **description** 目录下的 URDF 采用 mimic 机制，只需发布主关节即可，robot_state_publisher 会自动推导所有关节的 TF。
- **driver** 目录下的 C++ 节点通过串口与夹爪通信，发布主关节角度到 `/gripper/joint_states`，并订阅 `/gripper/ctrl` 控制夹爪。

---

