cmake_minimum_required(VERSION 3.8)
project(jaka_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================
# 1. 寻找依赖包
# =============================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)     # JointState / Joy
find_package(geometry_msgs REQUIRED)   # PoseStamped / TwistStamped
find_package(control_msgs REQUIRED)    # JointJog
find_package(std_srvs REQUIRED)        # Trigger
find_package(tf2 REQUIRED)             # Quaternion
find_package(moveit_msgs REQUIRED)     # PlanningScene
find_package(dh_gripper_driver REQUIRED)

# =============================================
# 2. 头文件路径
# =============================================
include_directories(
  include
  include/jaka_driver
)

# =============================================
# 3. 可执行文件（原有 JAKA 驱动相关节点）
# =============================================
add_executable(jaka_edg_node    src/jaka_edg_node.cpp)
add_executable(jaka_login       src/jaka_login.cpp)
add_executable(jaka_logout      src/jaka_logout.cpp)
add_executable(jaka_tcp_sensor  src/jaka_tcp_sensor.cpp)

# 3.1 链接 ROS 2 标准库
ament_target_dependencies(jaka_edg_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
)

# 3.2 链接 JAKA 第三方库 (libjakaAPI.so)
target_link_libraries(jaka_edg_node
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libjakaAPI.so
)
target_link_libraries(jaka_login
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libjakaAPI.so
)
target_link_libraries(jaka_logout
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libjakaAPI.so
)
target_link_libraries(jaka_tcp_sensor
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libjakaAPI.so
)

# =============================================
# 4. Joystick -> MoveIt Servo 组件
# =============================================
# 源码：src/jaka_joy_to_servo.cpp
# 会生成：
#   - 共享库 libjaka_joy_to_servo.so        -> 供 ComposableNodeContainer 加载
#   - 可执行文件 jaka_joy_to_servo_node     -> 可直接 ros2 run 调试
add_library(jaka_joy_to_servo SHARED
  src/jaka_joy_to_servo.cpp
)

target_include_directories(jaka_joy_to_servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(jaka_joy_to_servo
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  control_msgs
  std_srvs
  moveit_msgs
  dh_gripper_driver
)

# 注意：这里的 PLUGIN 必须和 .cpp 里
# RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::JoyToServoPub)
# 的全名保持一致。如果改了 namespace，这里也要跟着改。
rclcpp_components_register_node(jaka_joy_to_servo
  PLUGIN "moveit_servo::JoyToServoPub"
  EXECUTABLE jaka_joy_to_servo_node
)

# =============================================
# 5. 安装规则
# =============================================
# 5.1 原有可执行文件
install(TARGETS
  jaka_edg_node
  jaka_login
  jaka_logout
  jaka_tcp_sensor
  DESTINATION lib/${PROJECT_NAME}
)

# 5.2 joy_to_servo 组件库 + 独立可执行文件
install(TARGETS jaka_joy_to_servo
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 5.3 头文件
install(DIRECTORY include/
  DESTINATION include
)

# 5.4 JAKA 第三方库
install(FILES lib/libjakaAPI.so
  DESTINATION lib
)

# 5.5 config / launch
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# =============================================
# 6. 测试
# =============================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =============================================
# 7. 导出
# =============================================
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  control_msgs
  std_srvs
  tf2
  moveit_msgs
)

ament_package()
