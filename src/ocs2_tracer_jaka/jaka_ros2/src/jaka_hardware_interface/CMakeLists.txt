cmake_minimum_required(VERSION 3.8)
project(jaka_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED) # 这一行已经有了，很好

add_library(
  jaka_hardware_interface
  SHARED
  src/jaka_hardware_interface.cpp
)

target_include_directories(
  jaka_hardware_interface
  PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS} # 【修改 1】添加 Eigen 的头文件路径
)

# ament_target_dependencies 会处理大多数 ROS2 包的依赖
ament_target_dependencies(
  jaka_hardware_interface
  hardware_interface
  pluginlib            # 建议也加上
  rclcpp
  rclcpp_lifecycle
  Eigen3              # 【修改 2】将 Eigen3 加入 ament 依赖管理
)

# prevent pluginlib from using boost
target_compile_definitions(jaka_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface jaka_hardware_interface.xml)

# 指定外部库目录 (jakaAPI)
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
link_directories(${LIBRARY_DIR})

# 链接 jakaAPI
target_link_libraries(jaka_hardware_interface ${LIBRARY_DIR}/libjakaAPI.so)

# --- 以下是安装和导出部分 ---

install(
  TARGETS
  jaka_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

endif()

ament_export_include_directories(include)
ament_export_libraries(jaka_hardware_interface)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Eigen3
)

ament_package()
