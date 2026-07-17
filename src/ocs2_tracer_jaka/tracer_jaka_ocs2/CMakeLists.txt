cmake_minimum_required(VERSION 3.10)
project(tracer_jaka_ocs2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Eigen3 REQUIRED)

# 关键：Boost 必须在所有 ocs2_* 之前
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  thread
  log
  log_setup
  chrono
  date_time
  atomic
)

find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_mobile_manipulator REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)

set(common_deps
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  interactive_markers
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_oc
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_self_collision
  ocs2_ros_interfaces
  ocs2_mobile_manipulator
  ocs2_msgs
  std_srvs
  ocs2_self_collision_visualization
)

include_directories(include)

# ---- MPC node ----
add_executable(tracer_jaka_mpc_node src/TracerJakaMpcNode.cpp)
ament_target_dependencies(tracer_jaka_mpc_node ${common_deps})
target_link_libraries(tracer_jaka_mpc_node
  Eigen3::Eigen
  Boost::system
  Boost::filesystem
  Boost::thread
  Boost::log
  Boost::log_setup
)

# ---- MRT / Gazebo bridge node ----
add_executable(tracer_jaka_mrt_node 
src/TracerJakaMrtNode.cpp
src/TracerJakaVisualization.cpp
)
ament_target_dependencies(tracer_jaka_mrt_node ${common_deps})
target_link_libraries(tracer_jaka_mrt_node
  Eigen3::Eigen
  Boost::system
  Boost::filesystem
  Boost::thread
  Boost::log
  Boost::log_setup
)

# ---- Target publisher ----
add_executable(tracer_jaka_target_node src/TracerJakaTargetNode.cpp)
ament_target_dependencies(tracer_jaka_target_node ${common_deps})
target_link_libraries(tracer_jaka_target_node
  Eigen3::Eigen
  Boost::system
  Boost::filesystem
  Boost::thread
  Boost::log
  Boost::log_setup
)

# ---- Target publisher ----
add_executable(tracer_jaka_joy_target_node src/tracer_jaka_joy_target_node.cpp)
ament_target_dependencies(tracer_jaka_joy_target_node ${common_deps})
target_link_libraries(tracer_jaka_joy_target_node
  Eigen3::Eigen
  Boost::system
  Boost::filesystem
  Boost::thread
  Boost::log
  Boost::log_setup
)


add_executable(csv_path_visualizer_node src/csv_path_visualizer_node.cpp)
ament_target_dependencies(csv_path_visualizer_node ${common_deps})
target_link_libraries(csv_path_visualizer_node
  Eigen3::Eigen
  Boost::system
  Boost::filesystem
  Boost::thread
  Boost::log
  Boost::log_setup
)

add_executable(tracer_jaka_whole_body_trajectory_node
  src/tracer_jaka_whole_body_trajectory_node.cpp
)
ament_target_dependencies(tracer_jaka_whole_body_trajectory_node ${common_deps})
target_link_libraries(tracer_jaka_whole_body_trajectory_node
  Eigen3::Eigen
  Boost::system
  Boost::filesystem
  Boost::thread
  Boost::log
  Boost::log_setup
)



# ---- Install ----
install(TARGETS
  tracer_jaka_mpc_node
  tracer_jaka_mrt_node
  tracer_jaka_target_node
  tracer_jaka_joy_target_node
  tracer_jaka_whole_body_trajectory_node
  csv_path_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz scripts include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
