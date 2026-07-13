cmake_minimum_required(VERSION 3.14)
project(ocs2_robotic_assets)

find_package(ament_cmake REQUIRED)

# Resolve for the package path at compile time.
configure_file(
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

# Modern header-only INTERFACE target so downstream packages get a proper
# imported target (${ocs2_robotic_assets_TARGETS}) via find_package().
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)

ament_package()
