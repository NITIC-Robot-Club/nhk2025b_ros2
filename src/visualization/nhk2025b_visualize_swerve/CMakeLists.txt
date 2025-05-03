cmake_minimum_required(VERSION 3.8)
project(nhk2025b_visualize_swerve)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

target_include_directories(${PROJECT_NAME}
    SYSTEM PUBLIC
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "visualize_swerve::visualize_swerve"
    EXECUTABLE visualize_swerve
)

ament_auto_package(
    INSTALL_TO_SHARE
)
