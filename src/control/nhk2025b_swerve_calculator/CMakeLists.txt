
cmake_minimum_required(VERSION 3.8)
project(nhk2025b_swerve_calculator)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

target_include_directories(${PROJECT_NAME}
    SYSTEM PUBLIC
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "swerve_calculator::swerve_calculator"
    EXECUTABLE swerve_calculator
)

if($ENV{ROS_DISTRO} STREQUAL "iron" OR $ENV{ROS_DISTRO} STREQUAL "jazzy" OR $ENV{ROS_DISTRO} STREQUAL "rolling")
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
else()
  ament_auto_package(INSTALL_TO_SHARE)
endif()