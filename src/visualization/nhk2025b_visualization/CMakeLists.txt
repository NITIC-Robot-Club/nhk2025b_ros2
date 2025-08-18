cmake_minimum_required(VERSION 3.8)
project(nhk2025b_visualization)

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

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "visualize_box::visualize_box"
    EXECUTABLE visualize_box
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "visualize_stl::visualize_stl"
    EXECUTABLE visualize_stl
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "visualize_footprint::visualize_footprint"
    EXECUTABLE visualize_footprint
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "visualize_path_collision::visualize_path_collision"
    EXECUTABLE visualize_path_collision
)

if($ENV{ROS_DISTRO} STREQUAL "iron" OR $ENV{ROS_DISTRO} STREQUAL "jazzy" OR $ENV{ROS_DISTRO} STREQUAL "rolling")
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
else()
  ament_auto_package(INSTALL_TO_SHARE)
endif()