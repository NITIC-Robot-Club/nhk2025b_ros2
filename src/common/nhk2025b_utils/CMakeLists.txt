cmake_minimum_required(VERSION 3.8)
project(nhk2025b_utils)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(
  DIRECTORY include/
  DESTINATION include
)

ament_auto_package(
    INSTALL_TO_SHARE
)