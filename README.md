# nhk2025b_ros2
[![build docker image](https://github.com/NITIC-Robot-Club/nhk2025b_ros2/actions/workflows/docker_build.yaml/badge.svg)](https://github.com/NITIC-Robot-Club/nhk2025b_ros2/actions/workflows/docker_build.yaml)
ros2のwsです

[Document](https://nitic-robot-club.github.io/nhk2025b_ros2)

## Build All with Release
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```