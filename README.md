# nhk2025b_ros2
ros2 ws

## Setup
1. Install ROS 2 humble
2. Clone this repo
    ```bash
    git clone git@github.com:NITIC-Robot-Club/nhk2025b_ros2.git
    ```
3. Build
    ```bash
    cd nhk2025b_ros2
    colcon build
    ```
4. Source
    ```bash
    source install/setup.bash
    ```

## Run
### launch rpi
```bash
ros2 launch nhk2025b_launch launch_rpi.launch.xml
```

### launch debugger on pc
```bash
ros2 launch nhk2025b_launch debug.launch.xml
```