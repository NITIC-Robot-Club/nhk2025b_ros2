# nhk2025b_launch
launchファイルやパラメーターなどをまとめたパッケージです。

## How to use
### Simulation
```bash
ros2 launch nhk2025b_launch simulation.launch.xml
```

赤ゾーンのシミュレーションは

```bash
ros2 launch nhk2025b_launch simulation.launch.xml is_red:=true
```

## Launch一覧
- default
    - map
    - tf
    - behavior
- localization
    - ekf
    - wheel_odometry
    - mcl
    - pose_initializer
- planning
    - path_planner
    - pure_pursuit
- rviz2
    - rviz2
- swerve
    - swerve_calculator
    - canable
- visualization
    - visualize_swerve
    - footprint_publisher

- tf
    - base_link -> lidar

- real
    - default
    - planning
    - swerve
    - localization

- simulation
    - simulation
    - lidar_simulation
    - swerve_calculator
    - rviz2
    - default
    - localization
    - planning

