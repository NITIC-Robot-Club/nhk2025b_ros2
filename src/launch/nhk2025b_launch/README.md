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
- localization
- planning
- real
- rviz2
- simulation
- swerve
- tf
- visualization
