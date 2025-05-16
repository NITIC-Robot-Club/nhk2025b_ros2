# nhk2025b_simulation
シミュレーションです

# simulation
簡易的なシミュレーターです

## Input
| topic名 | 型 | 説明 |
| - | - | - |
| /swerve/cmd | Swerve | 独ステの入力 |

## Output
| topic名 | 型 | 説明 |
| - | - | - |
| /swerve/result | Swerve | 独ステの結果 |
| /robot_status | RobotStatus | ロボットの状態 |
| /simulation/pose | PoseStamped | シミュレーション内部の真の自己位置 |
| /sensor/imu | Imu | IMUシミュレーション結果 |


# lidar_simulation
LiDARのシミュレーションを行います

## Input
| topic名 | 型 | 説明 |
| - | - | - |
| /behavior/map | OccupancyGrid | マップ |
| /simulation/pose | PoseStamped | シミュレーション位置 |

## Output
| topic名 | 型 | 説明 |
| - | - | - |
| /sensor/scan | LaserScan | シミュレートされた点群 |