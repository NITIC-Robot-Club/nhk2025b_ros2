# nhk2025b_wheel_odometry
CANを受信して足回りのオドメトリを計算

## Input
| topic名 | 型 | 説明 |
| - | - | - |
| /swerve/result | Swerve | 現在の独ステ状態 |

## Output
| topic名 | 型 | 説明 |
| - | - | - |
| /localization/wheel_odometry | Odometry | 計算されたオドメトリ |