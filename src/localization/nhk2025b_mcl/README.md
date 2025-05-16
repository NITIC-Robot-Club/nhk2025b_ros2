# nhk2025b_mcl
LiDARとmapのマッチング

## Input
| topic名 | 型 | 説明 |
| - | - | - |
| /sensor/scan | LaserScan | 2D LiDARの入力 |
| /behavior/map | OccupancyGrid | 現在のフィールド |
| /localization/initialpose | PoseWithCovarianceStamped | 初期位置 |
| /localization/ekf/pose | PoseStamped | EKFのオドメトリ |

## Output
| topic名 | 型 | 説明 |
| - | - | - |
| /localization/distance_map | OccupancyGrid | 距離地図 |
| /localization/current_pose | PoseStamped | 最終的な自己位置 |
| /localization/mcl_particles | PoseArray | パーティクル |
| /localization/velocity | TwistStamped | 推定されたロボットの速度 |