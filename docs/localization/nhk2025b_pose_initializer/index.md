# nhk2025b_pose_initializer
RANSACで初期位置を取得

## Input
| topic名 | 型 | 説明 |
| - | - | - |
| /sensor/scan | LaserScan | 2D LiDAR |

## Output
| topic名 | 型 | 説明 |
| - | - | - |
| /localization/initialpose | PoseWithCovarianceStamped | 初期位置 |

## Parameter
| パラメーター名 | 型 | 説明 | デフォルト値 |
| - | - | - | - |
| max_iteration | int | 最大イテレーション数 | 100 |
| distance_threshold | double | 壁検出許容誤差 | 0.1 |