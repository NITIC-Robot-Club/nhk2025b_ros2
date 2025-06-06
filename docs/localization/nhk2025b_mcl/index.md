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

## Parameter
| パラメーター名 | 型 | 説明 | デフォルト値 |
| - | - | - | - |
| num_particles | int | パーティクル数 | 150 |
| motion_noise_linear | double | 移動時平行移動ノイズ | 0.01 |
| motion_noise_angle | double | 移動時旋回ノイズ | 0.01 |
| gaussian_stddev_linear | double | 初期化時ノイズ | 1.0 |
| gaussial_stddev_angle | double | 初期化時ノイズ | 1.0 |
| random_particle_map_num | int | 常にランダムなパーティクル数 | 0 |