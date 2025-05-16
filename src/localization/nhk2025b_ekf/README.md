# nhk2025b_ekf
足回りのオドメトリとIMUを合成し、低ノイズなオドメトリを計算

## Input
| topic名 | 型 | 説明 |
| - | - | - |
| /sensor/imu | Imu | IMUの入力 |
| /localization/wheel_odometry | Odometry | オドメトリ |

## Output
| topic名 | 型 | 説明 |
| - | - | - |
| /localization/ekf/pose | PoseStamped | 合成された位置 |

## Parameter
| パラメーター名 | 型 | 説明 | デフォルト値 |
| - | - | - | - |
