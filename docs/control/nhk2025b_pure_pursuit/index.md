# nhk2025b_pure_pursuit
単純追従を用いて経路から移動ベクトルを計算します

## Input
| topic名 | 型 | 説明 |
| - | - | - |
| /localization/current_pose | PoseStamped | 自己位置 |
| /planning/path | Path | 経路 |

## Output
| topic名 | 型 | 説明 |
| - | - | - |
| /cmd_vel | TwistStamped | 出力ベクトル |
| /control/lookahead_pose | PoseStamped | 注視点1 |
| /control/lookahead_pose2 | PoseStamped | 曲率計算時に使用するポーズ |

## Parameter
| パラメーター名 | 型 | 説明 | デフォルト値 |
| - | - | - | - |
| lookahead_time | double | 注視距離の割合 | 1.0 |
| min_lookahead_distance | double | 最小注視距離(m) | 0.3 |
| max_lookahead_distance | double | 最大注視距離(m) | 3.0 |
| angle_lookahead_distance | double | 角度注視距離(m) | 1.0 |
| curvature_decceleration_p | double | カーブ減速比 | 10.0 |
| angle_p | double | 角度Pゲイン | 2.0 |
| max_speed_xy_m_s | double | 最高速度(m/s) | 3.0 |
| max_speed_z_rad_s | double | 最高旋回速度(rad/s) | 3.14 |
| max_acceleration_xy_m_s2 | double | 最高加速度(m/s^2) | 6.0 |
