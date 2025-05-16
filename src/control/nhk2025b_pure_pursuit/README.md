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
