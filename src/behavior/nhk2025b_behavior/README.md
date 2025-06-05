# nhk2025b_behavior
行動管理を行います

## Input

| topic名 | 型 | 説明 |
| - | - | - |
| /localization/current_pose | PoseStamped | 自己位置 |
| /robot_status | RobotStatus | ロボットの現在の状態 |
| /behavior/set_status_num | Int32 | 状態の指定 |
| /command | Command | ロボットへのコマンド |

## Output

| topic名 | 型 | 説明 |
| - | - | - |
| /behavior/goal_pose | PoseStamped | ロボットの目標位置 |
| /behavior/state | State | 現在の状態 |
| /behavior/state_array | StateArray | 実行中のフローチャート |
