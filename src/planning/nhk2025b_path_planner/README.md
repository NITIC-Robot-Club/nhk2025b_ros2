# nhk2025b_path_planner
現在位置と目標位置から経路を生成します

## Input
| topic名 | 型 | 説明 |
| - | - | - |
| /localization/current_pose | PoseStamped | 自己位置 |
| /behavior/goal_pose | PoseStamped | 目標位置 |
| /behavior/map | OccupancyGrid | 現在のフィールド |

## Output
| topic名 | 型 | 説明 |
| - | - | - |
| /planning/path | Path | 生成した経路 |
| /planning/costmap | OccupancyGrid | コストマップ |
