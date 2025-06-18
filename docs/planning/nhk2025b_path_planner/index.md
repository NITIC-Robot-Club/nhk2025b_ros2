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

## Parameter
| パラメーター名 | 型 | 説明 | デフォルト値 |
| - | - | - | - |
| robot_width_mm | int | ロボットの横幅(mm) | 600 |
| robot_height_mm | int | ロボットの縦幅(mm) | 1000 |
| robot_offset_mm | int | 当たらないように余裕を(mm) | 50 |
| tolerance_xy_mm | int | 自己位置と目標位置の差の許容誤差 | 30 |
| tolerance_z_dig | int | 自己位置と目標位置の角度の差の許容誤差 | 2 |
