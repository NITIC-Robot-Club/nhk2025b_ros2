# nhk2025b_footprint_publisher
ロボットサイズの長方形を当たり判定付きで表示します

## Input
| topic名 | 型 | 説明 |
| - | - | - |
| /localization/current_pose | PoseStamped | 自己位置 |
| /behavior/map | OccupancyGrid | 当たり判定に利用するマップ |

## Output
| topic名 | 型 | 説明 |
| - | - | - |
| /visualization/robot_footprint | MarkerArray | 軌跡 |
