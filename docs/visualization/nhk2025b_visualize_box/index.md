# nhk2025b_visualize_swerve
箱の状態を表示します

## Input
| topic名 | 型 | 説明 |
| - | - | - |
| /box_state | BoxArray | 表示する独ステ |

## Output
| topic名 | 型 | 説明 |
| - | - | - |
| /visualization/box | MerkarArray | 箱 |

## Example
```bash
ros2 topic pub /box_state nhk2025b_msgs/msg/BoxArray "{boxes: [{pose: {position: {x: 7.0, y: 5.0, z: 0.144}, orientation: {x: 0.0, y: 0.0, z: 0.7, w: 0.7}}, size: {x: 1.0, y: 0.3, z: 0.3}}]}" -r 10
```