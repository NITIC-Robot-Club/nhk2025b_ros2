# nhk2025b_msgs
メッセージパッケージです

## 含まれるメッセージ
### RobotStatus
| 型 | 名前 | 説明 |
| - | - | -
| std_msgs/Header | header | 時間やフレーム情報 |
| bool | signal | 電源がついているか |
| bool | is_resetting | 原点をとっている最中か |
| float32 | voltage | 電圧(V) |

### Swerve
| 型 | 名前 | 説明 |
| - | - | - |
| std_msgs/Header | header | 時間やフレーム情報 |
| float32[4] | wheel_angle | ホイール角度(rad) |
| float32[4] | wheel_speed | ホイール速度(m/s) |