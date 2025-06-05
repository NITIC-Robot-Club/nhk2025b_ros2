# nhk2025b_msgs
メッセージパッケージです

## 含まれるメッセージ
### Command
| 型 | 名前 | 説明 |
| - | - | - |
| std_msgs/Header | header | 時間やフレーム情報
| bool | signal | 電源をつけるかどうか
| bool | automate_ready | 自動を許可するか

### RobotStatus
| 型 | 名前 | 説明 |
| - | - | -
| std_msgs/Header | header | 時間やフレーム情報 |
| bool | signal | 電源がついているか |
| bool | is_resetting | 原点をとっている最中か |
| float32 | voltage | 電圧(V) |

### State
| 型 | 名前 | 説明 |
| - | - | - |
| string | name | 状態の名前
| int32 | id | 状態を管理するためのID

### StateArray
| 型 | 名前 | 説明 |
| - | - | - |
| string | name | 状態遷移図の名前 |
| nhk2025b_msgs/State[] | state | 状態s |

### Swerve
| 型 | 名前 | 説明 |
| - | - | - |
| std_msgs/Header | header | 時間やフレーム情報 |
| float32[4] | wheel_angle | ホイール角度(rad) |
| float32[4] | wheel_speed | ホイール速度(m/s) |
