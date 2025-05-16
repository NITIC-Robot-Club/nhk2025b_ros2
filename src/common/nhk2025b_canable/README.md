# nhk2025b_canable
CANAbleとの通信を行います

## Input
| topic名 | 型 | 説明 |
| - | - | - |
| /swerve/cmd | Swerve | 独ステの入力 |


## Output
| topic名 | 型 | 説明 |
| - | - | - |
| /swerve/result | Swerve | 現在の独ステ状態 |
| /robot_status | RobotStatus | 現在のロボットの状態 |

## Parameter
| パラメーター名 | 型 | 説明 | デフォルト値 |
| - | - | - | - |
| retry_open_can | bool | 失敗したときにCANを再接続するか | true |
| retry_write_can | bool | 書き込みに失敗したときに再書き込みするか | true |
| max_retry_write_count | int | 書き込み失敗時再接続するまでの回数 | 5 |
