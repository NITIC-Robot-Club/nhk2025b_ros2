# nhk2025b_swerve_calculator
独ステの角度、速度を計算します

## Input
| topic名 | 型 | 説明 |
| - | - | - |
| /cmd_vel | TwistStamped | 移動方向のベクトル入力 |

## Output
| topic名 | 型 | 説明 |
| - | - | - |
| /swerve/cmd | Swerve | 計算された値 |

## Parameter
| パラメーター名 | 型 | 説明 | デフォルト値 |
| - | - | - | - |
| wheel_radius | double | タイヤ半径(m) | 0.0325 |