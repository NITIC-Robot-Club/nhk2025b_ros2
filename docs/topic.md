# Topic

# まだ途中
## /
| topic名 | 型 | 説明 |
| - | - | - |
| cmd_vel | geometry_msgs/TwistStamped | 足回りに送信する速度 |
| robot_status | nhk2025b_msgs/RobotStatus | ロボットの現在の状態 |
| command | nhk2025b_msgs/Command | ロボットの状態の司令 |

## /swerve
| topic名 | 型 | 説明 |
| - | - | - |
| cmd | nhk2025b_msgs/Swerve | 足回り司令 |
| result | nhk2025b_msgs/Swerve | 足回り結果 |

## /conveyor
| topic名 | 型 | 説明 |
| - | - | - |
| cmd | nhk2025b_msgs/Conveyor | コンベヤの司令 |
| result | nhk2025b_msgs/Conveyor | コンベヤの結果 |

## /box_arm
| topic名 | 型 | 説明 |
| - | - | - |
| cmd | nhk2025b_msgs/BoxArm | アーム司令 |
| result | nhk2025b_msgs/BoxArm | アーム結果 |

## /pylon_arm
| topic名 | 型 | 説明 |
| - | - | - |
| cmd | nhk2025b_msgs/PylonArm | パイロンアーム司令 |
| result | nhk2025b_msgs/PylonArm | パイロンアーム結果 |

## /localization
