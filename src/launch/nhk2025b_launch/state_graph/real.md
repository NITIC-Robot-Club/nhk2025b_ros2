```mermaid
stateDiagram-v2

    start_position_0     : 収納後スタートゾーンに
    start_position_0     : set_position(1.0, 1.0, 180.0)
    start_pylon_height   : set_pylon_arm_height(right = 0.0, left = 0.0)
    start_pylon_expand   : set_pylon_arm_expand(right = 90,  left = 90)
    start_box_arm_height : set_box_arm_height(right = 0.9, left = 0.9)
    start_box_arm_expand : set_box_arm_expand(right = 0.0, left = 0.0)
    start_e_arm          : set_e_arm(expand = 0, get = 0.00)
    start_position_1     : set_position(0.6, 0.5, 180.0)

#    [*]                  --> start_position_0
    start_position_0     --> start_pylon_height
    start_pylon_height   --> start_pylon_expand
    start_pylon_expand   --> start_box_arm_height
    start_box_arm_height --> start_box_arm_expand
    start_box_arm_expand --> start_e_arm
    start_e_arm          --> start_position_1 : check_pylon_arm() and check_e_arm() and check_box_arm()


    pylon_0_position_get   : パイロン0 移動
    pylon_0_position_get   : set_position(3.0, 1.15, 180.0)
    pylon_0_position_drop  : set_position(5.0, 1.15, 180.0)
    pylon_0_position_out_0 : set_position(6.0, 1.0, 90.0)
    pylon_0_position_out_1 : set_position(6.0, 2.0, 90.0)
    pylon_0_box_arm_expand : set_box_arm_expand(right = 90, left = 90)
    pylon_0_box_arm_height : set_box_arm_height(right = 0.0, left = 0.0)

#    start_position_1       --> pylon_0_position_get
    pylon_0_position_get   --> pylon_0_position_drop
    pylon_0_position_drop  --> pylon_0_position_out_0
    pylon_0_position_out_0 --> pylon_0_position_out_1
    pylon_0_position_out_1 --> pylon_0_box_arm_expand
    pylon_0_box_arm_expand --> pylon_0_box_arm_height
    

    e_01_pylon_height_init : E01 回収
    e_01_pylon_height_init : set_pylon_arm_height(right = 0.16, left = 0.16)
    e_01_pylon_expand_init : set_pylon_arm_expand(right = 0, left = 0)
    e_01_pylon_rpm_init    : set_pylon_arm_rpm(right = 250, left = 250)
    e_01_conveyor_init     : set_conveyor_rpm(right = 400, left = 400)
    e_01_position_init_0   : set_position(8.78, 3.5, 90.0)
    e_01_position_init_1   : set_position(8.78, 4.0, 90.0)
    e_01_position_get      : set_position(8.78, 4.7, 90.0)
    e_01_pylon_expand_get  : set_pylon_arm_expand(right = 30, left = 30)

    pylon_0_box_arm_height --> e_01_pylon_height_init
    e_01_pylon_height_init --> e_01_pylon_expand_init
    e_01_pylon_expand_init --> e_01_pylon_rpm_init
    e_01_pylon_rpm_init    --> e_01_conveyor_init
    e_01_conveyor_init     --> e_01_position_init_0
    e_01_position_init_0   --> e_01_position_init_1
    e_01_position_init_1   --> e_01_position_get
    e_01_position_get      --> e_01_pylon_expand_get


    e_01_pylon_height_hold : E01 保持完了
    e_01_pylon_height_hold : set_pylon_arm_height(right = 0.115, left = 0.115)
    e_01_pylon_expand_hold : set_pylon_arm_expand(right = 0, left = 0)
    e_01_pylon_rpm_hold    : set_pylon_arm_rpm(right = 0, left = 0)
    e_01_conveyor_hold     : set_conveyor_rpm(right = 300, left = 300)
    e_01_position_hold     : set_position(8.78, 3.5, 90.0)

#    e_01_pylon_expand_get  --> e_01_pylon_height_hold : check_pylon_arm()
    e_01_pylon_height_hold --> e_01_pylon_expand_hold
    e_01_pylon_expand_hold --> e_01_pylon_rpm_hold
    e_01_pylon_rpm_hold    --> e_01_conveyor_hold
    e_01_conveyor_hold     --> e_01_position_hold
    

    e_0_position_drop     : E0 吐き出し
    e_0_position_drop     : set_position(4.0, 3.5, 90.0)
    e_0_pylon_height_drop : set_pylon_arm_height(right = 0.2)
    e_0_pylon_rpm_drop    : set_pylon_arm_rpm(right = 0)
    e_0_conveyor_drop     : set_conveyor_rpm(right = -400)
    e_0_position_drop_out : set_position(4.0, 3.0, 90.0)
    
    e_01_position_hold    --> e_0_position_drop
    e_0_position_drop     --> e_0_pylon_height_drop
    e_0_pylon_height_drop --> e_0_pylon_rpm_drop
    e_0_pylon_rpm_drop    --> e_0_conveyor_drop
    e_0_conveyor_drop     --> e_0_position_drop_out
    
    e_1_position_drop     : E1 吐き出し
    e_1_position_drop     : set_position(2.0, 3.0, 90.0)
    e_1_pylon_height_drop : set_pylon_arm_height(left = 0.2)
    e_1_pylon_rpm_drop    : set_pylon_arm_rpm(left = 0)
    e_1_conveyor_drop     : set_conveyor_rpm(left = -400)
    e_1_position_drop_out : set_position(2.0, 2.5, 90.0)
    
    e_0_position_drop_out --> e_1_position_drop
    e_1_position_drop     --> e_1_pylon_height_drop
    e_1_pylon_height_drop --> e_1_pylon_rpm_drop
    e_1_pylon_rpm_drop    --> e_1_conveyor_drop
    e_1_conveyor_drop     --> e_1_position_drop_out


    pylon_12_position_setup_0 : パイロン12　移動
    pylon_12_position_setup_0 : set_position(1.5, 2.0, 0.0)
    pylon_12_pylon_expand     : set_pylon_arm_expand(right = 90, left = 90)
    pylon_12_conveyor_stop    : set_conveyor_rpm(right = 0, left = 0)
    pylon_12_position_setup_1 : set_position(1.5, 1.2, 0.0)

    e_1_position_drop_out     --> pylon_12_position_setup_0
    pylon_12_position_setup_0 --> pylon_12_pylon_expand
    pylon_12_pylon_expand     --> pylon_12_conveyor_stop
    pylon_12_conveyor_stop    --> pylon_12_position_setup_1


    pylon_12_position_get  : set_position(2.6, 1.2, 0.0)
    pylon_12_position_drop : set_position(6.0, 1.2, 0.0)
    pylon_12_position_out  : set_position(5.0, 1.2, 0.0)

    pylon_12_position_setup_1 --> pylon_12_position_get
    pylon_12_position_get     --> pylon_12_position_drop
    pylon_12_position_drop    --> pylon_12_position_out

    
    ex_0_init   : 専有0
    ex_0_init   : set_position(7.0, 3.4, 90.0)
    ex_0_setup  : set_position(9.5, 3.4, 90.0)
    ex_0_get    : set_position(9.5, 2.5, 90.0)
    ex_0_get_ok : set_position(9.5, 3.5, 90.0)

    pylon_12_position_out --> ex_0_init
    ex_0_init             --> ex_0_setup
    ex_0_setup            --> ex_0_get
    ex_0_get              --> ex_0_get_ok


    gate_0_init : ゲート作成0
    gate_0_init : set_position(3.5, 2.0, 90.0)
    gate_0_turn : set_position(3.5, 2.0, 0.0)
    gate_0_out  : set_position(2.0, 2.0, 0.0)

    ex_0_get_ok --> gate_0_init
    gate_0_init --> gate_0_turn
    gate_0_turn --> gate_0_out

    e_init : E回収
    e_init : set_position(2.0, 2.5, 0.0)
    
    gate_0_out --> e_init

    ex_1_init   : 専有1
    ex_1_init   : set_position(8.5, 3.0, 0.0)
    ex_1_setup  : set_position(9.5, 3.0, 0.0)
    ex_1_get    : set_position(9.5, 1.5, 0.0)
    ex_1_get_ok : set_position(8.5, 1.5, 0.0)

    e_init     --> ex_1_init
    ex_1_init  --> ex_1_setup
    ex_1_setup --> ex_1_get
    ex_1_get   --> ex_1_get_ok 
```