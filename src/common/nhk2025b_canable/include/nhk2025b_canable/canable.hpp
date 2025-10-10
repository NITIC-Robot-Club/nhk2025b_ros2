#ifndef __CANABLE_NODE_HPP__
#define __CANABLE_NODE_HPP__

#include "rclcpp/rclcpp.hpp"

#include "nhk2025b_msgs/msg/box_arm.hpp"
#include "nhk2025b_msgs/msg/command.hpp"
#include "nhk2025b_msgs/msg/conveyor.hpp"
#include "nhk2025b_msgs/msg/e_arm.hpp"
#include "nhk2025b_msgs/msg/pylon_arm.hpp"
#include "nhk2025b_msgs/msg/robot_status.hpp"
#include "nhk2025b_msgs/msg/swerve.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <filesystem>
#include <mutex>
#include <string>

bool can_interface_exists (const std::string &ifname) {
    return std::filesystem::exists ("/sys/class/net/" + ifname);
}

namespace canable {

class canable : public rclcpp::Node {
   public:
    explicit canable (const rclcpp::NodeOptions &node_options);
    ~canable ();

   private:
    // === 初期化と処理 ===
    int  init_can_socket ();    // CANソケット初期化
    void read_can_socket ();    // CAN受信スレッド
    void timer_callback ();     // 10ms周期送信コールバック
    void check_can_receive ();  // CAN受信確認

    // === サブスクコールバック ===
    void e_arm_callback (const nhk2025b_msgs::msg::EArm::SharedPtr msg);
    void swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg);
    void is_red_callback (const std_msgs::msg::Bool::SharedPtr msg);
    void box_arm_callback (const nhk2025b_msgs::msg::BoxArm::SharedPtr msg);
    void command_callback (const nhk2025b_msgs::msg::Command::SharedPtr msg);
    void conveyor_callback (const nhk2025b_msgs::msg::Conveyor::SharedPtr msg);
    void pylon_arm_callback (const nhk2025b_msgs::msg::PylonArm::SharedPtr msg);

    void e_arm_controller_callback (const nhk2025b_msgs::msg::EArm::SharedPtr msg);
    void box_arm_controller_callback (const nhk2025b_msgs::msg::BoxArm::SharedPtr msg);
    void conveyor_controller_callback (const nhk2025b_msgs::msg::Conveyor::SharedPtr msg);
    void pylon_arm_controller_callback (const nhk2025b_msgs::msg::PylonArm::SharedPtr msg);

    // === ソケット・ステータス ===
    int  can_socket_;
    bool retry_open_can        = true;
    bool retry_write_can       = true;
    int  retry_write_count     = 0;
    int  max_retry_write_count = 5;
    bool can_alive_            = false;

    struct sockaddr_can addr_;
    struct ifreq        ifr_;

    // === 最新メッセージ保存用 ===
    nhk2025b_msgs::msg::Command  command_;
    nhk2025b_msgs::msg::EArm     e_arm_cmd_;
    nhk2025b_msgs::msg::Swerve   swerve_cmd_;
    nhk2025b_msgs::msg::BoxArm   box_arm_cmd_;
    nhk2025b_msgs::msg::Conveyor conveyor_cmd_;
    nhk2025b_msgs::msg::PylonArm pylon_arm_cmd_;

    nhk2025b_msgs::msg::RobotStatus robot_status_;
    nhk2025b_msgs::msg::EArm        e_arm_result_;
    nhk2025b_msgs::msg::Swerve      swerve_result_;
    nhk2025b_msgs::msg::BoxArm      box_arm_result_;
    nhk2025b_msgs::msg::Conveyor    conveyor_result_;
    nhk2025b_msgs::msg::PylonArm    pylon_arm_result_;

    bool swerve_flag_[4]          = {false};
    bool robot_status_flag_       = false;
    int  robomas_current_[2]      = {0};
    bool robomas_current_flag_[2] = {false};

    bool             is_red_           = false;
    static const int max_led           = 115;
    static const int led_move_length   = 4;  // 各塊のLED数
    static const int led_move_count    = 4;  // 同時に進む塊の数
    int              led_step_         = 0;
    int              led_speed_div     = 3;  // 値を大きくすると遅くなる
    int              led_speed_counter = 0;
    std::mutex       data_mutex_;

    int  id_list[17] = {0x100, 0x101, 0x110, 0x111, 0x112, 0x113, 0x114, 0x115, 0x116, 0x117, 0x118, 0x120, 0x121, 0x122, 0x123, 0x124, 0x125};
    bool id_flag[17] = {false};

    // === 構造体 ===

    union float_bytes {  // float byte変換
        uint8_t bytes[4];
        float   value;
    };

    union power_receive_union {
        uint8_t raw;
        struct {
            uint8_t sig : 1;
            uint8_t is_red : 1;
            uint8_t : 6;
        } data;
    } power_receive;

    union power_transmit_union {
        uint8_t raw;
        struct {
            uint8_t sig : 1;
            uint8_t : 7;
        } data;
    } power_transmit;

    union claw_receive_union {
        uint8_t raw;
        struct {
            uint8_t reset : 1;
            uint8_t : 7;
        } data;
    } claw_receive;

    union claw_transmit_union {
        uint8_t raw;
        struct {
            uint8_t reset_height_0 : 1;
            uint8_t reset_height_1 : 1;
            uint8_t reset_expand_0 : 1;
            uint8_t reset_expand_1 : 1;
            uint8_t : 4;
        } data;
    } claw_transmit;

    union wing_receive_union {
        uint8_t raw;
        struct {
            uint8_t reset : 1;
            uint8_t : 5;
        } data;
    } wing_receive;

    union wing_transmit_union {
        uint8_t raw;
        struct {
            uint8_t reset_height_0 : 1;
            uint8_t reset_height_1 : 1;
            uint8_t reset_hand_0 : 1;
            uint8_t reset_hand_1 : 1;
            uint8_t reset_expand_0 : 1;
            uint8_t reset_expand_1 : 1;
            uint8_t reset_e_arm_expand : 1;
            uint8_t reset_e_arm_get : 1;
        } data;
    } wing_transmit;

    class UInt24 {
       private:
        uint32_t value_;

       public:
        UInt24 () : value_ (0) {}
        void set_value (uint32_t v) {
            value_ = v & 0xFFFFFF;
        }
        uint32_t get_value () const {
            return value_;
        }
        void set_byte (int i, uint8_t b) {
            if (i < 0 || i > 2) return;
            value_ &= ~(0xFF << (8 * i));
            value_ |= (uint32_t (b) << (8 * i));
        }
        uint8_t get_byte (int i) const {
            if (i < 0 || i > 2) return 0;
            return (value_ >> (8 * i)) & 0xFF;
        }
        void set_bytes (const uint8_t b[3]) {
            set_byte (0, b[0]);
            set_byte (1, b[1]);
            set_byte (2, b[2]);
        }
        void get_bytes (uint8_t b[3]) const {
            b[0] = get_byte (0);
            b[1] = get_byte (1);
            b[2] = get_byte (2);
        }
    };

    // === ROS 通信 ===
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          is_red_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::EArm>::SharedPtr     e_arm_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Swerve>::SharedPtr   swerve_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::BoxArm>::SharedPtr   box_arm_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Command>::SharedPtr  command_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Conveyor>::SharedPtr conveyor_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::PylonArm>::SharedPtr pylon_arm_sub_;

    rclcpp::Subscription<nhk2025b_msgs::msg::EArm>::SharedPtr     e_arm_controller_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::BoxArm>::SharedPtr   box_arm_controller_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Conveyor>::SharedPtr conveyor_controller_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::PylonArm>::SharedPtr pylon_arm_controller_sub_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr            robomas_current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr          bno_yaw_pub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::EArm>::SharedPtr        e_arm_pub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::Swerve>::SharedPtr      swerve_pub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::BoxArm>::SharedPtr      box_arm_pub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::Conveyor>::SharedPtr    conveyor_pub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::PylonArm>::SharedPtr    pylon_arm_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr  missing_can_id_pub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::RobotStatus>::SharedPtr robot_status_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr can_receive_timer_;
};

}  // namespace canable

#endif  // __CANABLE_NODE_HPP__
