#ifndef __CANABLE_NODE_HPP__
#define __CANABLE_NODE_HPP__

#include "rclcpp/rclcpp.hpp"

#include "nhk2025b_msgs/msg/box_arm.hpp"
#include "nhk2025b_msgs/msg/command.hpp"
#include "nhk2025b_msgs/msg/conveyor.hpp"
#include "nhk2025b_msgs/msg/pylon_arm.hpp"
#include "nhk2025b_msgs/msg/robot_status.hpp"
#include "nhk2025b_msgs/msg/swerve.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <mutex>

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
    void swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg);
    void box_arm_callback (const nhk2025b_msgs::msg::BoxArm::SharedPtr msg);
    void command_callback (const nhk2025b_msgs::msg::Command::SharedPtr msg);
    void conveyor_callback (const nhk2025b_msgs::msg::Conveyor::SharedPtr msg);
    void pylon_arm_callback (const nhk2025b_msgs::msg::PylonArm::SharedPtr msg);

    // === ソケット・ステータス ===
    int  can_socket_;
    bool retry_open_can        = true;
    bool retry_write_can       = true;
    int  retry_write_count     = 0;
    int  max_retry_write_count = 5;

    struct sockaddr_can addr_;
    struct ifreq        ifr_;

    // === 最新メッセージ保存用 ===
    nhk2025b_msgs::msg::Command::SharedPtr  command_;
    nhk2025b_msgs::msg::Swerve::SharedPtr   swerve_cmd_;
    nhk2025b_msgs::msg::BoxArm::SharedPtr   box_arm_cmd_;
    nhk2025b_msgs::msg::Conveyor::SharedPtr conveyor_cmd_;
    nhk2025b_msgs::msg::PylonArm::SharedPtr pylon_arm_cmd_;

    nhk2025b_msgs::msg::RobotStatus robot_status_;
    nhk2025b_msgs::msg::Swerve      swerve_result_;
    nhk2025b_msgs::msg::BoxArm      box_arm_result_;
    nhk2025b_msgs::msg::PylonArm    pylon_arm_result_;

    bool       swerve_flag_[4]    = {false, false, false, false};
    bool       robot_status_flag_ = false;
    bool       box_arm_flag_[2]   = {false, false};
    std::mutex data_mutex_;

    int  id_list[12] = {0x100, 0x101, 0x110, 0x111, 0x112, 0x113, 0x114, 0x115, 0x120, 0x121, 0x122, 0x123};
    bool id_flag[12] = {false, false, false, false, false, false, false, false, false, false, false, false};

    // === 構造体 ===

    union float_bytes {  // float byte変換
        uint8_t bytes[4];
        float   value;
    };

    union power_receive_union {
        uint8_t raw;
        struct {
            uint8_t sig : 1;
            uint8_t : 7;
        } data;
    } power_receive;

    union power_transmit_union {
        uint8_t raw;
        struct {
            uint8_t sig : 1;
            uint8_t : 6;
        } data;
    } power_transmit;

    union claw_receive_union {
        uint8_t raw;
        struct {
            uint8_t reset_swerve : 1;
            uint8_t expand_pylon_arm_left : 1;
            uint8_t expand_pylon_arm_right : 1;
            uint8_t : 5;
        } data;
    } claw_receive;

    union claw_transmit_union {
        uint8_t raw;
        struct {
            uint8_t swerve_reset_success : 1;
            uint8_t : 7;
        } data;
    } claw_transmit;

    union wing_receive_union {
        uint8_t raw;
        struct {
            uint8_t expand_left : 1;
            uint8_t expand_right : 1;
            uint8_t E_ready : 1;
            uint8_t E_get : 1;
            uint8_t : 4;
        } data;
    } wing_receive;

    union wing_transmit_union {
        uint8_t raw;
        struct {
            uint8_t expanded_left : 1;
            uint8_t expanded_right : 1;
            uint8_t E_ready : 1;
            uint8_t E_got : 1;
            uint8_t : 4;
        } data;
    } wing_transmit;

    // === ROS 通信 ===
    rclcpp::Subscription<nhk2025b_msgs::msg::Swerve>::SharedPtr   swerve_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Command>::SharedPtr  command_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::BoxArm>::SharedPtr   box_arm_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Conveyor>::SharedPtr conveyor_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::PylonArm>::SharedPtr pylon_arm_sub_;

    rclcpp::Publisher<nhk2025b_msgs::msg::Swerve>::SharedPtr      swerve_pub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::BoxArm>::SharedPtr      box_arm_pub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::PylonArm>::SharedPtr    pylon_arm_pub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::RobotStatus>::SharedPtr robot_status_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr can_receive_timer_;
};

}  // namespace canable

#endif  // __CANABLE_NODE_HPP__
