#ifndef __CANABLE_NODE_HPP__
#define __CANABLE_NODE_HPP__

#include "rclcpp/rclcpp.hpp"

#include "nhk2025b_msgs/msg/conveyor.hpp"
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
    int  init_can_socket ();  // CANソケット初期化
    void read_can_socket ();  // CAN受信スレッド
    void timer_callback ();   // 10ms周期送信コールバック

    // === サブスクコールバック ===
    void swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg);
    void conveyor_callback (const nhk2025b_msgs::msg::Conveyor::SharedPtr msg);

    // === ソケット・ステータス ===
    int  can_socket_;
    bool retry_open_can        = true;
    bool retry_write_can       = true;
    int  retry_write_count     = 0;
    int  max_retry_write_count = 5;

    struct sockaddr_can addr_;
    struct ifreq        ifr_;

    // === 最新メッセージ保存用 ===
    nhk2025b_msgs::msg::Swerve::SharedPtr   latest_swerve_;
    nhk2025b_msgs::msg::Swerve              swerve_cmd_;
    nhk2025b_msgs::msg::Conveyor::SharedPtr latest_conveyor_;
    nhk2025b_msgs::msg::RobotStatus         robot_status_;
    bool                                    swerve_flag_[4] = {false, false, false, false};
    std::mutex                              data_mutex_;

    // === ROS 通信 ===
    rclcpp::Publisher<nhk2025b_msgs::msg::RobotStatus>::SharedPtr robot_status_pub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::Swerve>::SharedPtr      swerve_pub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Swerve>::SharedPtr   swerve_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Conveyor>::SharedPtr conveyor_sub_;
    rclcpp::TimerBase::SharedPtr                                  timer_;
};

}  // namespace canable

#endif  // __CANABLE_NODE_HPP__
