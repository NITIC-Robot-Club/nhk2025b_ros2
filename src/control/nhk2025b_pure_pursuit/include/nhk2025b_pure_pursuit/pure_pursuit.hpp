#ifndef __pure_pursuit_hpp__
#define __pure_pursuit_hpp__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

namespace pure_pursuit {

class pure_pursuit : public rclcpp::Node {
   public:
    pure_pursuit (const rclcpp::NodeOptions &options);

   private:
    // パラメータ
    double lookahead_time_;            // 速度スケーリング用の時間 [s]
    double min_lookahead_distance_;    // 最小lookahead距離 [m]
    double max_lookahead_distance_;    // 最大lookahead距離 [m]
    double angle_lookahead_distance_;    // 角度スケーリング用の距離 [m]
    double angle_p_;                   // 角度ゲイン
    double max_speed_xy_m_s_;          // 最大並進速度
    double max_speed_z_rad_s_;         // 最大回転速度
    double max_acceleration_xy_m_s2_;  // 最大加速度
    double lookahead_distance_;        // 現在のlookahead距離 [m]（動的に計算される）

    // 入力データ
    geometry_msgs::msg::PoseStamped  current_pose_;
    geometry_msgs::msg::TwistStamped last_cmd_vel_;
    nav_msgs::msg::Path              path_;

    // コールバック
    void timer_callback ();
    void pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void path_callback (const nav_msgs::msg::Path::SharedPtr msg);

    // ROS2通信
    rclcpp::TimerBase::SharedPtr                                     timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr   cmd_vel_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    lookahead_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    lookahead2_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr             path_subscriber_;
};

}  // namespace pure_pursuit

#endif  // __pure_pursuit_hpp__
