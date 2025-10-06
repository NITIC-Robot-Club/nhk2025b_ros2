#ifndef __pure_pursuit_hpp__
#define __pure_pursuit_hpp__

#include "nhk2025b_utils/get_yaw_2d.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nhk2025b_msgs/msg/control_limit.hpp>

namespace pure_pursuit {

class pure_pursuit : public rclcpp::Node {
   public:
    pure_pursuit (const rclcpp::NodeOptions &options);

   private:
    // パラメータ
    double lookahead_time_;                // 速度スケーリング用の時間 [s]
    double min_lookahead_distance_;        // 最小lookahead距離 [m]
    double max_lookahead_distance_;        // 最大lookahead距離 [m]
    double angle_speed_p_;                 // 角度比例ゲイン
    double curvature_decceleration_p_;     // 曲率減速用の比例ゲイン
    double min_curvature_speed_m_s_;       // 曲率減速用の最低速度 [m/s]
    double angle_decceleration_p_;         // 角度減速用の比例ゲイン
    double max_speed_xy_m_s_;              // 最大並進速度
    double min_speed_xy_m_s_;              // 最小並進速度
    double max_speed_z_deg_s_;             // 最大回転速度
    double min_speed_z_deg_s_;             // 最小回転速度
    double max_acceleration_xy_m_s2_;      // 最大加速度
    double max_acceleration_z_deg_s2_;     // 最大角加速度 [deg/s^2]
    double goal_deceleration_m_s2_;        // ゴール減速用の減速度 [m/s^2]
    double goal_position_tolerance_;       // ゴール位置許容誤差 [m]
    double goal_yaw_tolerance_deg_;        // ゴールヨー許容誤差 [deg]
    double goal_speed_tolerance_xy_m_s_;   // ゴール速度許容誤差 [m/s]
    double goal_speed_tolerance_z_deg_s_;  // ゴール速度許容誤差 [deg/s]

    double lookahead_distance_;  // 現在のlookahead距離 [m]（動的に計算される）
    // 入力データ
    geometry_msgs::msg::PoseStamped  current_pose_;
    geometry_msgs::msg::TwistStamped last_cmd_vel_;
    nav_msgs::msg::Path              path_;

    // コールバック
    void timer_callback ();
    void pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void path_callback (const nav_msgs::msg::Path::SharedPtr msg);

    // ROS2通信
    rclcpp::TimerBase::SharedPtr                                      timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr    cmd_vel_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     lookahead_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr              path_subscriber_;
    rclcpp::Subscription<nhk2025b_msgs::msg::ControlLimit>::SharedPtr control_limit_subscriber_;
};

}  // namespace pure_pursuit

#endif  // __pure_pursuit_hpp__
