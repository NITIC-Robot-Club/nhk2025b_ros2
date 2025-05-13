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
    double lookahead_p_, angle_p_;
    double max_speed_xy_m_s_;
    double max_speed_z_rad_s_;
    double max_acceleration_xy_m_s2_;
    double lookahead_distance_;

    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::TwistStamped   last_cmd_vel_;
    nav_msgs::msg::Path             path_;

    void timer_callback ();
    void pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void path_callback (const nav_msgs::msg::Path::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr   cmd_vel_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    lookahead_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr             path_subscriber_;
};
}  // namespace pure_pursuit

#endif  //__pure_pursuit_hpp__