#ifndef __pure_pursuit_hpp__
#define __pure_pursuit_hpp__

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
namespace pure_pursuit {
class pure_pursuit : public rclcpp::Node {
   public:
    pure_pursuit (const rclcpp::NodeOptions &options);

   private:
    double lookahead_distance_, angle_p_;

    geometry_msgs::msg::PoseStamped current_pose_;
    nav_msgs::msg::Path             path_;

    void timer_callback ();
    void pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void path_callback (const nav_msgs::msg::Path::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr   cmd_vel_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr             path_subscriber_;
};
}  // namespace pure_pursuit

#endif  //__pure_pursuit_hpp__