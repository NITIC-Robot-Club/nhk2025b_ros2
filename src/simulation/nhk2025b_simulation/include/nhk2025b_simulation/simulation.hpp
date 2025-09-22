#ifndef __simulation_hpp__
#define __simulation_hpp__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nhk2025b_msgs/msg/robot_status.hpp>
#include <nhk2025b_msgs/msg/swerve.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

namespace simulation {
class simulation : public rclcpp::Node {
   public:
    simulation (const rclcpp::NodeOptions& options);

   private:
    void timer_callback ();
    void swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg);
    void is_red_callback (const std_msgs::msg::Bool::SharedPtr msg);

    double wheel_radius;
    double robot_width, robot_length;
    double x_sum_, y_sum_, z_sum_;
    double x_vec_, y_vec_;
    int    count_;
    bool   sig_;
    double x_, y_, z_;
    bool   is_red, last_is_red;
    double initial_x_blue, initial_y_blue;
    double initial_x_red, initial_y_red;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<nhk2025b_msgs::msg::Swerve>::SharedPtr      swerve_publisher_;
    rclcpp::Publisher<nhk2025b_msgs::msg::RobotStatus>::SharedPtr robot_status_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr           imu_publisher_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Swerve>::SharedPtr   swerve_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          is_red_subscriber_;
};
}  // namespace simulation

#endif  //__simulation_hpp__