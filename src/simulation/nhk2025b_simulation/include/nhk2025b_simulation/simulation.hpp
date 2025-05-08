#ifndef __simulation_hpp__
#define __simulation_hpp__

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nhk2025b_msgs/msg/robot_status.hpp>
#include <nhk2025b_msgs/msg/swerve.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace simulation {
class simulation : public rclcpp::Node {
   public:
    simulation (const rclcpp::NodeOptions& options);

   private:
    void timer_callback ();
    void swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg);

    double wheel_radius;
    double wheel_position;
    double x_sum_, y_sum_, z_sum_;
    double x_vec_, y_vec_;
    int    count_;
    bool   sig_;
    double x_, y_, z_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<nhk2025b_msgs::msg::Swerve>::SharedPtr      swerve_publisher_;
    rclcpp::Publisher<nhk2025b_msgs::msg::RobotStatus>::SharedPtr robot_status_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr           imu_publisher_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Swerve>::SharedPtr   swerve_subscriber_;
};
}  // namespace simulation

#endif  //__simulation_hpp__