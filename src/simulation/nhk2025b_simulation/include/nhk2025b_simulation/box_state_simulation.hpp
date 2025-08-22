#ifndef __box_state_simulation_hpp__
#define __box_state_simulation_hpp__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nhk2025b_msgs/msg/box_array.hpp>

namespace box_state_simulation {
class box_state_simulation : public rclcpp::Node {
   public:
    box_state_simulation (const rclcpp::NodeOptions &options);

   private:
    double box_size_x, box_size_y, box_size_z;

    nhk2025b_msgs::msg::BoxArray current_box_array;

    void point_callback (const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void timer_callback ();

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber;
    rclcpp::Publisher<nhk2025b_msgs::msg::BoxArray>::SharedPtr        box_publisher;
    rclcpp::TimerBase::SharedPtr                                      timer;
};
}  // namespace box_state_simulation

#endif  //__box_state_simulation_hpp__