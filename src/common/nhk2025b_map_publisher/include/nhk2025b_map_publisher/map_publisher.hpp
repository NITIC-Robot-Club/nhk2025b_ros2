#ifndef __map_publisher_hpp__
#define __map_publisher_hpp__

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace map_publisher {
class map_publisher : public rclcpp::Node {
   public:
    map_publisher (const rclcpp::NodeOptions& options);

   private:
    void   publish_map ();
    bool   is_red;
    double resolution_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr                               timer_;
};
}  // namespace map_publisher

#endif  //__map_publisher_hpp__