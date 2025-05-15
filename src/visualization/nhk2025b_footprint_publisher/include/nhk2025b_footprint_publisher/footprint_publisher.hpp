#ifndef __footprint_publisher_hpp__
#define __footprint_publisher_hpp__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <deque>
#include <vector>

namespace footprint_publisher {
class footprint_publisher : public rclcpp::Node {
   public:
    footprint_publisher (const rclcpp::NodeOptions &options);

   private:
    std::deque<visualization_msgs::msg::Marker> marker_history;
    nav_msgs::msg::OccupancyGrid                map;

    double robot_width, robot_length;
    void   pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void   map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    bool   is_hit (const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr   pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr      map_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};
}  // namespace footprint_publisher

#endif  //__footprint_publisher_hpp__