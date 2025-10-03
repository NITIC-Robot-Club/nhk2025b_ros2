#ifndef __visualize_footprint_hpp__
#define __visualize_footprint_hpp__

#include <nhk2025b_utils/get_yaw_2d.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <deque>
#include <vector>

namespace visualize_footprint {
class visualize_footprint : public rclcpp::Node {
   public:
    visualize_footprint (const rclcpp::NodeOptions &options);

   private:
    std::deque<visualization_msgs::msg::Marker> marker_history;
    nav_msgs::msg::OccupancyGrid                map;

    double robot_width = 1.0;
    double robot_length;
    int    history;
    void   pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void   map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    bool   is_hit (const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr   pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr      map_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr            robot_width_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};
}  // namespace visualize_footprint

#endif  //__visualize_footprint_hpp__