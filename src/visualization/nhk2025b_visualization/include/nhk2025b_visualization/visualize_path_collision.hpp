#ifndef __visualize_path_collision_hpp__
#define __visualize_path_collision_hpp__

#include <nhk2025b_utils/get_yaw_2d.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <deque>
#include <vector>

namespace visualize_path_collision {
class visualize_path_collision : public rclcpp::Node {
   public:
    visualize_path_collision (const rclcpp::NodeOptions &options);

   private:
    nav_msgs::msg::OccupancyGrid map;

    double robot_width, robot_length;
    void   path_callback (const nav_msgs::msg::Path::SharedPtr msg);
    void   map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    bool   is_hit (const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr   path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr      map_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};
}  // namespace visualize_path_collision

#endif  //__visualize_path_collision_hpp__