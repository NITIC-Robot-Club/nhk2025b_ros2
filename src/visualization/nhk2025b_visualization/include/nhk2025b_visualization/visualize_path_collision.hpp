#ifndef __visualize_path_collision_hpp__
#define __visualize_path_collision_hpp__

#include <nhk2025b_utils/get_yaw_2d.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <deque>
#include <vector>

namespace visualize_path_collision {
class visualize_path_collision : public rclcpp::Node {
   public:
    visualize_path_collision (const rclcpp::NodeOptions &options);

   private:
    nav_msgs::msg::OccupancyGrid map;
    nav_msgs::msg::Path          path;

    double robot_width = 1.0;
    double robot_length;
    void   path_callback (const nav_msgs::msg::Path::SharedPtr msg);
    void   timer_callback ();
    void   map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    bool   is_hit (const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr               path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr      map_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr            robot_width_sub_;
    rclcpp::TimerBase::SharedPtr                                       timer_;
};
}  // namespace visualize_path_collision

#endif  //__visualize_path_collision_hpp__