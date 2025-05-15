#ifndef __path_planner_HPP__
#define __path_planner_HPP__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include <queue>

namespace path_planner {
class path_planner : public rclcpp::Node {
   public:
    path_planner (const rclcpp::NodeOptions& options);

   private:
    const std::vector<std::pair<int, int>> directions = {
        { 1,  0},
        {-1,  0},
        { 0,  1},
        { 0, -1},
        { 1,  1},
        {-1, -1},
        { 1, -1},
        {-1,  1}
    };
    struct astar_node {
        int    x, y;
        double cost, priority;
        bool   operator> (const astar_node& other) const {
              return priority > other.priority;
        }
    };
    int    resolution_ms;
    int    offset_mm;
    int    robot_size_mm;
    int    tolerance_xy_mm;
    double tolerance_z_rad;
    double sigmoid_gain;

    int    map_width, map_height;
    double map_resolution;

    void current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void goal_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void vel_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void find_freespace (std::pair<int, int>& point);
    void timer_callback ();

    void inflate_map ();
    void astar (nav_msgs::msg::Path& path);

    geometry_msgs::msg::PoseStamped                                   current_pose;
    geometry_msgs::msg::PoseStamped                                   goal_pose;
    geometry_msgs::msg::PoseStamped                                   safe_goal_pose;
    nav_msgs::msg::OccupancyGrid                                      original_map;
    nav_msgs::msg::OccupancyGrid                                      inflated_map;
    geometry_msgs::msg::TwistStamped                                  current_vel;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                 path_publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  current_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  goal_pose_subscriber;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr     map_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_subscriber;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr        inflate_map_publisher;
    rclcpp::TimerBase::SharedPtr                                      timer_;
};
}  // namespace path_planner

#endif  //__path_planner_HPP__