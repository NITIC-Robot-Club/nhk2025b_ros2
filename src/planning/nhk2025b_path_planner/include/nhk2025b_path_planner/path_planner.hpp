#ifndef __path_planner_HPP__
#define __path_planner_HPP__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

namespace path_planner {
class path_planner : public rclcpp::Node {
   public:
    path_planner (const rclcpp::NodeOptions& options);

   private:
    int    resolution_ms;
    int    offset_mm;
    int    robot_size_mm;
    double max_xy_acceleration_m_s2;
    double max_xy_velocity_m_s;
    double max_z_acceleration_rad_s2;
    double max_z_velocity_rad_s;

    void current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void goal_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void vel_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void timer_callback ();

    geometry_msgs::msg::PoseStamped                                   current_pose;
    geometry_msgs::msg::PoseStamped                                   goal_pose;
    nav_msgs::msg::OccupancyGrid                                      map;
    geometry_msgs::msg::TwistStamped                                  current_vel;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                 path_publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  current_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  goal_pose_subscriber;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr     map_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_subscriber;
    rclcpp::TimerBase::SharedPtr                                      timer;
};
}  // namespace path_planner

#endif  //__path_planner_HPP__