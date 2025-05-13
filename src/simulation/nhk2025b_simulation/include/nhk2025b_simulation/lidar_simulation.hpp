#ifndef __lidar_simulation_hpp__
#define __lidar_simulation_hpp__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace lidar_simulation {
class lidar_simulation : public rclcpp::Node {
   public:
    lidar_simulation (const rclcpp::NodeOptions &options);

   private:
    double lidar_x, lidar_y, lidar_z;
    double lidar_frequency;

    nav_msgs::msg::OccupancyGrid    current_map;
    geometry_msgs::msg::PoseStamped current_pose;
    rclcpp::TimerBase::SharedPtr    timer;

    void timer_callback ();
    void map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr        laser_publisher;
};
}  // namespace lidar_simulation

#endif  //__lidar_simulation_hpp__