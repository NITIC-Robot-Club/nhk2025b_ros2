#ifndef __lidar_simulation_hpp__
#define __lidar_simulation_hpp__

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace lidar_simulation {
class lidar_simulation : public rclcpp::Node {
   public:
    lidar_simulation (const rclcpp::NodeOptions &options);

   private:


    void timer_callback ();
    void pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher;    
};
}  // namespace lidar_simulation

#endif  //__lidar_simulation_hpp__