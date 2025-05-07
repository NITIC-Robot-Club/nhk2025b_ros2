#ifndef __lidar_simulation_hpp__
#define __lidar_simulation_hpp__

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>


namespace lidar_simulation {
class lidar_simulation : public rclcpp::Node {
   public:
    lidar_simulation (const rclcpp::NodeOptions &options);

   private:

    geometry_msgs::msg::PoseStamped current_pose;
    nav_msgs::msg::OccupancyGrid    current_map;
    rclcpp::TimerBase::SharedPtr    timer;

    void timer_callback ();
    void pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr        laser_publisher;
};
}  // namespace lidar_simulation

#endif  //__lidar_simulation_hpp__