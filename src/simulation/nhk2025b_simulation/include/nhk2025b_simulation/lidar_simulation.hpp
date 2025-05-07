#ifndef __lidar_simulation_hpp__
#define __lidar_simulation_hpp__

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace lidar_simulation {
class lidar_simulation : public rclcpp::Node {
   public:
    lidar_simulation (const rclcpp::NodeOptions &options);

   private:
    nav_msgs::msg::OccupancyGrid    current_map;
    rclcpp::TimerBase::SharedPtr    timer;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    void timer_callback ();
    void map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr        laser_publisher;
};
}  // namespace lidar_simulation

#endif  //__lidar_simulation_hpp__