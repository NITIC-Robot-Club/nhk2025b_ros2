#include "nhk2025b_simulation/lidar_simulation.hpp"

namespace lidar_simulation {

lidar_simulation::lidar_simulation (const rclcpp::NodeOptions &options) : Node ("lidar_simulation", options) {
    map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid> (
        "/behavior/map", 10, std::bind (&lidar_simulation::map_callback, this, std::placeholders::_1));
    pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/simulation/pose", 10, std::bind (&lidar_simulation::pose_callback, this, std::placeholders::_1));
    laser_publisher = this->create_publisher<sensor_msgs::msg::LaserScan> ("/sensor/scan", rclcpp::SensorDataQoS ());
    lidar_x         = this->declare_parameter<double> ("lidar_x", 0);
    lidar_y         = this->declare_parameter<double> ("lidar_y", 0);
    lidar_z         = this->declare_parameter<double> ("lidar_z", 0);
    lidar_frequency = this->declare_parameter<double> ("lidar_frequency", 12.0);
    timer = this->create_wall_timer (std::chrono::milliseconds (int (1000 / lidar_frequency)), std::bind (&lidar_simulation::timer_callback, this));
}

void lidar_simulation::map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map = *msg;
}

void lidar_simulation::pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose = *msg;
}

void lidar_simulation::timer_callback () {
    if (current_map.data.empty ()) {
        RCLCPP_WARN (this->get_logger (), "Map is empty");
        return;
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp    = this->now ();
    scan.header.frame_id = "lidar";
    scan.angle_min       = -1.57;
    scan.angle_max       = 1.57;
    scan.angle_increment = lidar_frequency / 13.953488372093023;
    scan.time_increment  = 0.0;
    scan.scan_time       = 1.0 / lidar_frequency;
    scan.range_min       = 0.12;
    scan.range_max       = 11.0;
    scan.ranges.resize (360 / scan.angle_increment, scan.range_max);
    scan.intensities.resize (360 / scan.angle_increment, 1.0);

    double yaw     = get_yaw_2d (current_pose.pose.orientation);
    double start_x = current_pose.pose.position.x + lidar_x * std::cos (yaw) - lidar_y * std::sin (yaw);
    double start_y = current_pose.pose.position.y + lidar_x * std::sin (yaw) + lidar_y * std::cos (yaw);
    for (size_t i = 0; i < scan.ranges.size (); ++i) {  // Ensure we iterate within bounds
        double angle = scan.angle_min + i * scan.angle_increment + yaw + lidar_z;
        for (double r = 0.12; r < scan.range_max; r += 0.01) {
            double x = start_x + r * std::cos (angle);
            double y = start_y + r * std::sin (angle);

            int map_x = static_cast<int> (x / current_map.info.resolution);
            int map_y = static_cast<int> (y / current_map.info.resolution);
            if (map_x < 0 || map_x >= static_cast<int> (current_map.info.width) || map_y < 0 || map_y >= static_cast<int> (current_map.info.height)) {
                continue;
            }
            int index = map_x + map_y * current_map.info.width;
            if (index < 0 || index >= static_cast<int> (current_map.data.size ())) {
                continue;  // Ensure index is within bounds
            }
            if (current_map.data[index] > 0) {
                scan.ranges[i]      = r;
                scan.intensities[i] = 1;
                break;
            }
        }
    }

    laser_publisher->publish (scan);
}

}  // namespace lidar_simulation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (lidar_simulation::lidar_simulation);