#include "nhk2025b_simulation/lidar_simulation.hpp"

namespace lidar_simulation {

lidar_simulation::lidar_simulation (const rclcpp::NodeOptions &options)
    : Node ("lidar_simulation", options), tf_buffer (this->get_clock ()), tf_listener (tf_buffer) {
    pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/localization/pose", 10, std::bind (&lidar_simulation::pose_callback, this, std::placeholders::_1));
    map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid> (
        "/map", 10, std::bind (&lidar_simulation::map_callback, this, std::placeholders::_1));
    laser_publisher = this->create_publisher<sensor_msgs::msg::LaserScan> ("/scan", rclcpp::SensorDataQoS ());
    timer           = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&lidar_simulation::timer_callback, this));
}

void lidar_simulation::map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map = *msg;
}

void lidar_simulation::timer_callback () {
    if (current_map.data.empty ()) {
        RCLCPP_WARN (this->get_logger (), "Map is empty");
        return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer.lookupTransform ("lidar", "map", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN (this->get_logger (), "Could not get transform: %s", ex.what ());
        return;
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp    = this->now ();
    scan.header.frame_id = "lidar";
    scan.angle_min       = -1.57;
    scan.angle_max       = 1.57;
    scan.angle_increment = 0.0174533;  // 1 degree
    scan.time_increment  = 0.0;
    scan.scan_time       = 0.1;
    scan.range_min       = 0.0;
    scan.range_max       = 10.0;
    scan.ranges.resize (360, 10.0);
    scan.intensities.resize (360, 1.0);

    double start_x = transform.transform.translation.x;
    double start_y = transform.transform.translation.y;
    for (size_t i = 0; i < 3600; ++i) {
        double angle = scan.angle_min + i * scan.angle_increment + std::asin (transform.transform.rotation.z) * 2.0;
        for (double r = 0.1; r < scan.range_max; r += 0.1) {
            double x = start_x + r * std::cos (angle);
            double y = start_y + r * std::sin (angle);

            int map_x = x / current_map.info.resolution;
            int map_y = y / current_map.info.resolution;
            if (map_x < 0 || map_x >= current_map.info.width || map_y < 0 || map_y >= current_map.info.height) {
                continue;
            }
            int index = map_x + map_y * current_map.info.width;
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