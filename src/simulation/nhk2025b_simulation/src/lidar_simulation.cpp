#include "nhk2025b_simulation/lidar_simulation.hpp"

namespace lidar_simulation {

lidar_simulation::lidar_simulation (const rclcpp::NodeOptions &options) : Node ("lidar_simulation", options) {
    map_subscriber  = this->create_subscription<nav_msgs::msg::OccupancyGrid> ("/behavior/map", 1, std::bind (&lidar_simulation::map_callback, this, std::placeholders::_1));
    pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> ("/simulation/pose", 1, std::bind (&lidar_simulation::pose_callback, this, std::placeholders::_1));
    laser_publisher = this->create_publisher<sensor_msgs::msg::LaserScan> ("/sensor/scan", 1);
    lidar_x         = this->declare_parameter<double> ("lidar_x", 0);
    lidar_y         = this->declare_parameter<double> ("lidar_y", 0);
    lidar_z         = this->declare_parameter<double> ("lidar_z", 0);
    lidar_frequency = this->declare_parameter<double> ("lidar_frequency", 12.0);
    frame_name      = this->declare_parameter<std::string> ("frame_name", "lidar");
    timer           = this->create_wall_timer (std::chrono::milliseconds (int (1000 / lidar_frequency)), std::bind (&lidar_simulation::timer_callback, this));
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
    scan.header.frame_id = frame_name;

    // --- 新しいパラメータ ---
    double horizontal_fov = 180.0 * M_PI / 180.0;
    double resolution_deg = 0.3;
    double resolution_rad = resolution_deg * M_PI / 180.0;

    scan.angle_min       = -horizontal_fov / 2.0;
    scan.angle_max       = horizontal_fov / 2.0;
    scan.angle_increment = resolution_rad;
    scan.time_increment  = 1.0 / 43200.0;          // 43.2kHz
    scan.scan_time       = 1.0 / lidar_frequency;  // 30Hz

    scan.range_min = 0.12;
    scan.range_max = 50.0;

    size_t num_points = static_cast<size_t> ((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.resize (num_points, scan.range_max);
    scan.intensities.resize (num_points, 1.0);

    double yaw     = nhk2025b_utils::get_yaw_2d (current_pose.pose.orientation);
    double start_x = current_pose.pose.position.x + lidar_x * std::cos (yaw) - lidar_y * std::sin (yaw);
    double start_y = current_pose.pose.position.y + lidar_x * std::sin (yaw) + lidar_y * std::cos (yaw);
    start_x -= current_map.info.origin.position.x;
    start_y -= current_map.info.origin.position.y;

    for (size_t i = 0; i < scan.ranges.size (); ++i) {
        double angle = scan.angle_min + i * scan.angle_increment + yaw + lidar_z;
        for (double r = scan.range_min; r < scan.range_max; r += 0.01) {
            double x = start_x + r * std::cos (angle);
            double y = start_y + r * std::sin (angle);

            int map_x = static_cast<int> (x / current_map.info.resolution);
            int map_y = static_cast<int> (y / current_map.info.resolution);

            if (map_x < 0 || map_x >= static_cast<int> (current_map.info.width) || map_y < 0 || map_y >= static_cast<int> (current_map.info.height)) {
                continue;
            }

            int index = map_x + map_y * current_map.info.width;
            if (index < 0 || index >= static_cast<int> (current_map.data.size ())) {
                continue;
            }

            if (current_map.data[index] > 0) {
                scan.ranges[i]      = r;
                scan.intensities[i] = 1.0;
                break;
            }
        }
    }

    laser_publisher->publish (scan);
}

}  // namespace lidar_simulation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (lidar_simulation::lidar_simulation);