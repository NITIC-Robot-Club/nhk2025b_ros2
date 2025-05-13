#ifndef __pose_initializer_hpp__
#define __pose_initializer_hpp__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cmath>
#include <limits>
#include <random>
#include <tuple>
#include <vector>

namespace pose_initializer {
class pose_initializer : public rclcpp::Node {
   public:
    explicit pose_initializer (const rclcpp::NodeOptions& options);

   private:
    using Point = std::pair<double, double>;

    void lidar_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // RANSACによる直線フィッティング
    std::tuple<double, double, double> ransac_line_fit (const std::vector<Point>& points, std::vector<Point>& inliers_out);

    std::optional<Point> compute_intersection (const std::tuple<double, double, double>& line1, const std::tuple<double, double, double>& line2);

    std::tuple<double, double, double> compute_yaw_and_position (
        const std::tuple<double, double, double>& line1, const std::tuple<double, double, double>& line2, const Point& intersection);

    double point_line_distance (const Point& pt, double a, double b, double c);
    bool   is_horizontal (const std::tuple<double, double, double>& line);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                lidar_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher;
};
}  // namespace pose_initializer

#endif  // __pose_initializer_hpp__