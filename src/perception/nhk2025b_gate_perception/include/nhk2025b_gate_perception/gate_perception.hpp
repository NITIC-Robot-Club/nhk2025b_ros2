#ifndef __gate_perception_HPP__
#define __gate_perception_HPP__

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace gate_perception {
class gate_perception : public rclcpp::Node {
   public:
    gate_perception (const rclcpp::NodeOptions &options);

   private:
    using Point = std::pair<double, double>;
    struct Line {
        double a, b, c;
        Point  start, end;
    };
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gate_placement_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   lidar_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr      gate_detection_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr      side_detection_area_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr      centre_detection_area_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr      gate_pose_publisher_;
    double                                                           distance_threshold;
    int                                                              iter;

    void                               pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void                               lidar_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    geometry_msgs::msg::PoseStamped    gate_placement_pose_;
    sensor_msgs::msg::PointCloud2      point_cloud;
    double                             gate_detection_width, gate_detection_length;
    std::vector<Point>                 gate_detection_area_points, side_detection_area_points, centre_detection_area_points;
    std::vector<std::vector<Point>>    detected_areas_;
    double                             get_yaw_2d (const geometry_msgs::msg::PoseStamped &q);
    std::vector<Point>                 cloud_to_points (const sensor_msgs::msg::PointCloud2 &cloud);
    std::vector<Point>                 get_gate_detection_area (const geometry_msgs::msg::PoseStamped &pose, double width, double length);
    std::vector<Point>                 get_side_detection_area (const geometry_msgs::msg::PoseStamped &pose, double width, double length, bool right_side);
    void                               publish_points_pose_array (const std::vector<Point> points);
    std::tuple<double, double, double> ransac (const std::vector<Point> &points, std::vector<Point> &inliers_out);
    Line                               ransac_line (const std::vector<Point> &points, std::vector<Point> &inliers_out);
    std::vector<Point>                 remove_detected_points (const std::vector<Point> &points, const std::vector<std::vector<Point>> &detected_points);
    Point                              line_intersection (const Line &l1, const Line &l2);
    double                             point_line_distance (const Point &pt, double a, double b, double c);
    gate_perception::Point             compute_midpoint (const std::vector<Point> &intersections);
};
}  // namespace gate_perception

#endif  //__gate_perception_HPP__