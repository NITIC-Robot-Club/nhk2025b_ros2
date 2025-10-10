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
#include <visualization_msgs/msg/marker.hpp>

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

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr      gate_pose_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gate_placement_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   lidar_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             is_red_subscriber_;
    double                                                           distance_threshold;
    int                                                              iter;
    double                                                           detection_width, detection_length;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_left_detection_area_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_right_detection_area_;

    void                               pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void                               lidar_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void                               is_red_callback (const std_msgs::msg::Bool::SharedPtr is_red);
    geometry_msgs::msg::PoseStamped    gate_placement_pose_;
    sensor_msgs::msg::PointCloud2      point_cloud;
    bool                               is_red_;
    double                             min_x, max_x, min_y, max_y;
    int                                detection_count_ = 0;
    std::vector<Point>                 cloud_to_points (const sensor_msgs::msg::PointCloud2 &cloud);
    std::tuple<double, double, double> ransac (const std::vector<Point> &points, std::vector<Point> &inliers_out);
    Line                               ransac_line (const std::vector<Point> &points, std::vector<Point> &inliers_out);
    Point                              line_intersection (const Line &l1, const Line &l2);
    double                             point_line_distance (const Point &pt, double a, double b, double c);
    Point             compute_midpoint (const std::vector<Point> &intersections);
    std::vector<Point>                 filtering_points (const std::vector<Point> &data, const std::vector<Point> &polygon);
    bool                               isInsidePolygon (const Point &pt, const std::vector<Point> &polygon);
    std::vector<Point>                 get_robot_backward_area (const geometry_msgs::msg::PoseStamped &pose, double width, double length);
    std::vector<Point>                 get_rear_side_detection_area (const geometry_msgs::msg::PoseStamped &pose, double width, double length, bool right_side);
    Point             closest_endpoint_to_robot (const geometry_msgs::msg::PoseStamped &robot_pose, const Line &line);
};
}  // namespace gate_perception

#endif  //__gate_perception_HPP__