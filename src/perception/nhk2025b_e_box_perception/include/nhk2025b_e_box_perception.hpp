#ifndef __e_box_perception_hpp__
#define __e_box_perception_hpp__

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nhk2025b_msgs/msg/box.hpp>
#include <nhk2025b_msgs/msg/box_array.hpp>
#include <nhk2025b_msgs/msg/box_info.hpp>
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

namespace e_box_perception {
class e_box_perception : public rclcpp::Node {
   public:
    e_box_perception (const rclcpp::NodeOptions &options);

   private:
    using Point = std::pair<double, double>;
    struct Line {
        double a, b, c;
        Point  start, end;
    };
    std::shared_ptr<tf2_ros::TransformListener>                      tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer>                                 tf_buffer_;
    rclcpp::Publisher<nhk2025b_msgs::msg::BoxArray>::SharedPtr       box_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    e_collect_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    test_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr      pose_array_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr e_drop_pose_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   lidar_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             is_red_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    detection_area_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    centre_of_detected_line_;

    int    iter;
    double distance_threshold;
    bool   is_red_;
    double min_x, max_x, min_y, max_y;
    Point  fl, fr, bl, br;
    double normal_distance;
    double front_line_length, side_line_length;
    int    detection_count_ = 0;
    double permissible_segment_distance;

    std::vector<Point>              detection_areas_;
    std::vector<std::vector<Point>> detected_areas_;
    geometry_msgs::msg::PoseStamped e_drop_pose_;
    nhk2025b_msgs::msg::BoxArray    box_array_;
    sensor_msgs::msg::PointCloud2   lidar_data_;
    geometry_msgs::msg::PoseStamped e_box_normal_;
    geometry_msgs::msg::PoseStamped robot_pose_;
    std::vector<Point>              filtered_points_;

    std::vector<Point>                 cloud_to_points (const sensor_msgs::msg::PointCloud2 &cloud);
    std::tuple<double, double, double> ransac (const std::vector<Point> &points, std::vector<Point> &inliers_out);
    std::vector<Point>                 filtering_points (std::vector<e_box_perception::Point> data, const std::vector<Point> &polygon);
    std::vector<Point>                 remove_detected_points (const std::vector<Point> &points, const std::vector<std::vector<Point>> &detected_points);

    void   pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr pose);
    void   lidar_callback (const sensor_msgs::msg::PointCloud2::SharedPtr lidar);
    void   is_red_callback (const std_msgs::msg::Bool::SharedPtr is_red);
    void   current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr current_pose);
    Point  find_box_centre (e_box_perception::Line line, e_box_perception::Point point, double normal_distance);
    Point  find_collect_point (e_box_perception::Line line, e_box_perception::Point point, double normal_distance);
    double point_line_distance (const Point &pt, double a, double b, double c);
    void   update_detection_area (const geometry_msgs::msg::PoseStamped &pose);
    Line   ransac_line (const std::vector<Point> &points, std::vector<Point> &inliers_out);
    Point  line_midpoint (e_box_perception::Line line);
    void   get_robot_forward_area (const geometry_msgs::msg::PoseStamped &pose);
    bool   isInsidePolygon (const Point &pt, const std::vector<Point> &polygon);
    double lineLength (const Line &line, const std::vector<Point> &inliers);
};
}  // namespace e_box_perception

#endif  // __e_box_perception_hpp__