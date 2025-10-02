#ifndef __e_box_perception_hpp__
#define __e_box_perception_hpp__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nhk2025b_msgs/msg/box.hpp>
#include <nhk2025b_msgs/msg/box_array.hpp>
#include <nhk2025b_msgs/msg/box_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace e_box_perception {
class e_box_perception : public rclcpp::Node {
   public:
    e_box_perception (const rclcpp::NodeOptions &options);

   private:
    struct Point {
        double x, y;
    };
    struct Line {
        double a, b;
        Point  start, end;
    };
    rclcpp::Publisher<nhk2025b_msgs::msg::BoxArray>::SharedPtr       box_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    e_collect_pose_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr e_drop_pose_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   lidar_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             is_red_subscriber_;
    int                                                              iter;
    double                                                           distance_threshold;
    bool                                                             is_red_;
    double                                                           min_x, max_x, min_y, max_y;
    double                                                           normal_distance;
    geometry_msgs::msg::PoseStamped                                  e_drop_pose_;
    nhk2025b_msgs::msg::BoxArray                                     box_array_;
    sensor_msgs::msg::PointCloud2                                    lidar_data_;
    geometry_msgs::msg::PoseStamped                                  e_box_normal_;

    void                    pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr pose);
    void                    lidar_callback (const sensor_msgs::msg::PointCloud2::SharedPtr lidar);
    void                    is_red_callback (const std_msgs::msg::Bool::SharedPtr is_red);
    std::vector<Point>      cloud_to_points (const sensor_msgs::msg::PointCloud2 &cloud);
    e_box_perception::Line  ransac (std::vector<Point> data, double line_length);
    std::vector<Point>      filtering_points (std::vector<Point> data);
    e_box_perception::Point line_centre (e_box_perception::Line line);
    e_box_perception::Point normal_point (e_box_perception::Line line, e_box_perception::Point point, double normal_distance);
};
}  // namespace e_box_perception

#endif  // __e_box_perception_hpp__