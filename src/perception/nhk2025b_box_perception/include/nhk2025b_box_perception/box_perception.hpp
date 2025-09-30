#ifndef __box_perception_hpp__
#define __box_perception_hpp__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nhk2025b_msgs/msg/box.hpp>
#include <nhk2025b_msgs/msg/box_array.hpp>
#include <nhk2025b_msgs/msg/line.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace box_perception {

struct Cluster {
    std::vector<geometry_msgs::msg::Point32> points;
    geometry_msgs::msg::Point32              centroid;
    float                                    min_x, max_x, min_y, max_y;
};

class box_perception : public rclcpp::Node {
   public:
    box_perception (const rclcpp::NodeOptions& options);

   private:
    rclcpp::Publisher<nhk2025b_msgs::msg::BoxArray>::SharedPtr     box_state_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Subscription<nhk2025b_msgs::msg::BoxArray>::SharedPtr  box_array_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr  map_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           is_red_subscriber_;

    // functions
    void                 point_cloud_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void                 box_array_callback (const nhk2025b_msgs::msg::BoxArray::SharedPtr msg);
    void                 map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void                 is_red_callback (const std_msgs::msg::Bool::SharedPtr msg);
    std::vector<Cluster> cluster_points (const sensor_msgs::msg::PointCloud2& cloud, float tolerance);
    uint32_t                 assign_label (float width, float depth);
    void                 match_clusters (const std::vector<Cluster>& prev, const std::vector<Cluster>& curr, std::vector<bool>& moved, float move_thresh);

    nhk2025b_msgs::msg::BoxArray                prev_box_array_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
};
}  // namespace box_perception

#endif  //__box_perception_hpp__