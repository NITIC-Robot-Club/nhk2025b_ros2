#ifndef __path_planner_HPP__
#define __path_planner_HPP__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include <queue>

namespace path_planner {
class path_planner : public rclcpp::Node {
   public:
    path_planner (const rclcpp::NodeOptions& options);

   private:
    const std::vector<std::pair<int, int>> directions = {
        { 1,  0},
        {-1,  0},
        { 0,  1},
        { 0, -1},
        { 1,  1},
        {-1, -1},
        { 1, -1},
        {-1,  1}
    };
    const std::vector<int> rotations = {-2, -1, 0, 1, 2};

    struct astar_node {
        int    x, y;
        double cost, priority;
        bool   operator> (const astar_node& other) const {
            return priority > other.priority;
        }
    };
    int    theta_resolution;
    int    resolution_ms;
    int    offset_mm;
    int    robot_height_mm;
    int    robot_width_mm;
    int    tolerance_xy_mm;
    double tolerance_z_rad;
    double sigmoid_gain;

    double grad_alpha;      // 障害物回避の重み
    double grad_beta;       // 曲率項の重み
    double grad_gamma;      // 元のパスからの引き戻し項の重み
    double grad_step_size;  // 勾配降下のステップサイズ

    int    map_width, map_height;
    double map_resolution;

    int test = 0;

    void   current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void   goal_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void   map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void   vel_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void   find_freespace (std::pair<int, int>& point);
    void   timer_callback ();
    void   linear_astar ();
    void   angular_astar (nav_msgs::msg::Path& path);
    void   path_smoother ();
    void   inflate_map ();
    void   init_rotated_footprint ();
    bool   is_collision (int x, int y, int theta);
    double theta_heuristic (int dx, int theta);

    double get_yaw_2d (const geometry_msgs::msg::Quaternion& orientation) {
        return std::atan2 (2.0 * (orientation.z * orientation.w), 1.0 - 2.0 * (orientation.z * orientation.z));
    }
    std::vector<std::vector<int8_t>> inflated_map;
    std::vector<std::vector<int8_t>> offset_map;
    // std::vector<std::pair<int, int>>                linear_path;

    std::vector<std::array<std::pair<int, int>, 4>> rotated_footprint;

    int angle_to_index (int deg) {
        if (deg < 0) deg += 360;
        return deg / theta_resolution;
    }
    int rad_to_deg (double rad) {
        return static_cast<int> (rad * 180.0 / M_PI);
    }
    geometry_msgs::msg::PoseStamped                                   current_pose;
    geometry_msgs::msg::PoseStamped                                   goal_pose;
    geometry_msgs::msg::PoseStamped                                   safe_goal_pose;
    nav_msgs::msg::OccupancyGrid                                      original_map;
    nav_msgs::msg::OccupancyGrid                                      last_map;
    nav_msgs::msg::OccupancyGrid                                      occ_map;
    nav_msgs::msg::Path                                               linear_path;
    nav_msgs::msg::Path                                               smoothed_path;
    geometry_msgs::msg::TwistStamped                                  current_vel;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                 path_publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  current_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  goal_pose_subscriber;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr     map_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_subscriber;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr        inflate_map_publisher;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr        theta_map_publisher;
    rclcpp::TimerBase::SharedPtr                                      timer_;
};
}  // namespace path_planner

#endif  //__path_planner_HPP__