#ifndef __path_planner_HPP__
#define __path_planner_HPP__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>

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
    const std::vector<int> rotations = {-1, 0, 1};

    struct astar_node {
        int    x, y;
        double cost, priority;
        bool   operator> (const astar_node& other) const {
            return priority > other.priority;
        }
    };
    int    theta_resolution;
    int    resolution_ms;
    int    penalty_mm;
    int    offset_mm;
    double robot_width = 1.0;
    double robot_length;
    int    tolerance_xy_mm;
    double tolerance_z_rad;

    double grad_alpha;      // 障害物回避の重み
    double grad_beta;       // 曲率項の重み
    double grad_gamma;      // 元のパスからの引き戻し項の重み
    double grad_step_size;  // 勾配降下のステップサイズ

    int    map_width, map_height;
    double map_resolution;

    bool is_map_changed = true;

    void   current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void   goal_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void   map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void   vel_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void   create_path ();
    void   find_freespace (std::pair<int, int>& point, int theta, const std::vector<std::vector<int8_t>>& use_inflated_map);
    void   timer_callback ();
    void   inflate_map ();
    void   init_rotated_footprint ();
    bool   is_collision (int x, int y, int theta, const std::vector<std::vector<int8_t>>& use_inflated_map);
    bool   is_same_map ();
    double theta_heuristic (int dx, int theta);
    void   angular_astar (
          nav_msgs::msg::Path& path, const nav_msgs::msg::Path& smoothed_path, const geometry_msgs::msg::PoseStamped& use_current_pose, const geometry_msgs::msg::PoseStamped& use_goal_pose, const std::vector<std::vector<int8_t>>& use_inflated_map);

    nav_msgs::msg::Path linear_astar (const geometry_msgs::msg::PoseStamped& use_current_pose, const geometry_msgs::msg::PoseStamped& use_goal_pose, const std::vector<std::vector<int8_t>>& use_inflated_map);
    nav_msgs::msg::Path path_smoother (const nav_msgs::msg::Path& linear_path, const nav_msgs::msg::OccupancyGrid& use_occ_map);

    std::vector<std::pair<int, double>> angular_smoother (std::vector<std::pair<int, double>> theta_path);
    std::pair<int, int> to_grid (double x, double y);

    double get_yaw_2d (const geometry_msgs::msg::Quaternion& orientation) {
        return std::atan2 (2.0 * (orientation.z * orientation.w), 1.0 - 2.0 * (orientation.z * orientation.z));
    }
    std::vector<std::vector<int8_t>> inflated_map;
    std::vector<std::vector<int8_t>> angle_cost_map;

    std::vector<std::array<std::pair<int, int>, 4>> rotated_footprint;

    int angle_to_index (int deg) {
        if (deg < 0) deg += 360;
        return deg / theta_resolution;
    }
    int rad_to_deg (double rad) {
        return static_cast<int> (rad * 180.0 / M_PI);
    }
    geometry_msgs::msg::PoseStamped  current_pose;
    geometry_msgs::msg::PoseStamped  goal_pose;
    geometry_msgs::msg::PoseStamped  safe_goal_pose;
    nav_msgs::msg::OccupancyGrid     original_map;
    nav_msgs::msg::OccupancyGrid     last_map;
    nav_msgs::msg::OccupancyGrid     occ_map;
    nav_msgs::msg::Path              send_path;
    geometry_msgs::msg::TwistStamped current_vel;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                 path_publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  current_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  goal_pose_subscriber;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr     map_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           robot_width_subscriber;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr        inflate_map_publisher;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr        theta_map_publisher;
    rclcpp::TimerBase::SharedPtr                                      timer_;
};
}  // namespace path_planner

#endif  //__path_planner_HPP__