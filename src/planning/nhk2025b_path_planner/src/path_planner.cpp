#include "nhk2025b_path_planner/path_planner.hpp"

namespace path_planner {
path_planner::path_planner (const rclcpp::NodeOptions &options) : Node ("path_planner", options) {
    resolution_ms   = this->declare_parameter<int> ("resolution_ms", 100);
    offset_mm       = this->declare_parameter<int> ("offset_mm", 50);
    robot_size_mm   = this->declare_parameter<int> ("robot_size_mm", 1414);
    tolerance_xy_mm = this->declare_parameter<int> ("tolerance_xy_mm", 30);
    tolerance_z_rad = this->declare_parameter<double> ("tolerance_z_rad", 0.03);
    sigmoid_gain    = this->declare_parameter<double> ("sigmoid_gain", 7.5);

    path_publisher          = this->create_publisher<nav_msgs::msg::Path> ("/planning/path", 10);
    current_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/localization/current_pose", 10, std::bind (&path_planner::current_pose_callback, this, std::placeholders::_1));
    goal_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/behavior/goal_pose", 10, std::bind (&path_planner::goal_pose_callback, this, std::placeholders::_1));
    map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid> (
        "/behavior/map", 10, std::bind (&path_planner::map_callback, this, std::placeholders::_1));
    vel_subscriber = this->create_subscription<geometry_msgs::msg::TwistStamped> (
        "/cmd_vel", 10, std::bind (&path_planner::vel_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&path_planner::timer_callback, this));

    inflate_map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid> ("/planning/costmap", 10);
}

void path_planner::timer_callback () {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp    = this->now ();
    double diff_x        = safe_goal_pose.pose.position.x - current_pose.pose.position.x;
    double diff_y        = safe_goal_pose.pose.position.y - current_pose.pose.position.y;
    double distance      = std::hypot (diff_x, diff_y);
    double current_yaw   = get_yaw_2d (current_pose.pose.orientation);
    double goal_yaw      = get_yaw_2d (safe_goal_pose.pose.orientation);
    double delta_yaw     = goal_yaw - current_yaw;
    if (delta_yaw > M_PI)
        delta_yaw -= 2 * M_PI;
    else if (delta_yaw < -M_PI)
        delta_yaw += 2 * M_PI;
    if (distance < tolerance_xy_mm / 1000.0 && std::abs (delta_yaw) < tolerance_z_rad) {
        path_publisher->publish (path);
    }
}
void path_planner::goal_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    msg->pose.position.x = std::clamp (msg->pose.position.x, 0.01, original_map.info.width * original_map.info.resolution - 0.01);
    msg->pose.position.y = std::clamp (msg->pose.position.y, 0.01, original_map.info.height * original_map.info.resolution - 0.01);
    goal_pose            = *msg;

    double clamped_current_x = std::clamp (current_pose.pose.position.x, 0.01, original_map.info.width * original_map.info.resolution - 0.01);
    double clamped_current_y = std::clamp (current_pose.pose.position.y, 0.01, original_map.info.height * original_map.info.resolution - 0.01);
    if (current_pose.pose.position.x != clamped_current_x || current_pose.pose.position.y != clamped_current_y) {
        RCLCPP_WARN (this->get_logger (), "current pose is out of map");
        return;
    }

    if (original_map.header.stamp.sec == 0) return;
    if (current_pose.header.stamp.sec == 0) return;
    if (goal_pose.header.stamp.sec == 0) return;
    nav_msgs::msg::Path   path;
    std_msgs::msg::Header header;
    header.frame_id = "map";
    header.stamp    = this->now ();
    path.header     = header;

    astar (path);

    double current_yaw = get_yaw_2d (current_pose.pose.orientation);
    double goal_yaw    = get_yaw_2d (goal_pose.pose.orientation);
    double delta_yaw   = goal_yaw - current_yaw;
    if (delta_yaw > M_PI)
        delta_yaw -= 2 * M_PI;
    else if (delta_yaw < -M_PI)
        delta_yaw += 2 * M_PI;
    double delta_t = resolution_ms / 1000.0;
    for (int i = 0; i < path.poses.size () - 1; i++) {
        double now_yaw                   = current_yaw + delta_yaw / (1.0 + std::exp (-sigmoid_gain * ((double)i / path.poses.size () - 0.5)));
        path.poses[i].pose.orientation.z = std::sin (now_yaw / 2.0);
        path.poses[i].pose.orientation.w = std::cos (now_yaw / 2.0);
        path.poses[i].header             = header;
    }
    path.poses[path.poses.size () - 1].pose.orientation.z = goal_pose.pose.orientation.z;
    path.poses[path.poses.size () - 1].pose.orientation.w = goal_pose.pose.orientation.w;
    path.poses[path.poses.size () - 1].header             = header;
    safe_goal_pose                                        = path.poses[path.poses.size () - 1];
    path_publisher->publish (path);
}

void path_planner::inflate_map () {
    inflated_map         = original_map;
    int inflation_radius = std::ceil ((robot_size_mm / 2000.0 + offset_mm / 1000.0) / map_resolution);
    // 事前計算：距離に応じたコスト値を計算しておく（距離 → コストのLUT）
    std::vector<int8_t> cost_lookup (inflation_radius + 1);
    for (int r = 0; r <= inflation_radius; ++r) {
        double dist = r * map_resolution;
        if (dist < robot_size_mm / 2000.0) {
            cost_lookup[r] = 100;
        } else if (dist < robot_size_mm / 2000.0 + offset_mm / 1000.0) {
            cost_lookup[r] = 60;
        } else {
            cost_lookup[r] = 0;
        }
    }

    // マップ全体を走査
    for (int y = 0; y < map_height; ++y) {
        for (int x = 0; x < map_width; ++x) {
            if (original_map.data[y * map_width + x] > 50) {  // 障害物
                // 周囲にインフレーション
                for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                    for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < map_width && ny >= 0 && ny < map_height) {
                            int    nidx = ny * map_width + nx;
                            double dist = std::hypot (dx, dy);
                            if (dist <= inflation_radius) {
                                int8_t cost             = cost_lookup[static_cast<int> (std::round (dist))];
                                inflated_map.data[nidx] = std::max (inflated_map.data[nidx], cost);
                            }
                        }
                    }
                }
            }
        }
    }
}

void path_planner::astar (nav_msgs::msg::Path &path) {
    auto to_grid = [&] (double x, double y) -> std::pair<int, int> {
        int gx = static_cast<int> ((x - inflated_map.info.origin.position.x) / inflated_map.info.resolution);
        int gy = static_cast<int> ((y - inflated_map.info.origin.position.y) / inflated_map.info.resolution);
        return {gx, gy};
    };

    auto start = to_grid (current_pose.pose.position.x, current_pose.pose.position.y);
    auto goal  = to_grid (goal_pose.pose.position.x, goal_pose.pose.position.y);

    auto to_index = [&] (int x, int y) { return y * map_width + x; };

    std::priority_queue<astar_node, std::vector<astar_node>, std::greater<astar_node>> open;
    std::unordered_map<int, std::pair<int, int>>                                       came_from;
    std::unordered_map<int, double>                                                    cost_so_far;

    find_freespace (start);
    find_freespace (goal);

    open.push ({start.first, start.second, 0.0, 0.0});
    cost_so_far[to_index (start.first, start.second)] = 0.0;
    while (!open.empty ()) {
        astar_node current = open.top ();
        open.pop ();
        if (current.x == goal.first && current.y == goal.second) break;

        for (auto [dx, dy] : directions) {
            int nx = current.x + dx, ny = current.y + dy;
            if (nx < 0 || ny < 0 || nx >= map_width || ny >= map_height) continue;
            int idx = to_index (nx, ny);
            if (inflated_map.data[idx] > 50) continue;
            double new_cost = cost_so_far[to_index (current.x, current.y)] + std::hypot (dx, dy);
            if (!cost_so_far.count (idx) || new_cost < cost_so_far[idx]) {
                cost_so_far[idx] = new_cost;
                double priority  = new_cost + std::hypot (goal.first - nx, goal.second - ny);
                open.push ({nx, ny, new_cost, priority});
                came_from[idx] = {current.x, current.y};
            }
        }
    }

    auto curr = goal;
    while (curr != start) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = curr.first * map_resolution + inflated_map.info.origin.position.x + map_resolution / 2;
        pose.pose.position.y = curr.second * map_resolution + inflated_map.info.origin.position.y + map_resolution / 2;
        path.poses.push_back (pose);
        int idx = to_index (curr.first, curr.second);
        if (!came_from.count (idx)) return;
        curr = came_from[idx];
    }
    std::reverse (path.poses.begin (), path.poses.end ());
    if (path.poses.size () == 0) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = goal.first * map_resolution + inflated_map.info.origin.position.x + map_resolution / 2;
        pose.pose.position.y = goal.second * map_resolution + inflated_map.info.origin.position.y + map_resolution / 2;
        path.poses.push_back (pose);
    }
}

void path_planner::find_freespace (std::pair<int, int> &point) {
    std::vector<std::vector<bool>>  visited (map_height, std::vector<bool> (map_width, false));
    std::queue<std::pair<int, int>> q;
    q.push ({point.first, point.second});
    visited[point.second][point.first] = true;

    while (!q.empty ()) {
        auto [x, y] = q.front ();
        q.pop ();
        if (inflated_map.data[y * map_width + x] == 0) {
            point.first  = x;
            point.second = y;
            break;
        }

        for (auto [dx, dy] : directions) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < map_width && ny >= 0 && ny < map_height && !visited[ny][nx]) {
                visited[ny][nx] = true;
                q.push ({nx, ny});
            }
        }
    }
}
void path_planner::current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose = *msg;
}

void path_planner::map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    original_map   = *msg;
    map_width      = original_map.info.width;
    map_height     = original_map.info.height;
    map_resolution = original_map.info.resolution;
    inflate_map ();
    inflate_map_publisher->publish (inflated_map);
}

void path_planner::vel_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    current_vel = *msg;
}
}  // namespace path_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (path_planner::path_planner)