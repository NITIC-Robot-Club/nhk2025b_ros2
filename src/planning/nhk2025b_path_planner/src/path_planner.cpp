#include "nhk2025b_path_planner/path_planner.hpp"

namespace path_planner {
path_planner::path_planner (const rclcpp::NodeOptions &options) : Node ("path_planner", options) {
    resolution_ms = this->declare_parameter<int> ("resolution_ms", 100);
    offset_mm     = this->declare_parameter<int> ("offset_mm", 50);
    robot_size_mm = this->declare_parameter<int> ("robot_size_mm", 1414);
    tolerance_xy  = this->declare_parameter<int> ("tolerance_xy", 10);
    tolerance_z   = this->declare_parameter<double> ("tolerance_z", 0.1);

    path_publisher          = this->create_publisher<nav_msgs::msg::Path> ("/planning/path", 10);
    current_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/localization/current_pose", 10, std::bind (&path_planner::current_pose_callback, this, std::placeholders::_1));
    goal_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/behavior/goal_pose", 10, std::bind (&path_planner::goal_pose_callback, this, std::placeholders::_1));
    map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid> (
        "/behavior/map", 10, std::bind (&path_planner::map_callback, this, std::placeholders::_1));
    vel_subscriber = this->create_subscription<geometry_msgs::msg::TwistStamped> (
        "/cmd_vel", 10, std::bind (&path_planner::vel_callback, this, std::placeholders::_1));
    timer = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&path_planner::timer_callback, this));

    inflate_map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid> ("/planning/costmap", 10);
}
void path_planner::timer_callback () {
    if (original_map.header.stamp.sec == 0) return;
    if (current_pose.header.stamp.sec == 0) return;
    if (goal_pose.header.stamp.sec == 0) return;
    nav_msgs::msg::Path   path;
    std_msgs::msg::Header header;
    header.frame_id = "map";
    header.stamp    = this->now ();
    path.header     = header;

    double dx       = goal_pose.pose.position.x - current_pose.pose.position.x;
    double dy       = goal_pose.pose.position.y - current_pose.pose.position.y;
    double distance = std::hypot (dx, dy);
    if (distance < tolerance_xy) {
        path_publisher->publish (path);
        return;
    }
    astar (path);

    double current_yaw = 2.0 * std::asin (current_pose.pose.orientation.z);
    double goal_yaw    = 2.0 * std::asin (goal_pose.pose.orientation.z);
    double delta_yaw   = goal_yaw - current_yaw;
    if (delta_yaw > M_PI)
        delta_yaw -= 2 * M_PI;
    else if (delta_yaw < -M_PI)
        delta_yaw += 2 * M_PI;
    double delta_t = resolution_ms / 1000.0;
    for (int i = 0; i < path.poses.size (); i++) {
        double now_yaw                   = current_yaw + delta_yaw / (1.0 + std::exp (-7.5 * ((double)i / path.poses.size () - 0.5)));
        path.poses[i].pose.orientation.z = std::sin (now_yaw / 2.0);
        path.poses[i].pose.orientation.w = std::cos (now_yaw / 2.0);
        path.poses[i].header             = header;
    }
    path_publisher->publish (path);
}

void path_planner::inflate_map () {
    inflated_map         = original_map;
    int inflation_radius = std::ceil ((robot_size_mm / 2000.0 + offset_mm / 1000.0) / map_resolution);
    // 事前計算：距離に応じたコスト値を計算しておく（距離 → コストのLUT）
    std::vector<int8_t> cost_lookup (inflation_radius + 1);
    for (int r = 0; r <= inflation_radius; ++r) {
        double dist = r * map_resolution;
        if (dist > inflation_radius) {
            cost_lookup[r] = 0;
        } else {
            // 距離に応じて指数的に減衰（例：100 → 1）
            double factor  = (inflation_radius - dist) / inflation_radius;
            cost_lookup[r] = static_cast<int8_t> (std::round (100 * factor));
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

    std::vector<std::pair<int, int>> directions = {
        { 1,  0},
        {-1,  0},
        { 0,  1},
        { 0, -1},
        { 1,  1},
        {-1, -1},
        { 1, -1},
        {-1,  1}
    };
    std::vector<std::vector<bool>>  visited_start (map_height, std::vector<bool> (map_width, false));
    std::queue<std::pair<int, int>> q_start;
    q_start.push ({start.first, start.second});
    visited_start[start.second][start.first] = true;

    while (!q_start.empty ()) {
        auto [x, y] = q_start.front ();
        q_start.pop ();
        if (inflated_map.data[y * map_width + x] == 0) {
            start.first  = x;
            start.second = y;
            break;
        }

        for (auto [dx, dy] : directions) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < map_width && ny >= 0 && ny < map_height && !visited_start[ny][nx]) {
                visited_start[ny][nx] = true;
                q_start.push ({nx, ny});
            }
        }
    }

    std::vector<std::vector<bool>>  visited_goal (map_height, std::vector<bool> (map_width, false));
    std::queue<std::pair<int, int>> q_goal;
    q_goal.push ({goal.first, goal.second});
    visited_goal[goal.second][goal.first] = true;

    while (!q_goal.empty ()) {
        auto [x, y] = q_goal.front ();
        q_goal.pop ();
        if (inflated_map.data[y * map_width + x] == 0) {
            goal.first  = x;
            goal.second = y;
            break;
        }

        for (auto [dx, dy] : directions) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < map_width && ny >= 0 && ny < map_height && !visited_goal[ny][nx]) {
                visited_goal[ny][nx] = true;
                q_goal.push ({nx, ny});
            }
        }
    }

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
}

void path_planner::current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose = *msg;
}

void path_planner::goal_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_pose = *msg;
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