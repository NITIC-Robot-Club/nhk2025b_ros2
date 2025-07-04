#include "nhk2025b_path_planner/path_planner.hpp"

namespace path_planner {
path_planner::path_planner (const rclcpp::NodeOptions &options) : Node ("path_planner", options) {
    theta_resolution = this->declare_parameter<int> ("theta_resolution", 5);
    resolution_ms    = this->declare_parameter<int> ("resolution_ms", 100);
    offset_mm        = this->declare_parameter<int> ("offset_mm", 30);
    penalty_mm       = this->declare_parameter<int> ("penalty_mm", 750);
    robot_height_mm  = this->declare_parameter<int> ("robot_height_mm", 800);
    robot_width_mm   = this->declare_parameter<int> ("robot_width_mm", 600);
    tolerance_xy_mm  = this->declare_parameter<int> ("tolerance_xy_mm", 30);
    tolerance_z_rad  = this->declare_parameter<double> ("tolerance_z_rad", 0.03);
    sigmoid_gain     = this->declare_parameter<double> ("sigmoid_gain", 7.5);
    grad_alpha       = this->declare_parameter<double> ("grad_alpha", 1.0);
    grad_beta        = this->declare_parameter<double> ("grad_beta", 8.2);
    grad_gamma       = this->declare_parameter<double> ("grad_gamma", 0.0);
    grad_step_size   = this->declare_parameter<double> ("grad_step_size", 0.1);

    path_publisher          = this->create_publisher<nav_msgs::msg::Path> ("/planning/path", 1);
    current_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/localization/current_pose", 1, std::bind (&path_planner::current_pose_callback, this, std::placeholders::_1));
    goal_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/behavior/goal_pose", 1, std::bind (&path_planner::goal_pose_callback, this, std::placeholders::_1));
    map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid> (
        "/behavior/map", 1, std::bind (&path_planner::map_callback, this, std::placeholders::_1));
    vel_subscriber = this->create_subscription<geometry_msgs::msg::TwistStamped> (
        "/cmd_vel", 1, std::bind (&path_planner::vel_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&path_planner::timer_callback, this));

    inflate_map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid> ("/planning/costmap", 1);
    theta_map_publisher   = this->create_publisher<nav_msgs::msg::OccupancyGrid> ("/planning/thetamap", 1);
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
    goal_pose = *msg;

    if (original_map.header.stamp.sec == 0) return;
    if (current_pose.header.stamp.sec == 0) return;
    if (goal_pose.header.stamp.sec == 0) return;
    nav_msgs::msg::Path   path;
    std_msgs::msg::Header header;
    header.frame_id = "map";
    header.stamp    = this->now ();
    path.header     = header;

    linear_astar ();

    path_smoother ();
    angular_astar (path);

    for (int i = 0; i < path.poses.size (); i++) {
        path.poses[i].header = header;
    }
    if (path.poses.size () != 0) safe_goal_pose = path.poses.back ();
    path_publisher->publish (path);
}

void path_planner::inflate_map () {
    if (original_map.data.size () == last_map.data.size ()) {
        bool is_same_map = true;
        for (int i = 0; i < original_map.data.size (); ++i) {
            if (original_map.data[i] != last_map.data[i]) {
                is_same_map = false;
                break;
            }
        }
        if (is_same_map) {
            return;
        }
    }
    occ_map.data.clear ();
    inflated_map.clear ();
    offset_map.clear ();
    occ_map.data.resize (map_width * map_height, 0);
    inflated_map.resize (map_height, std::vector<int8_t> (map_width, 0));
    offset_map.resize (map_height, std::vector<int8_t> (map_width, 0));
    double width_radius         = robot_width_mm / 2000.0 / map_resolution;
    double height_radius        = robot_height_mm / 2000.0 / map_resolution;
    int    offset_radius        = std::ceil (offset_mm / 1000.0 / map_resolution);
    int    max_inflation_radius = std::ceil (std::hypot (width_radius, height_radius)) + offset_radius;
    int    min_inflation_radius = std::ceil (std::min (width_radius, height_radius)) + offset_radius;
    int    inflate_radius       = std::max (max_inflation_radius, static_cast<int> (std::ceil (penalty_mm / 1000.0 / map_resolution)));
    // マップ全体を走査
    for (int y = 0; y < map_height; ++y) {
        for (int x = 0; x < map_width; ++x) {
            if (original_map.data[y * map_width + x] > 50) {  // 障害物
                // 周囲にインフレーション
                for (int dy = -inflate_radius; dy <= inflate_radius; ++dy) {
                    for (int dx = -inflate_radius; dx <= inflate_radius; ++dx) {
                        int next_x = x + dx;
                        int next_y = y + dy;
                        if (next_x >= 0 && next_x < map_width && next_y >= 0 && next_y < map_height) {
                            double dist = std::hypot (dx, dy);
                            if (dist <= offset_radius) {
                                offset_map[next_y][next_x]   = std::max (offset_map[next_y][next_x], int8_t (100));
                                inflated_map[next_y][next_x] = std::max (inflated_map[next_y][next_x], int8_t (100));
                            } else if (dist <= min_inflation_radius) {
                                inflated_map[next_y][next_x] = std::max (inflated_map[next_y][next_x], int8_t (80));
                            } else if (dist <= max_inflation_radius) {
                                inflated_map[next_y][next_x] = std::max (inflated_map[next_y][next_x], int8_t (30));
                            }
                            int8_t dist_penalty = 0;
                            if (dist <= inflate_radius) {
                                dist_penalty = static_cast<int8_t> (100 * (1.0 - (dist * dist) / (inflate_radius * inflate_radius)));
                            }
                            occ_map.data[next_y * map_width + next_x] = std::max (occ_map.data[next_y * map_width + next_x], dist_penalty);
                        }
                    }
                }
            }
        }
    }
}
std::pair<int, int> path_planner::to_grid (double x, double y) {
    int gx = static_cast<int> ((x - original_map.info.origin.position.x) / map_resolution);
    int gy = static_cast<int> ((y - original_map.info.origin.position.y) / map_resolution);
    gx     = std::clamp (gx, 0, map_width - 1);
    gy     = std::clamp (gy, 0, map_height - 1);
    return {gx, gy};
}
void path_planner::path_smoother () {
    auto get_cost = [&] (int x, int y) -> double {
        if (x < 0 || y < 0 || x >= map_width || y >= map_height) return 1.0;
        return static_cast<double> (occ_map.data[y * map_width + x]) / 100.0;
    };
    auto get_cost_gradient = [&] (double x, double y) -> std::pair<double, double> {
        auto [grid_x, grid_y] = to_grid (x, y);
        double dx             = get_cost (grid_x + 1, grid_y) - get_cost (grid_x - 1, grid_y);
        double dy             = get_cost (grid_x, grid_y + 1) - get_cost (grid_x, grid_y - 1);
        return {dx / 2.0, dy / 2.0};
    };
    smoothed_path.poses.clear ();
    smoothed_path.poses      = linear_path.poses;
    const int max_iterations = 300;
    if (smoothed_path.poses.size () < 3) {
        return;
    }
    for (int iter = 0; iter < max_iterations; ++iter) {
        for (size_t i = 1; i + 1 < smoothed_path.poses.size (); ++i) {
            geometry_msgs::msg::PoseStamped p_prev = smoothed_path.poses[i - 1];
            geometry_msgs::msg::PoseStamped p_curr = smoothed_path.poses[i];
            geometry_msgs::msg::PoseStamped p_next = smoothed_path.poses[i + 1];

            // コストマップ勾配（障害物回避）
            auto [cost_x, cost_y] = get_cost_gradient (p_curr.pose.position.x, p_curr.pose.position.y);

            // 曲率項（滑らかさ） : 2nd derivative
            double smooth_x = p_curr.pose.position.x - 0.5 * (p_prev.pose.position.x + p_next.pose.position.x);
            double smooth_y = p_curr.pose.position.y - 0.5 * (p_prev.pose.position.y + p_next.pose.position.y);

            // 元のパスからの引き戻し項
            double anchor_x = p_curr.pose.position.x - linear_path.poses[i].pose.position.x;
            double anchor_y = p_curr.pose.position.y - linear_path.poses[i].pose.position.y;

            // 合成勾配
            double total_x = cost_x * grad_alpha + smooth_x * grad_beta + anchor_x * grad_gamma;
            double total_y = cost_y * grad_alpha + smooth_y * grad_beta + anchor_y * grad_gamma;

            // 勾配降下による更新
            smoothed_path.poses[i].pose.position.x -= total_x * grad_step_size;
            smoothed_path.poses[i].pose.position.y -= total_y * grad_step_size;
        }
    }
    // RCLCPP_INFO (this->get_logger (), "Smoothing  %zu points", smoothed_path.poses.size ());
}
void path_planner::linear_astar () {
    linear_path.poses.clear ();

    auto start = to_grid (current_pose.pose.position.x, current_pose.pose.position.y);
    auto goal  = to_grid (goal_pose.pose.position.x, goal_pose.pose.position.y);

    auto to_index = [&] (int x, int y) { return y * map_width + x; };

    std::priority_queue<astar_node, std::vector<astar_node>, std::greater<astar_node>> open;
    std::unordered_map<int, std::pair<int, int>>                                       came_from;
    std::unordered_map<int, double>                                                    cost_so_far;

    int start_theta = rad_to_deg (get_yaw_2d (current_pose.pose.orientation));
    int goal_theta  = rad_to_deg (get_yaw_2d (goal_pose.pose.orientation));
    start_theta     = angle_to_index (start_theta);
    goal_theta      = angle_to_index (goal_theta);

    find_freespace (start, start_theta);
    find_freespace (goal, goal_theta);

    open.push ({start.first, start.second, 0.0, 0.0});
    cost_so_far[to_index (start.first, start.second)] = 0.0;
    while (!open.empty ()) {
        astar_node current = open.top ();
        open.pop ();
        if (current.x == goal.first && current.y == goal.second) break;

        for (auto [dx, dy] : directions) {
            int next_x = current.x + dx, next_y = current.y + dy;
            if (next_x < 0 || next_y < 0 || next_x >= map_width || next_y >= map_height) continue;
            int idx = to_index (next_x, next_y);
            if (inflated_map[next_y][next_x] > 50) continue;
            double new_cost = cost_so_far[to_index (current.x, current.y)] + std::hypot (dx, dy);
            if (!cost_so_far.count (idx) || new_cost < cost_so_far[idx]) {
                cost_so_far[idx] = new_cost;
                double priority  = new_cost + std::hypot (goal.first - next_x, goal.second - next_y);
                open.push ({next_x, next_y, new_cost, priority});
                came_from[idx] = {current.x, current.y};
            }
        }
    }

    auto curr = goal;
    while (curr != start) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = curr.first * map_resolution + original_map.info.origin.position.x + map_resolution / 2;
        pose.pose.position.y = curr.second * map_resolution + original_map.info.origin.position.y + map_resolution / 2;
        linear_path.poses.push_back (pose);
        int idx = to_index (curr.first, curr.second);
        if (!came_from.count (idx)) {
            linear_path.poses.clear ();
            return;
        }
        curr = came_from[idx];
    }
    std::reverse (linear_path.poses.begin (), linear_path.poses.end ());
    // RCLCPP_INFO (this->get_logger (), "Linear %zu points", linear_path.poses.size ());
}
void path_planner::angular_astar (nav_msgs::msg::Path &path) {
    if (smoothed_path.poses.size () == 0) {
        // RCLCPP_WARN (this->get_logger (), "linear path is empty, cannot perform angular A*");
        return;
    }

    int start_theta = rad_to_deg (get_yaw_2d (current_pose.pose.orientation));
    int goal_theta  = rad_to_deg (get_yaw_2d (goal_pose.pose.orientation));
    start_theta     = angle_to_index (start_theta);
    goal_theta      = angle_to_index (goal_theta);

    std::priority_queue<astar_node, std::vector<astar_node>, std::greater<astar_node>> open;
    std::unordered_map<int, std::pair<int, int>>                                       came_from;
    std::unordered_map<int, double>                                                    cost_so_far;

    came_from.clear ();
    cost_so_far.clear ();
    nav_msgs::msg::OccupancyGrid theta_map;
    theta_map.header.frame_id        = "map";
    theta_map.header.stamp           = this->now ();
    theta_map.info.width             = smoothed_path.poses.size ();
    theta_map.info.height            = 360 / theta_resolution;
    theta_map.info.resolution        = 0.05;
    theta_map.info.origin.position.x = 0.0;
    theta_map.info.origin.position.y = 0.0;
    theta_map.data.resize (theta_map.info.width * theta_map.info.height, 0);
    // 各角度のコストを計算
    angle_cost_map.clear ();
    angle_cost_map.resize (smoothed_path.poses.size (), std::vector<int8_t> (360 / theta_resolution, 0));
    for (int i = 0; i < smoothed_path.poses.size (); ++i) {
        auto [gx, gy] = to_grid (smoothed_path.poses[i].pose.position.x, smoothed_path.poses[i].pose.position.y);
        if (inflated_map[gy][gx] > 10) {
            for (int j = 0; j < angle_cost_map[0].size (); ++j) {
                if (is_collision (gx, gy, j)) {
                    angle_cost_map[i][j]                         = 100;
                    theta_map.data[j * theta_map.info.width + i] = 100;
                }
            }
        }
    }
    theta_map_publisher->publish (theta_map);

    auto to_index = [&] (int x, int y) { return y * angle_cost_map.size () + x; };
    open.push ({0, start_theta, 0.0, 0.0});
    cost_so_far[to_index (0, start_theta)] = 0.0;

    while (!open.empty ()) {
        astar_node current = open.top ();
        open.pop ();
        if (current.x == smoothed_path.poses.size () - 1 && current.y == goal_theta) {
            break;
        }
        for (auto dx : {0, 1}) {
            for (auto dth : rotations) {
                if (dx == 0 && dth == 0) continue;
                int next_x = current.x + dx, next_theta = current.y + dth;
                if (next_x >= smoothed_path.poses.size ()) continue;
                if (next_theta < 0) {
                    next_theta += angle_cost_map[0].size ();
                } else if (next_theta >= angle_cost_map[0].size ()) {
                    next_theta -= angle_cost_map[0].size ();
                }
                if (angle_cost_map[next_x][next_theta] > 50) continue;

                double new_cost = cost_so_far[to_index (current.x, current.y)] + theta_heuristic (dx, dth);
                if (!cost_so_far.count (to_index (next_x, next_theta)) || new_cost < cost_so_far[to_index (next_x, next_theta)]) {
                    cost_so_far[to_index (next_x, next_theta)] = new_cost;

                    double priority = new_cost + theta_heuristic (angle_cost_map.size () - 1 - next_x, goal_theta - next_theta);
                    open.push ({next_x, next_theta, new_cost, priority});
                    came_from[to_index (next_x, next_theta)] = {current.x, current.y};
                }
            }
        }
    }
    auto curr = std::make_pair (smoothed_path.poses.size () - 1, goal_theta);
    while (curr.first != 0 || curr.second != start_theta) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x    = smoothed_path.poses[curr.first].pose.position.x;
        pose.pose.position.y    = smoothed_path.poses[curr.first].pose.position.y;
        double yaw              = curr.second * theta_resolution * M_PI / 180.0;
        pose.pose.orientation.z = std::sin (yaw / 2.0);
        pose.pose.orientation.w = std::cos (yaw / 2.0);
        path.poses.push_back (pose);
        int idx = to_index (curr.first, curr.second);
        if (!came_from.count (idx)) {
            path.poses.clear ();
            return;
        }
        curr = came_from[idx];
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x    = smoothed_path.poses[0].pose.position.x;
    pose.pose.position.y    = smoothed_path.poses[0].pose.position.y;
    double yaw              = start_theta * theta_resolution * M_PI / 180.0;
    pose.pose.orientation.z = std::sin (yaw / 2.0);
    pose.pose.orientation.w = std::cos (yaw / 2.0);
    path.poses.push_back (pose);

    std::reverse (path.poses.begin (), path.poses.end ());
    // RCLCPP_INFO (this->get_logger (), "Angular %zu points", path.poses.size ());
}
double path_planner::theta_heuristic (int dx, int theta) {
    theta = std::abs (theta);
    if (theta >= 180 / theta_resolution) theta = 360 / theta_resolution - theta;
    double angle_weight = 0.5;
    return hypot (dx, angle_weight * theta);
}
void path_planner::init_rotated_footprint () {
    int num_rotations = 360 / theta_resolution;
    rotated_footprint.resize (num_rotations);
    double inital_angle = std::atan2 (robot_height_mm, robot_width_mm);
    double half_radius  = std::hypot (robot_width_mm / 2000.0, robot_height_mm / 2000.0) / map_resolution;
    for (int i = 0; i < num_rotations; ++i) {
        double angle            = i * theta_resolution * M_PI / 180.0;
        rotated_footprint[i][0] = {
            static_cast<int> (half_radius * std::cos (angle + inital_angle)), static_cast<int> (half_radius * std::sin (angle + inital_angle))};
        rotated_footprint[i][1] = {
            static_cast<int> (half_radius * std::cos (angle - inital_angle)), static_cast<int> (half_radius * std::sin (angle - inital_angle))};
        rotated_footprint[i][2] = {-rotated_footprint[i][0].first, -rotated_footprint[i][0].second};
        rotated_footprint[i][3] = {-rotated_footprint[i][1].first, -rotated_footprint[i][1].second};
    }
}
bool path_planner::is_collision (int x, int y, int theta) {
    for (int i = 0; i < 4; ++i) {
        int next_x = x + rotated_footprint[theta][i].first;
        int next_y = y + rotated_footprint[theta][i].second;
        if (next_x < 0 || next_y < 0 || next_x >= map_width || next_y >= map_height) {
            return true;
        }
        if (offset_map[next_y][next_x] > 50) {
            return true;
        }
    }
    return false;
}
void path_planner::find_freespace (std::pair<int, int> &point, int theta) {
    std::vector<std::vector<bool>>  visited (map_height, std::vector<bool> (map_width, false));
    std::queue<std::pair<int, int>> q;
    q.push ({point.first, point.second});
    visited[point.second][point.first] = true;

    while (!q.empty ()) {
        auto [x, y] = q.front ();
        q.pop ();
        if (!is_collision (x, y, theta)) {
            point.first  = x;
            point.second = y;
            return;
        }

        for (auto [dx, dy] : directions) {
            int next_x = x + dx;
            int next_y = y + dy;
            if (next_x >= 0 && next_x < map_width && next_y >= 0 && next_y < map_height && !visited[next_y][next_x]) {
                visited[next_y][next_x] = true;
                q.push ({next_x, next_y});
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
    occ_map.info   = original_map.info;
    occ_map.header = original_map.header;
    inflate_map ();
    last_map = original_map;
    inflate_map_publisher->publish (occ_map);

    if (rotated_footprint.size () == 0) {
        init_rotated_footprint ();
    }
}

void path_planner::vel_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    current_vel = *msg;
}
}  // namespace path_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (path_planner::path_planner)