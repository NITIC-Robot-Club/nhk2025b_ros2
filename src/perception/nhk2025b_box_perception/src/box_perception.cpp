#include "nhk2025b_box_perception/box_perception.hpp"

namespace box_perception {

box_perception::box_perception(const rclcpp::NodeOptions &options) : Node("box_perception", options) {
    box_state_publisher_ = this->create_publisher<nhk2025b_msgs::msg::BoxArray>("/box_state", 10);
    point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/sensor/lidar", 10, std::bind(&box_perception::point_cloud_callback, this, std::placeholders::_1));
    box_array_subscriber_ = this->create_subscription<nhk2025b_msgs::msg::BoxArray>("/box_state", 10, std::bind(&box_perception::box_array_callback, this, std::placeholders::_1));
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/behavior/map", 10, std::bind(&box_perception::map_callback, this, std::placeholders::_1));
    is_red_subscriber_ = this->create_subscription<std_msgs::msg::Bool> ("/is_red", 1, std::bind (&box_perception::is_red_callback, this, std::placeholders::_1));
    iteration_num = this->declare_parameter("iteration_num", 100);
    threshold = this->declare_parameter("threshold", 0.05);
}

void box_perception::box_array_callback(const nhk2025b_msgs::msg::BoxArray::SharedPtr msg) {
    last_box_state = *msg;
    RCLCPP_INFO(this->get_logger(), "Detected %zu boxes", msg->boxes.size());
    for (size_t i = 0; i < msg->boxes.size(); ++i) {
        const auto& box = msg->boxes[i];
        RCLCPP_INFO(this->get_logger(), "Box[%zu]: pos=(%.2f, %.2f, %.2f) quat=(%.2f, %.2f, %.2f, %.2f)",
            i,
            box.pose.position.x, box.pose.position.y, box.pose.position.z,
            box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z, box.pose.orientation.w
        );
    }
}

void box_perception::is_red_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    is_red = msg->data;
}

void box_perception::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    resolution = map->info.resolution;
    map_width = map->info.width;
    map_height = map->info.height;
    map_position_x = map->info.origin.position.x;
    map_position_y = map->info.origin.position.y;

    min_x = map_position_x;
    min_y = map_position_y;
    max_x = map_position_x + map_width * resolution;
    max_y = map_position_y + map_height * resolution;

    last_map_state = *map;
}

void box_perception::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 1. 点群データを取得
    std::vector<geometry_msgs::msg::Point32> points;
    int x_offset = -1, y_offset = -1;
    for (size_t i = 0; i < msg->fields.size(); ++i) {
        if (msg->fields[i].name == "x") x_offset = msg->fields[i].offset;
        if (msg->fields[i].name == "y") y_offset = msg->fields[i].offset;
    }
    int point_step = msg->point_step;
    const uint8_t* data_ptr = msg->data.data();
    int pc2_index = msg->width * msg->height;
    for (int i = 0; i < pc2_index; i++) {
        const uint8_t* base = data_ptr + i * point_step;
        float x = *reinterpret_cast<const float*>(base + x_offset);
        float y = *reinterpret_cast<const float*>(base + y_offset);
        geometry_msgs::msg::Point32 pt;
        pt.x = x; pt.y = y; pt.z = 0.0;
        points.push_back(pt);
    }

    // マップ点群除去
    std::vector<geometry_msgs::msg::Point32> map_points;
    for (uint32_t y = 0; y < map_height; ++y) {
        for (uint32_t x = 0; x < map_width; ++x) {
            int idx = y * map_width + x;
            if (last_map_state.data[idx] > 50) {
                geometry_msgs::msg::Point32 pt;
                pt.x = map_position_x + x * resolution;
                pt.y = map_position_y + y * resolution;
                pt.z = 0.0;
                map_points.push_back(pt);
            }
        }
    }

    std::vector<geometry_msgs::msg::Point32> filtered_points;
    for (const auto& pt : points) {
        bool is_map = false;
        for (const auto& mpt : map_points) {
            double dx = pt.x - mpt.x, dy = pt.y - mpt.y;
            if (dx*dx + dy*dy < 0.01) { is_map = true; break; }
        }
        if (!is_map) filtered_points.push_back(pt);
    }

    // クラスタリング（簡易的な距離分割例）
    std::vector<std::vector<geometry_msgs::msg::Point32>> clusters;
    double cluster_tolerance = 0.3;
    for (const auto& pt : filtered_points) {
        bool assigned = false;
        for (auto& cluster : clusters) {
            for (const auto& cpt : cluster) {
                double dx = pt.x - cpt.x, dy = pt.y - cpt.y;
                if (dx*dx + dy*dy < cluster_tolerance*cluster_tolerance) {
                    cluster.push_back(pt);
                    assigned = true;
                    break;
                }
            }
            if (assigned) break;
        }
        if (!assigned) clusters.push_back({pt});
    }

    // ICPでbox位置・姿勢推定（簡易例）
    nhk2025b_msgs::msg::BoxArray box_array;
    for (const auto& cluster : clusters) {
        geometry_msgs::msg::Pose pose = estimate_pose_icp(cluster, map_points);
        nhk2025b_msgs::msg::Box box;
        box.pose = pose;
        box.size.x = 0.3; box.size.y = 0.3; box.size.z = 0.3;
        box_array.boxes.push_back(box);
    }

    box_state_publisher_->publish(box_array);
}

void box_perception::ransac(sensor_msgs::msg::PointCloud2 pc2) {
    int pc2_index = pc2.width * pc2.height;
    nhk2025b_msgs::msg::Line likelihood_line;
    int likelihood_points = 0;
    sensor_msgs::msg::PointCloud2 new_pc2;
    int new_pc2_points = pc2_index;

    // 1本目の直線
    for(int i = 0; i < iteration_num; i++) {
        int within_points = 0;
        int without_points = 0;
        nhk2025b_msgs::msg::Line iter_line;
        sensor_msgs::msg::PointCloud2 residue_pc2;
        int index_1 = rand_range(pc2_index-1);
        int index_2 = rand_range(pc2_index-1);
        while(index_1 == index_2) index_2 = rand_range(pc2_index-1);

        int x_offset = -1, y_offset = -1;
        for (size_t i = 0; i < pc2.fields.size(); ++i) {
            if (pc2.fields[i].name == "x") x_offset = pc2.fields[i].offset;
            if (pc2.fields[i].name == "y") y_offset = pc2.fields[i].offset;
        }
        int point_step = pc2.point_step;
        const uint8_t* data_ptr = pc2.data.data();
        int idx1 = index_1 * point_step;
        int idx2 = index_2 * point_step;
        float x1 = *reinterpret_cast<const float*>(data_ptr + idx1 + x_offset);
        float y1 = *reinterpret_cast<const float*>(data_ptr + idx1 + y_offset);
        float x2 = *reinterpret_cast<const float*>(data_ptr + idx2 + x_offset);
        float y2 = *reinterpret_cast<const float*>(data_ptr + idx2 + y_offset);

        iter_line.a = y1 - y2;
        iter_line.b = x1 - x2;
        iter_line.c = -1 * (iter_line.a * x1 - iter_line.b * y2);
        iter_line.x1 = x1; iter_line.y1 = y1; iter_line.x2 = x2; iter_line.y2 = y2;

        for (int j = 0; j < pc2_index; j++) {
            const uint8_t* base = data_ptr + j * point_step;
            float x = *reinterpret_cast<const float*>(base + x_offset);
            float y = *reinterpret_cast<const float*>(base + y_offset);
            geometry_msgs::msg::Point32 pt;
            pt.x = x; pt.y = y; pt.z = 0.0;
            double distance = get_distance(iter_line, pt);
            if(distance <= threshold) within_points++;
            else {
                without_points++;
                const uint8_t* src = pc2.data.data() + j * point_step;
                for (int k = 0; k < point_step; k++) {
                    residue_pc2.data.push_back(*(src + k));
                }
                residue_pc2.width++;
            }
        }
        if(within_points > likelihood_points) {
            likelihood_points = within_points;
            likelihood_line = iter_line;
        }
    }
}


uint64_t box_perception::rand_range(uint64_t max) {
    static std::mt19937_64 mt64(0);
    std::uniform_int_distribution<uint64_t> get_rand_uni_int(0, max);
    return get_rand_uni_int(mt64);
}

double box_perception::abs(double val) {
    return val >= 0 ? val : -val;
}

double box_perception::get_distance(nhk2025b_msgs::msg::Line line, geometry_msgs::msg::Point32 point) {
    return abs(line.a * point.x + line.b * point.y + line.c) / std::sqrt(line.a * line.a + line.b * line.b);
}

geometry_msgs::msg::Pose box_perception::estimate_pose_icp(
    const std::vector<geometry_msgs::msg::Point32>& cluster,
    const std::vector<geometry_msgs::msg::Point32>& map_points)
{
    geometry_msgs::msg::Pose pose;
    // クラスタの重心を初期位置とする
    double mx = 0, my = 0;
    for (const auto& pt : cluster) { mx += pt.x; my += pt.y; }
    mx /= cluster.size(); my /= cluster.size();
    pose.position.x = mx; pose.position.y = my; pose.position.z = 0.0;
    pose.orientation.w = 1.0; // 初期値

    // ICP反復
    double dx = 0, dy = 0, dtheta = 0;
    for (int iter = 0; iter < 10; ++iter) {
        // クラスタ点群をposeで変換
        std::vector<geometry_msgs::msg::Point32> transformed;
        double yaw = atan2(2.0 * pose.orientation.w * pose.orientation.z, 1.0 - 2.0 * pose.orientation.z * pose.orientation.z);
        for (const auto& pt : cluster) {
            geometry_msgs::msg::Point32 tpt;
            tpt.x = cos(yaw) * pt.x - sin(yaw) * pt.y + pose.position.x;
            tpt.y = sin(yaw) * pt.x + cos(yaw) * pt.y + pose.position.y;
            tpt.z = pt.z;
            transformed.push_back(tpt);
        }
        // ICPステップ
        icp_step(transformed, map_points, dx, dy, dtheta);
        // 適用
        apply_pose(pose, dx, dy, dtheta);
        if (std::abs(dx) < 1e-3 && std::abs(dy) < 1e-3 && std::abs(dtheta) < 1e-4) break;
    }
    return pose;
}

// ICPステップ
void box_perception::icp_step(
    const std::vector<geometry_msgs::msg::Point32>& src,
    const std::vector<geometry_msgs::msg::Point32>& tgt,
    double& dx, double& dy, double& dtheta)
{
    if (src.empty() || tgt.empty()) {
        dx = dy = dtheta = 0;
        return;
    }
    std::vector<geometry_msgs::msg::Point32> src_corr, tgt_corr;
    for (const auto& p : src) {
        int idx = -1;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < tgt.size(); ++i) {
            double ddx = p.x - tgt[i].x, ddy = p.y - tgt[i].y;
            double dist = ddx * ddx + ddy * ddy;
            if (dist < min_dist) {
                min_dist = dist;
                idx = i;
            }
        }
        if (min_dist < 0.25 && idx >= 0) { // 0.5m^2以内
            src_corr.push_back(p);
            tgt_corr.push_back(tgt[idx]);
        }
    }
    if (src_corr.empty()) {
        dx = dy = dtheta = 0;
        return;
    }
    // 重心計算
    double mx = 0, my = 0, mxp = 0, myp = 0;
    int N = src_corr.size();
    for (int i = 0; i < N; ++i) {
        mx += src_corr[i].x;
        my += src_corr[i].y;
        mxp += tgt_corr[i].x;
        myp += tgt_corr[i].y;
    }
    mx /= N; my /= N; mxp /= N; myp /= N;
    // 回転推定
    double Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;
    for (int i = 0; i < N; ++i) {
        double x = src_corr[i].x - mx, y = src_corr[i].y - my;
        double xp = tgt_corr[i].x - mxp, yp = tgt_corr[i].y - myp;
        Sxx += x * xp;
        Sxy += x * yp;
        Syx += y * xp;
        Syy += y * yp;
    }
    double theta = atan2(Sxy - Syx, Sxx + Syy);
    dtheta = theta;
    dx = mxp - (mx * cos(theta) - my * sin(theta));
    dy = myp - (mx * sin(theta) + my * cos(theta));
}

// pose適用
void box_perception::apply_pose(geometry_msgs::msg::Pose& pose, double dx, double dy, double dtheta)
{
    pose.position.x += dx;
    pose.position.y += dy;
    double yaw = atan2(2.0 * pose.orientation.w * pose.orientation.z, 1.0 - 2.0 * pose.orientation.z * pose.orientation.z);
    yaw += dtheta;
    pose.orientation.z = sin(yaw / 2);
    pose.orientation.w = cos(yaw / 2);
}

} // namespace box_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(box_perception::box_perception)
