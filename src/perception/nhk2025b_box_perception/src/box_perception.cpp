#include "nhk2025b_box_perception/box_perception.hpp"

namespace box_perception {

box_perception::box_perception(const rclcpp::NodeOptions &options) : Node("box_perception", options) {
    box_state_update_publisher_ = this->create_publisher<nhk2025b_msgs::msg::BoxArray>("/box_state", 10);
    point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/sensor/lidar", 10, std::bind(&box_perception::point_cloud_callback, this, std::placeholders::_1));
    box_array_subscriber_ = this->create_subscription<nhk2025b_msgs::msg::BoxArray>("/box_state", 10, std::bind(&box_perception::box_array_callback, this, std::placeholders::_1));
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/behavior/map", 10, std::bind(&box_perception::map_callback, this, std::placeholders::_1));
    is_red_subscriber_ = this->create_subscription<std_msgs::msg::Bool> ("/is_red", 1, std::bind (&box_perception::is_red_callback, this, std::placeholders::_1));
    iteration_num = this->declare_parameter("iteration_num", 100);
    threshold = this->declare_parameter("threshold", 0.05);
}

void box_perception::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    resolution = map->info.resolution;
    map_width = map->info.width;
    map_height = map->info.height;
    map_position_x = map->info.origin.position.x;
    map_position_y = map->info.origin.position.y;
    // マップの4隅を計算
    min_x = map_position_x;
    min_y = map_position_y;
    max_x = map_position_x + map_width * resolution;
    max_y = map_position_y + map_height * resolution;
    printf("Map corners: (%.2f, %.2f) - (%.2f, %.2f)\n", min_x, min_y, max_x, max_y);
}

void box_perception::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (msg->width * msg->height == 0) return;
    int cluster_id = 0;

    // 点群をマップ内側のみ抽出
    std::vector<geometry_msgs::msg::Point32> filtered_points;
    int x_offset = -1, y_offset = -1;
    for (size_t i = 0; i < msg->fields.size(); ++i) {
        if (msg->fields[i].name == "x") x_offset = msg->fields[i].offset;
        if (msg->fields[i].name == "y") y_offset = msg->fields[i].offset;
    }
    int point_step = msg->point_step;
    int num_points = msg->width * msg->height;
    const uint8_t* data_ptr = msg->data.data();
    for (int i = 0; i < num_points; ++i) {
        const uint8_t* base = data_ptr + i * point_step;
        float x = *reinterpret_cast<const float*>(base + x_offset);
        float y = *reinterpret_cast<const float*>(base + y_offset);
        if (x > min_x + 0.05 && x < max_x - 0.05 && y > min_y + 0.05 && y < max_y - 0.05) {
            geometry_msgs::msg::Point32 pt;
            pt.x = x; pt.y = y; pt.z = 0.0;
            filtered_points.push_back(pt);
        }
    }

    // クラスタリング（簡易DBSCAN風）
    std::vector<std::vector<geometry_msgs::msg::Point32>> clusters;
    double cluster_dist = 0.15;
    for (auto& pt : filtered_points) {
        bool added = false;
        for (auto& cluster : clusters) {
            for (auto& cpt : cluster) {
                double dist = std::hypot(pt.x - cpt.x, pt.y - cpt.y);
                if (dist < cluster_dist) {
                    cluster.push_back(pt);
                    added = true;
                    break;
                }
            }
            if (added) break;
        }
        if (!added) clusters.push_back({pt});
    }

    // 各クラスタにRANSAC直線検出
    nhk2025b_msgs::msg::BoxArray box_array_msg;
    for (auto& cluster : clusters) {
        if (cluster.size() < 30) continue; // クラスタサイズ下限
        // 点群→PointCloud2変換
        sensor_msgs::msg::PointCloud2 pc2;
        pc2.width = cluster.size();
        pc2.height = 1;
        pc2.point_step = sizeof(float) * 3;
        pc2.row_step = pc2.point_step * pc2.width;
        pc2.fields.resize(3);
        pc2.fields[0].name = "x"; pc2.fields[0].offset = 0; pc2.fields[0].datatype = 7; pc2.fields[0].count = 1;
        pc2.fields[1].name = "y"; pc2.fields[1].offset = 4; pc2.fields[1].datatype = 7; pc2.fields[1].count = 1;
        pc2.fields[2].name = "z"; pc2.fields[2].offset = 8; pc2.fields[2].datatype = 7; pc2.fields[2].count = 1;
        pc2.data.resize(pc2.row_step);
        for (size_t i = 0; i < cluster.size(); ++i) {
            float* ptr = reinterpret_cast<float*>(&pc2.data[i * pc2.point_step]);
            ptr[0] = cluster[i].x;
            ptr[1] = cluster[i].y;
            ptr[2] = cluster[i].z;
        }
        // RANSACで線分検出
        nhk2025b_msgs::msg::Line line;
        int inliers = ransac_line(pc2, line, threshold, iteration_num);
        double length = std::hypot(line.x2 - line.x1, line.y2 - line.y1);
        if (inliers < 20) continue; // インライア数下限
        if (length < 0.18 || length > 1.1) continue; // 長さ範囲
        std::string box_type = estimate_box_type(length);
        if (box_type == "Unknown") continue; // 箱種別不明は除外

        // 直前のbox_stateと比較して更新
        bool updated = true;
        for (const auto& prev_box : last_box_state.boxes) {
            double prev_length = prev_box.size.x; // 代表値としてxを使用
            if (std::abs(prev_length - length) < 0.05) {
                updated = false;
                break;
            }
        }

        if (updated) {
            nhk2025b_msgs::msg::Box box;

            // 箱種別をBoxInfo.typeにセット
            if (box_type == "A") box.info.type = nhk2025b_msgs::msg::BoxInfo::A;
            else if (box_type == "B") box.info.type = nhk2025b_msgs::msg::BoxInfo::B;
            else if (box_type == "C") box.info.type = nhk2025b_msgs::msg::BoxInfo::C;
            else if (box_type == "D") box.info.type = nhk2025b_msgs::msg::BoxInfo::D;
            else if (box_type == "E") box.info.type = nhk2025b_msgs::msg::BoxInfo::E;
            else box.info.type = 0; // Unknown

            // 箱サイズ（長さ・幅・高さ）
            if (box_type == "A") { box.size.x = 0.3; box.size.y = 0.3; box.size.z = 0.3; }
            else if (box_type == "B") { box.size.x = 0.4; box.size.y = 0.4; box.size.z = 0.4; }
            else if (box_type == "C") { box.size.x = 0.5; box.size.y = 0.5; box.size.z = 0.5; }
            else if (box_type == "D") { box.size.x = 0.2; box.size.y = 0.2; box.size.z = 0.8; }
            else if (box_type == "E") { box.size.x = 0.3; box.size.y = 0.3; box.size.z = 1.0; }
            else { box.size.x = length; box.size.y = length; box.size.z = length; } // Unknown

            // 箱の中心座標と向き（線分の中点と角度）
            box.pose.position.x = (line.x1 + line.x2) / 2.0;
            box.pose.position.y = (line.y1 + line.y2) / 2.0;
            box.pose.position.z = 0.0;
            double yaw = std::atan2(line.y2 - line.y1, line.x2 - line.x1);
            box.pose.orientation.x = 0.0;
            box.pose.orientation.y = 0.0;
            box.pose.orientation.z = std::sin(yaw / 2.0);
            box.pose.orientation.w = std::cos(yaw / 2.0);

            // IDは適宜設定（例:クラスタ番号やユニーク値）
            box.info.id = cluster_id++;

            box_array_msg.boxes.push_back(box);
        }
    }

    // box_stateをpublish
    if (!box_array_msg.boxes.empty()) {
        box_state_update_publisher_->publish(box_array_msg);
    }
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
                // 残差点群生成（必要なら実装）
            }
        }
        if(within_points > likelihood_points) {
            likelihood_points = within_points;
            likelihood_line = iter_line;
        }
    }
    double line_length = std::hypot(likelihood_line.x2 - likelihood_line.x1, likelihood_line.y2 - likelihood_line.y1);
    std::string box_type = estimate_box_type(line_length);
    printf("Detected line: a=%.3f, b=%.3f, c=%.3f, x1=%.3f, y1=%.3f, x2=%.3f, y2=%.3f, length=%.3f, type=%s, inliers=%d\n",
        likelihood_line.a, likelihood_line.b, likelihood_line.c,
        likelihood_line.x1, likelihood_line.y1, likelihood_line.x2, likelihood_line.y2,
        line_length, box_type.c_str(), likelihood_points);
}

int box_perception::ransac_line(const sensor_msgs::msg::PointCloud2& pc2, nhk2025b_msgs::msg::Line& out_line, double threshold, int iteration_num) {
    int pc2_index = pc2.width * pc2.height;
    int likelihood_points = 0;
    nhk2025b_msgs::msg::Line likelihood_line;
    int x_offset = 0, y_offset = 4;
    const uint8_t* data_ptr = pc2.data.data();
    for(int i = 0; i < iteration_num; i++) {
        int index_1 = rand_range(pc2_index-1);
        int index_2 = rand_range(pc2_index-1);
        while(index_1 == index_2) index_2 = rand_range(pc2_index-1);
        int idx1 = index_1 * pc2.point_step;
        int idx2 = index_2 * pc2.point_step;
        float x1 = *reinterpret_cast<const float*>(data_ptr + idx1 + x_offset);
        float y1 = *reinterpret_cast<const float*>(data_ptr + idx1 + y_offset);
        float x2 = *reinterpret_cast<const float*>(data_ptr + idx2 + x_offset);
        float y2 = *reinterpret_cast<const float*>(data_ptr + idx2 + y_offset);
        nhk2025b_msgs::msg::Line iter_line;
        iter_line.a = y1 - y2;
        iter_line.b = x1 - x2;
        iter_line.c = -1 * (iter_line.a * x1 - iter_line.b * y2);
        iter_line.x1 = x1; iter_line.y1 = y1; iter_line.x2 = x2; iter_line.y2 = y2;
        int within_points = 0;
        for (int j = 0; j < pc2_index; j++) {
            int idx = j * pc2.point_step;
            float x = *reinterpret_cast<const float*>(data_ptr + idx + x_offset);
            float y = *reinterpret_cast<const float*>(data_ptr + idx + y_offset);
            geometry_msgs::msg::Point32 pt;
            pt.x = x; pt.y = y; pt.z = 0.0;
            double distance = get_distance(iter_line, pt);
            if(distance <= threshold) within_points++;
        }
        if(within_points > likelihood_points) {
            likelihood_points = within_points;
            likelihood_line = iter_line;
        }
    }
    out_line = likelihood_line;
    return likelihood_points;
}

uint64_t box_perception::rand_range(uint64_t max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<uint64_t> dis(0, max);
    return dis(gen);
}

double box_perception::abs(double val) {
    return val >= 0 ? val : -val;
}

double box_perception::get_distance(nhk2025b_msgs::msg::Line line, geometry_msgs::msg::Point32 point) {
    double distance = abs(line.a * point.x + line.b * point.y + line.c) / sqrt(line.a * line.a + line.b * line.b);
    return distance;
}

std::string box_perception::estimate_box_type(double length) {
    const double box_sizes[][3] = {
        {0.3, 0.3, 0.3},   // A
        {0.4, 0.4, 0.4},   // B
        {0.5, 0.5, 0.5},   // C
        {0.2, 0.2, 0.8},   // D
        {0.3, 0.3, 1.0}    // E
    };
    const char* box_names[] = {"A", "B", "C", "D", "E"};
    double min_err = 1e9;
    int best_idx = -1;
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 3; ++j) {
            double err = std::abs(length - box_sizes[i][j]);
            if (err < min_err) {
                min_err = err;
                best_idx = i;
            }
        }
    }
    if (min_err > 0.05) return "Unknown";
    return box_names[best_idx];
}

std::string box_perception::estimate_box_type_multi(const std::vector<double>& lengths) {
    const double box_sizes[][3] = {
        {0.3, 0.3, 0.3},   // A
        {0.4, 0.4, 0.4},   // B
        {0.5, 0.5, 0.5},   // C
        {0.2, 0.2, 0.8},   // D
        {0.3, 0.3, 1.0}    // E
    };
    const char* box_names[] = {"A", "B", "C", "D", "E"};
    double min_err = 1e9;
    int best_idx = -1;
    for (int i = 0; i < 5; ++i) {
        double err = 0;
        for (auto len : lengths) {
            double min_face_err = 1e9;
            for (int j = 0; j < 3; ++j) {
                double face_err = std::abs(len - box_sizes[i][j]);
                if (face_err < min_face_err) min_face_err = face_err;
            }
            err += min_face_err;
        }
        if (err < min_err) {
            min_err = err;
            best_idx = i;
        }
    }
    if (min_err > 0.05 * lengths.size()) return "Unknown";
    return box_names[best_idx];
}

void box_perception::box_array_callback(const nhk2025b_msgs::msg::BoxArray::SharedPtr msg) {
    last_box_state = *msg;
}

void box_perception::is_red_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    is_red = msg->data;
}

} // namespace box_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(box_perception::box_perception)
