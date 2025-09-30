#include "nhk2025b_box_perception/box_perception.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace box_perception {

box_perception::box_perception (const rclcpp::NodeOptions& options) : Node ("box_perception", options) {
    box_state_publisher_    = this->create_publisher<nhk2025b_msgs::msg::BoxArray> ("/box_state", 10);
    point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2> ("/sensor/lidar", 10, std::bind (&box_perception::point_cloud_callback, this, std::placeholders::_1));
    box_array_subscriber_   = this->create_subscription<nhk2025b_msgs::msg::BoxArray> ("/box_state", 10, std::bind (&box_perception::box_array_callback, this, std::placeholders::_1));
    map_subscriber_         = this->create_subscription<nav_msgs::msg::OccupancyGrid> ("/behavior/map", 10, std::bind (&box_perception::map_callback, this, std::placeholders::_1));
    is_red_subscriber_      = this->create_subscription<std_msgs::msg::Bool> ("/is_red", 1, std::bind (&box_perception::is_red_callback, this, std::placeholders::_1));

    tf_buffer_   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);
}

void box_perception::is_red_callback (const std_msgs::msg::Bool::SharedPtr msg) {}

void box_perception::map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr map) {}

void box_perception::box_array_callback (const nhk2025b_msgs::msg::BoxArray::SharedPtr msg) {
    prev_box_array_ = *msg;
}

void box_perception::point_cloud_callback (const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    static std::vector<Cluster> prev_clusters;
    static rclcpp::Time         prev_stamp;

    float cluster_tolerance = 0.25;
    auto  clusters          = cluster_points (*cloud, cluster_tolerance);

    std::vector<bool> moved;
    match_clusters (prev_clusters, clusters, moved, 0.05);

    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform ("map", cloud->header.frame_id, cloud->header.stamp);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN (this->get_logger (), "TF lookup failed: %s", ex.what ());
        return;
    }

    nhk2025b_msgs::msg::BoxArray box_array;
    for (size_t i = 0; i < clusters.size (); ++i) {
        auto&   c     = clusters[i];
        float   width = c.max_x - c.min_x;
        float   depth = c.max_y - c.min_y;
        char    label = assign_label (width, depth);
        uint8_t box_type;

        geometry_msgs::msg::Pose         pose;
        geometry_msgs::msg::PointStamped pt_in, pt_out;
        pt_in.header.frame_id = cloud->header.frame_id;
        pt_in.point.x         = c.centroid.x;
        pt_in.point.y         = c.centroid.y;
        pt_in.point.z         = 0.0;
        tf2::doTransform (pt_in, pt_out, tf);
        pose.position      = pt_out.point;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        nhk2025b_msgs::msg::Box box;
        box.pose      = pose;
        box.info.type = box_type;
        if (box_type == 1) {
            box.size.x = 0.3;
            box.size.y = 0.3;
            box.size.z = 0.3;
        } else if (box_type == 2) {
            box.size.x = 0.4;
            box.size.y = 0.4;
            box.size.z = 0.4;
        } else if (box_type == 3) {
            box.size.x = 0.5;
            box.size.y = 0.5;
            box.size.z = 0.5;
        } else if (box_type == 4) {
            box.size.x = 0.2;
            box.size.y = 0.2;
            box.size.z = 0.8;
        } else if (box_type == 5) {
            box.size.x = 0.3;
            box.size.y = 0.3;
            box.size.z = 1.0;
        } else {
            box.size.x = width;
            box.size.y = depth;
            box.size.z = 0.3;
        }

        box_array.boxes.push_back (box);
    }

    for (size_t i = 0; i < box_array.boxes.size (); ++i) {
        const auto& box   = box_array.boxes[i];
        bool        found = false;
        for (const auto& prev_box : prev_box_array_.boxes) {
            if (box.info.type == prev_box.info.type) {
                double dx   = box.pose.position.x - prev_box.pose.position.x;
                double dy   = box.pose.position.y - prev_box.pose.position.y;
                double dist = std::hypot (dx, dy);
                if (dist < 0.05) {
                    found = true;
                    if (dist > 0.05) {
                        RCLCPP_INFO (this->get_logger (), "Box[%zu] type=%d moved! Δ=(%.2f, %.2f)", i, box.info.type, dx, dy);
                    }
                    break;
                }
            }
        }
        if (!found) {
            RCLCPP_INFO (this->get_logger (), "Box[%zu] type=%d appeared! pos=(%.2f, %.2f)", i, box.info.type, box.pose.position.x, box.pose.position.y);
        }
    }

    for (const auto& prev_box : prev_box_array_.boxes) {
        bool still_exists = false;
        for (const auto& box : box_array.boxes) {
            if (box.info.type == prev_box.info.type) {
                double dx   = box.pose.position.x - prev_box.pose.position.x;
                double dy   = box.pose.position.y - prev_box.pose.position.y;
                double dist = std::hypot (dx, dy);
                if (dist < 0.05) {
                    still_exists = true;
                    break;
                }
            }
        }
        if (!still_exists) {
            RCLCPP_INFO (this->get_logger (), "Box type=%d disappeared! prev_pos=(%.2f, %.2f)", prev_box.info.type, prev_box.pose.position.x, prev_box.pose.position.y);
        }
    }

    box_state_publisher_->publish (box_array);
    prev_box_array_ = box_array;

    prev_clusters = clusters;
    prev_stamp    = cloud->header.stamp;
}

std::vector<Cluster> cluster_points (const sensor_msgs::msg::PointCloud2& cloud, float tolerance) {
    std::vector<Cluster> clusters;
    std::vector<bool>    assigned (cloud.width * cloud.height, false);

    std::vector<geometry_msgs::msg::Point32> pts;
    int                                      x_offset = -1, y_offset = -1;
    int                                      point_step = cloud.point_step;
    for (const auto& field : cloud.fields) {
        if (field.name == "x") x_offset = field.offset;
        if (field.name == "y") y_offset = field.offset;
    }
    size_t num_points = cloud.width * cloud.height;
    for (size_t i = 0; i < num_points; ++i) {
        float                       x = *reinterpret_cast<const float*> (&cloud.data[i * point_step + x_offset]);
        float                       y = *reinterpret_cast<const float*> (&cloud.data[i * point_step + y_offset]);
        geometry_msgs::msg::Point32 pt;
        pt.x = x;
        pt.y = y;
        pt.z = 0.0;
        pts.push_back (pt);
    }

    for (size_t i = 0; i < pts.size (); ++i) {
        if (assigned[i]) continue;
        Cluster cluster;
        cluster.points.push_back (pts[i]);
        assigned[i]   = true;
        cluster.min_x = cluster.max_x = pts[i].x;
        cluster.min_y = cluster.max_y = pts[i].y;

        for (size_t j = i + 1; j < pts.size (); ++j) {
            if (assigned[j]) continue;
            for (const auto& cpt : cluster.points) {
                float dx = pts[j].x - cpt.x;
                float dy = pts[j].y - cpt.y;
                if (dx * dx + dy * dy < tolerance * tolerance) {
                    cluster.points.push_back (pts[j]);
                    assigned[j]   = true;
                    cluster.min_x = std::min (cluster.min_x, pts[j].x);
                    cluster.max_x = std::max (cluster.max_x, pts[j].x);
                    cluster.min_y = std::min (cluster.min_y, pts[j].y);
                    cluster.max_y = std::max (cluster.max_y, pts[j].y);
                    break;
                }
            }
        }

        float sum_x = 0, sum_y = 0;
        for (const auto& pt : cluster.points) {
            sum_x += pt.x;
            sum_y += pt.y;
        }
        cluster.centroid.x = sum_x / cluster.points.size ();
        cluster.centroid.y = sum_y / cluster.points.size ();
        cluster.centroid.z = 0.0;
        clusters.push_back (cluster);
    }
    return clusters;
}

uint32_t assign_label (float width, float depth) {
    if (fabs (width - 0.3) < 0.07 && fabs (depth - 0.3) < 0.07) return 1;
    if (fabs (width - 0.4) < 0.07 && fabs (depth - 0.4) < 0.07) return 2;
    if (fabs (width - 0.5) < 0.07 && fabs (depth - 0.5) < 0.07) return 3;
    if (fabs (width - 0.2) < 0.07 && fabs (depth - 0.8) < 0.07) return 4;
    if (fabs (width - 0.3) < 0.07 && fabs (depth - 1.0) < 0.07) return 5;
    return 0;
}

// クラスタ重心マッチング
void match_clusters (const std::vector<Cluster>& prev, const std::vector<Cluster>& curr, std::vector<bool>& moved, float move_thresh) {
    moved.resize (curr.size (), false);
    for (size_t i = 0; i < curr.size (); ++i) {
        float min_dist = 1e6;
        for (const auto& p : prev) {
            float dx   = curr[i].centroid.x - p.centroid.x;
            float dy   = curr[i].centroid.y - p.centroid.y;
            float dist = std::hypot (dx, dy);
            if (dist < min_dist) min_dist = dist;
        }
        if (min_dist > move_thresh) moved[i] = true;
    }
}

}  // namespace box_perception
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (box_perception::box_perception)
