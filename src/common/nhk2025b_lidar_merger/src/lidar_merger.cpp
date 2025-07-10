#include "nhk2025b_lidar_merger/lidar_mermer.hpp"

namespace lidar_merger {
lidar_merger::lidar_merger (const rclcpp::NodeOptions &options)
    : Node ("lidar_merger", options),
      tf_buffer_ (std::make_shared<tf2_ros::Buffer> (this->get_clock ())),
      tf_listener_ (std::make_shared<tf2_ros::TransformListener> (*tf_buffer_)) {
    scan1_sub = this->create_subscription<sensor_msgs::msg::LaserScan> (
        "/sensor/lidar/front/scan", rclcpp::SensorDataQoS (), [this] (const sensor_msgs::msg::LaserScan::SharedPtr scan1_msg) {
            scan1 = *scan1_msg;
            publish_merged_point_cloud2 ();
        });

    scan2_sub = this->create_subscription<sensor_msgs::msg::LaserScan> (
        "/sensor/lidar/rear/scan", rclcpp::SensorDataQoS (), [this] (const sensor_msgs::msg::LaserScan::SharedPtr scan2_msg) {
            scan2 = *scan2_msg;
            publish_merged_point_cloud2 ();
        });
    pointcloud2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2> ("/sensor/lidar", rclcpp::SensorDataQoS ());
}

void lidar_merger::publish_merged_point_cloud2 () {
    if (scan1.ranges.empty () && scan2.ranges.empty ()) {
        // どちらかのスキャンが未受信なら何もしない
        return;
    }
    sensor_msgs::msg::PointCloud2    merged_cloud;
    sensor_msgs::PointCloud2Modifier modifier (merged_cloud);
    modifier.setPointCloud2FieldsByString (1, "xyz");
    modifier.resize (scan1.ranges.size () + scan2.ranges.size ());
    sensor_msgs::PointCloud2Iterator<float> iter_x (merged_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y (merged_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z (merged_cloud, "z");

    merged_cloud.header.frame_id = "base_link";
    merged_cloud.header.stamp    = this->now ();
    double robot_width           = 0.8;
    double robot_length          = 0.6;

    if (!scan1.ranges.empty ()) {
        // scan1のtf -> base_link変換
        geometry_msgs::msg::TransformStamped transform1;
        try {
            transform1 = tf_buffer_->lookupTransform ("base_link", scan1.header.frame_id, scan1.header.stamp, rclcpp::Duration::from_seconds (0.1));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN (this->get_logger (), "Transform lookup failed for scan1: %s", ex.what ());
            return;
        }
        // scan1の点群を変換して追加
        for (size_t i = 0; i < scan1.ranges.size (); ++i) {
            if (std::isfinite (scan1.ranges[i]) && scan1.ranges[i] > scan1.range_min && scan1.ranges[i] < scan1.range_max) {
                float angle = scan1.angle_min + i * scan1.angle_increment;
                float x     = scan1.ranges[i] * std::cos (angle);
                float y     = scan1.ranges[i] * std::sin (angle);
                float z     = 0.0;  // ライダーは通常2Dなのでzは0
                // Transform the point
                geometry_msgs::msg::PointStamped point_in, point_out;
                point_in.header  = scan1.header;
                point_in.point.x = x;
                point_in.point.y = y;
                point_in.point.z = z;
                tf2::doTransform (point_in, point_out, transform1);
                if ((point_out.point.x > -robot_length / 2.0 && point_out.point.x < robot_length / 2.0) &&
                    (point_out.point.y > -robot_width / 2.0 && point_out.point.y < robot_width / 2.0)) {
                    continue;
                }

                *iter_x = point_out.point.x;
                *iter_y = point_out.point.y;
                *iter_z = point_out.point.z;
                ++iter_x;
                ++iter_y;
                ++iter_z;
            }
        }
    }
    if (!scan2.ranges.empty ()) {
        // scan2のtf -> base_link変換
        geometry_msgs::msg::TransformStamped transform2;
        try {
            transform2 = tf_buffer_->lookupTransform ("base_link", scan2.header.frame_id, scan2.header.stamp, rclcpp::Duration::from_seconds (0.1));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN (this->get_logger (), "Transform lookup failed for scan2: %s", ex.what ());
            return;
        }
        // scan2の点群を変換して追加
        for (size_t i = 0; i < scan2.ranges.size (); ++i) {
            if (std::isfinite (scan2.ranges[i]) && scan2.ranges[i] > scan2.range_min && scan2.ranges[i] < scan2.range_max) {
                float angle = scan2.angle_min + i * scan2.angle_increment;
                float x     = scan2.ranges[i] * std::cos (angle);
                float y     = scan2.ranges[i] * std::sin (angle);
                float z     = 0.0;  // ライダーは通常2Dなのでzは0
                // Transform the point
                geometry_msgs::msg::PointStamped point_in, point_out;
                point_in.header  = scan2.header;
                point_in.point.x = x;
                point_in.point.y = y;
                point_in.point.z = z;
                tf2::doTransform (point_in, point_out, transform2);
                if ((point_out.point.x > -robot_length / 2.0 && point_out.point.x < robot_length / 2.0) &&
                    (point_out.point.y > -robot_width / 2.0 && point_out.point.y < robot_width / 2.0)) {
                    continue;
                }
                *iter_x = point_out.point.x;
                *iter_y = point_out.point.y;
                *iter_z = point_out.point.z;
                ++iter_x;
                ++iter_y;
                ++iter_z;
            }
        }
    }
    pointcloud2_pub->publish (merged_cloud);
}
}  // namespace lidar_merger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (lidar_merger::lidar_merger)
