#include "nhk2025b_scan_matcher/ndt.hpp"

namespace ndt {
ndt::ndt(const rclcpp::NodeOptions & options) : Node("ndte", options) {
    // TFの初期化
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_  = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // サブスクライバの初期化
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/sensor/scan", rclcpp::SensorDataQoS(), std::bind(&ndt::scanCallback, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/behavior/map", 10, std::bind(&ndt::mapCallback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/localization/ekf/pose", 10, std::bind(&ndt::poseCallback, this, std::placeholders::_1));

    // パブリッシャの初期化
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/current_pose", 10);

    this->declare_parameter("ndt_resolution", 1.0);
    this->declare_parameter("ndt_step_size", 0.1);
    this->declare_parameter("ndt_transformation_epsilon", 0.01);
    this->declare_parameter("ndt_max_iterations", 30);
}

void ndt::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
    current_ekf_pose_ = *pose_msg;
}

void ndt::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
    map_cloud_ = convertMapToPointCloud(*map_msg);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ndt::convertMapToPointCloud(const nav_msgs::msg::OccupancyGrid & map) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    int width = map.info.width;
    int height = map.info.height;
    float resolution = map.info.resolution;
    float origin_x = map.info.origin.position.x;
    float origin_y = map.info.origin.position.y;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = x + y * width;
            if (map.data[index] > 50) { // occupied
                pcl::PointXYZ point;
                point.x = origin_x + x * resolution;
                point.y = origin_y + y * resolution;
                point.z = 0.0;
                cloud->points.push_back(point);
            }
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}

void ndt::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    // LaserScan to PointCloud2 (manual conversion or via laser_geometry if needed)
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float r = scan_msg->ranges[i];
        if (std::isfinite(r)) {
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            pcl::PointXYZ point;
            point.x = r * std::cos(angle);
            point.y = r * std::sin(angle);
            point.z = 0.0;
            input_cloud->points.push_back(point);
        }
    }
    input_cloud->width = input_cloud->points.size();
    input_cloud->height = 1;
    input_cloud->is_dense = true;

    // NDTの準備と実行
    if (!map_cloud_ || map_cloud_->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Map not received yet.");
        return;
    }

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setInputSource(input_cloud);
    ndt.setInputTarget(map_cloud_);

    ndt.setMaximumIterations(this->get_parameter("ndt_max_iterations").as_int());
    ndt.setTransformationEpsilon(this->get_parameter("ndt_transformation_epsilon").as_double());
    ndt.setStepSize(this->get_parameter("ndt_step_size").as_double());
    ndt.setResolution(this->get_parameter("ndt_resolution").as_double());

    double ekf_delta_x = current_ekf_pose_.pose.position.x - last_ekf_pose_.pose.position.x;
    double ekf_delta_y = current_ekf_pose_.pose.position.y - last_ekf_pose_.pose.position.y;
    double ekf_delta_theta = tf2::getYaw(current_ekf_pose_.pose.orientation) - tf2::getYaw(last_ekf_pose_.pose.orientation);
    last_ekf_pose_ = current_ekf_pose_;

    // 初期位置の推定
    Eigen::Translation3f init_translation(last_pose_.pose.position.x + ekf_delta_x, last_pose_.pose.position.y + ekf_delta_y, 0.0);
    Eigen::AngleAxisf init_rotation(Eigen::AngleAxisf(tf2::getYaw(last_pose_.pose.orientation)+ekf_delta_theta, Eigen::Vector3f::UnitZ()));
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    pcl::PointCloud<pcl::PointXYZ> aligned;
    ndt.align(aligned, init_guess);

    if (!ndt.hasConverged()) {
        RCLCPP_WARN(this->get_logger(), "NDT did not converge.");
        return;
    }

    // 結果の取得と変換
    Eigen::Matrix4f result = ndt.getFinalTransformation();
    geometry_msgs::msg::PoseStamped result_pose;
    result_pose.header.stamp = this->now();
    result_pose.header.frame_id = "map";
    result_pose.pose.position.x = result(0, 3);
    result_pose.pose.position.y = result(1, 3);
    result_pose.pose.position.z = 0.0;

    Eigen::Quaternionf q(Eigen::Matrix3f(result.block<3, 3>(0, 0)));
    result_pose.pose.orientation.x = q.x();
    result_pose.pose.orientation.y = q.y();
    result_pose.pose.orientation.z = q.z();
    result_pose.pose.orientation.w = q.w();

    // パブリッシュ
    pose_pub_->publish(result_pose);
    last_pose_ = result_pose;

    // PointCloud2のパブリッシュ
    // pcl::toROSMsg(aligned, cloud_msg);
    // cloud_msg.header.stamp = this->now();
    // cloud_msg.header.frame_id = "base_link";
    // pointcloud_pub_->publish(cloud_msg);

    // TF
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = result_pose.pose.position.x;
    transform.transform.translation.y = result_pose.pose.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = result_pose.pose.orientation.x;
    transform.transform.rotation.y = result_pose.pose.orientation.y;
    transform.transform.rotation.z = result_pose.pose.orientation.z;
    transform.transform.rotation.w = result_pose.pose.orientation.w;
    tf_broadcaster_->sendTransform(transform);

    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f", result_pose.pose.position.x, result_pose.pose.position.y, tf2::getYaw(result_pose.pose.orientation));
}



}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ndt::ndt)