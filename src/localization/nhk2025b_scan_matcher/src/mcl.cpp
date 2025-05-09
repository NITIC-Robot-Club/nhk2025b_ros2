#include "nhk2025b_scan_matcher/mcl.hpp"

namespace mcl {

mcl::mcl(const rclcpp::NodeOptions & options)
: Node("mcl_node", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
  tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)),
  rng_(std::random_device{}())
{
  this->declare_parameter("num_particles", 50);
  this->declare_parameter("motion_noise_lin", 0.01);
  this->declare_parameter("motion_noise_ang", 0.01);
  this->declare_parameter("resample_threshold", 0.5);

  this->get_parameter("num_particles", num_particles_);
  this->get_parameter("motion_noise_lin", motion_noise_lin_);
  this->get_parameter("motion_noise_ang", motion_noise_ang_);
  this->get_parameter("resample_threshold", resample_threshold_);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/sensor/scan", rclcpp::SensorDataQoS(), std::bind(&mcl::scan_callback, this, std::placeholders::_1));
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/behavior/map", 10, std::bind(&mcl::map_callback, this, std::placeholders::_1));
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/initialpose", 10, std::bind(&mcl::pose_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/wheel_odometory", 10, std::bind(&mcl::odom_callback, this, std::placeholders::_1));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mcl_pose", 10);
  particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/mcl_particles", 10);
}

void mcl::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
  map_ = map_msg;
  RCLCPP_INFO(this->get_logger(), "Received map");
}

void mcl::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
  last_pose_ = current_pose_;
  current_pose_ = *pose_msg;
  initialize_particles_uniform();
  RCLCPP_INFO(this->get_logger(), "Initialized %d particles", static_cast<int>(particles_.size()));
}

void mcl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  last_pose_ = current_pose_;
  current_pose_.header = odom_msg->header;
  current_pose_.pose = odom_msg->pose.pose;
  RCLCPP_INFO(this->get_logger(), "Received odom");
}

void mcl::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
  if (!map_ || particles_.empty()) return;

  RCLCPP_INFO(this->get_logger(), "Received scan");
  motion_update(current_pose_, last_pose_);
  sensor_update(*scan_msg);
  resample_particles();

  auto estimated = estimate_pose();
  pose_pub_->publish(estimated);

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.frame_id = "map";
  pose_array.header.stamp = this->now();
  for (const auto& p : particles_) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(p.theta/2.0), cos(p.theta/2.0)));
    pose_array.poses.push_back(pose);
  }
  particles_pub_->publish(pose_array);
}

void mcl::initialize_particles_uniform() {
  if (!map_) return;

  particles_.clear();
  std::uniform_real_distribution<double> dist_x(map_->info.origin.position.x,
                                                map_->info.origin.position.x + map_->info.width * map_->info.resolution);
  std::uniform_real_distribution<double> dist_y(map_->info.origin.position.y,
                                                map_->info.origin.position.y + map_->info.height * map_->info.resolution);
  std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);

  while (particles_.size() < static_cast<size_t>(num_particles_)) {
    double x = dist_x(rng_);
    double y = dist_y(rng_);
    if (!is_pose_valid(x, y)) continue;
    Particle p{x, y, dist_theta(rng_), 1.0 / num_particles_};
    particles_.push_back(p);
  }
}

void mcl::motion_update(const geometry_msgs::msg::PoseStamped & current,
                        const geometry_msgs::msg::PoseStamped & last) {
  double dx = current.pose.position.x - last.pose.position.x;
  double dy = current.pose.position.y - last.pose.position.y;
  double dtheta = tf2::getYaw(current.pose.orientation) - tf2::getYaw(last.pose.orientation);

  std::normal_distribution<double> noise_x(0.0, motion_noise_lin_);
  std::normal_distribution<double> noise_y(0.0, motion_noise_lin_);
  std::normal_distribution<double> noise_theta(0.0, motion_noise_ang_);

  for (auto & p : particles_) {
    double cos_theta = cos(p.theta);
    double sin_theta = sin(p.theta);
    p.x += cos_theta * dx - sin_theta * dy + noise_x(rng_);
    p.y += sin_theta * dx + cos_theta * dy + noise_y(rng_);
    p.theta += dtheta + noise_theta(rng_);
  }
}

void mcl::sensor_update(const sensor_msgs::msg::LaserScan & scan) {
  double total_weight = 0.0;
  for (auto & p : particles_) {
    p.weight = compute_likelihood(p, scan);
    total_weight += p.weight;
  }

  for (auto & p : particles_) {
    p.weight /= (total_weight + 1e-6);
  }
}

double mcl::compute_likelihood(const Particle & p,
                               const sensor_msgs::msg::LaserScan & scan) const {
  if (!map_) return 0.0;

  double score = 0.0;
  const double max_range = scan.range_max;
  const double sigma_hit = 0.2;
  const double z_hit = 0.8;
  const double z_rand = 0.2;

  for (size_t i = 0; i < scan.ranges.size(); i += 10) {
    double range = scan.ranges[i];
    if (std::isnan(range) || range > max_range) continue;

    double angle = scan.angle_min + i * scan.angle_increment + p.theta;
    double hit_x = p.x + range * cos(angle);
    double hit_y = p.y + range * sin(angle);

    int mx = static_cast<int>((hit_x - map_->info.origin.position.x) / map_->info.resolution);
    int my = static_cast<int>((hit_y - map_->info.origin.position.y) / map_->info.resolution);

    if (mx >= 0 && mx < static_cast<int>(map_->info.width) &&
        my >= 0 && my < static_cast<int>(map_->info.height)) {
      int index = my * map_->info.width + mx;
      int occ = map_->data[index];
      if (occ > 50) {
        double expected_range = range;
        double prob_hit = exp(-0.5 * pow((0.0) / sigma_hit, 2)) / (sigma_hit * sqrt(2 * M_PI));
        score += z_hit * prob_hit + z_rand / max_range;
      } else {
        score += z_rand / max_range;
      }
    }
  }

  return score;
}

void mcl::resample_particles() {
  std::vector<Particle> new_particles;
  std::vector<double> cumulative;
  cumulative.reserve(particles_.size());

  double sum = 0.0;
  for (const auto & p : particles_) {
    sum += p.weight;
    cumulative.push_back(sum);
  }

  std::uniform_real_distribution<double> dist(0.0, sum);

  for (int i = 0; i < num_particles_; ++i) {
    double r = dist(rng_);
    auto it = std::lower_bound(cumulative.begin(), cumulative.end(), r);
    int index = std::distance(cumulative.begin(), it);
    Particle selected = particles_[index];
    selected.weight = 1.0 / num_particles_;
    new_particles.push_back(selected);
  }

  particles_ = std::move(new_particles);
}

geometry_msgs::msg::PoseStamped mcl::estimate_pose() const {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = rclcpp::Clock().now();

  double x = 0.0, y = 0.0, sin_sum = 0.0, cos_sum = 0.0;

  for (const auto & p : particles_) {
    x += p.x * p.weight;
    y += p.y * p.weight;
    sin_sum += sin(p.theta) * p.weight;
    cos_sum += cos(p.theta) * p.weight;
  }

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0, 0, atan2(sin_sum, cos_sum));
  pose.pose.orientation = tf2::toMsg(q);

  return pose;
}

bool mcl::is_pose_valid(double x, double y) const {
  if (!map_) return false;
  int mx = static_cast<int>((x - map_->info.origin.position.x) / map_->info.resolution);
  int my = static_cast<int>((y - map_->info.origin.position.y) / map_->info.resolution);
  if (mx < 0 || mx >= static_cast<int>(map_->info.width) ||
      my < 0 || my >= static_cast<int>(map_->info.height)) {
    return false;
  }
  int index = my * map_->info.width + mx;
  return map_->data[index] < 50;
}

bool mcl::get_transform(const std::string & target_frame,
                        const std::string & source_frame,
                        const rclcpp::Time & time,
                        geometry_msgs::msg::TransformStamped & transform_out) const {
  try {
    transform_out = tf_buffer_->lookupTransform(target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.1));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
    return false;
  }
}

} // namespace mcl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mcl::mcl)
