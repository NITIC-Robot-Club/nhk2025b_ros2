#include "nhk2025b_scan_matcher/mcl.hpp"

namespace mcl {

mcl::mcl (const rclcpp::NodeOptions& options)
    : Node ("mcl", options),
      tf_buffer_ (std::make_shared<tf2_ros::Buffer> (this->get_clock ())),
      tf_listener_ (std::make_shared<tf2_ros::TransformListener> (*tf_buffer_)),
      tf_broadcaster_ (std::make_unique<tf2_ros::TransformBroadcaster> (*this)),
      rng_ (std::random_device{}()) {
    num_particles_           = this->declare_parameter ("num_particles", 150);
    motion_noise_linear_     = this->declare_parameter ("motion_noise_linear", 0.01);
    motion_noise_angle_      = this->declare_parameter ("motion_noise_angle", 0.01);
    gaussian_stddev_linear_  = this->declare_parameter ("gaussian_stddev_linear", 1.0);
    gaussian_stddev_angle_   = this->declare_parameter ("gaussian_stddev_angle", 1.0);
    random_particle_map_num_ = this->declare_parameter ("random_particle_map_num", 100);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan> (
        "/sensor/scan", rclcpp::SensorDataQoS (), std::bind (&mcl::scan_callback, this, std::placeholders::_1));
    map_sub_ =
        this->create_subscription<nav_msgs::msg::OccupancyGrid> ("/behavior/map", 10, std::bind (&mcl::map_callback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped> (
        "/localization/initialpose", 10, std::bind (&mcl::pose_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry> (
        "/localization/wheel_odometry", 10, std::bind (&mcl::odom_callback, this, std::placeholders::_1));
    distance_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid> ("/localization/distance_map", 10);

    pose_pub_      = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/localization/current_pose", 10);
    particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray> ("/localization/mcl_particles", 10);
    twist_pub_      = this->create_publisher<geometry_msgs::msg::TwistStamped> ("/localization/velocity", 10);
}

void mcl::create_distance_map () {
    if (!map_) return;

    nav_msgs::msg::OccupancyGrid distance_map_msg;
    distance_map_msg.header.frame_id = "map";
    distance_map_msg.header.stamp    = this->now ();
    distance_map_msg.info            = map_->info;
    distance_map_msg.data.resize (map_->info.width * map_->info.height, 0);

    distance_map_.resize (map_->info.width * map_->info.height, std::numeric_limits<double>::max ());

    std::queue<std::pair<int, int>> queue;
    for (int y = 0; y < static_cast<int> (map_->info.height); ++y) {
        for (int x = 0; x < static_cast<int> (map_->info.width); ++x) {
            int index = y * map_->info.width + x;
            if (map_->data[index] > 50) {
                distance_map_[index] = 0.0;
                queue.emplace (x, y);
            }
        }
    }

    const std::vector<std::pair<int, int>> directions = {
        { 1,  0},
        {-1,  0},
        { 0,  1},
        { 0, -1}
    };
    while (!queue.empty ()) {
        auto [cx, cy] = queue.front ();
        queue.pop ();

        int current_index = cy * map_->info.width + cx;
        for (const auto& [dx, dy] : directions) {
            int nx = cx + dx;
            int ny = cy + dy;
            if (nx >= 0 && nx < static_cast<int> (map_->info.width) && ny >= 0 && ny < static_cast<int> (map_->info.height)) {
                int    neighbor_index = ny * map_->info.width + nx;
                double new_distance   = distance_map_[current_index] + map_->info.resolution;
                if (new_distance < distance_map_[neighbor_index]) {
                    distance_map_[neighbor_index] = new_distance;
                    queue.emplace (nx, ny);
                }
            }
        }
    }

    for (size_t i = 0; i < distance_map_.size (); ++i) {
        if (distance_map_[i] == std::numeric_limits<double>::max ()) {
            distance_map_[i] = -1.0;
        }
        distance_map_msg.data[i] = static_cast<int8_t> (distance_map_[i] / map_->info.resolution);
    }

    distance_map_pub_->publish (distance_map_msg);
}

void mcl::map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
    if (!map_) {
        map_ = map_msg;
        create_distance_map ();
        initialize_particles_gaussian (map_->info.origin);
    }
    map_ = map_msg;
    create_distance_map ();
}

void mcl::pose_callback (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
    current_pose_ = pose_msg->pose.pose;  // Extract pose from PoseWithCovarianceStamped

    this->get_parameter ("num_particles", num_particles_);
    this->get_parameter ("motion_noise_linear", motion_noise_linear_);
    this->get_parameter ("motion_noise_angle", motion_noise_angle_);
    this->get_parameter ("gaussian_stddev_linear", gaussian_stddev_linear_);
    this->get_parameter ("gaussian_stddev_angle", gaussian_stddev_angle_);
    this->get_parameter ("random_particle_map_num", random_particle_map_num_);
    initialize_particles_gaussian (current_pose_);
}

void mcl::odom_callback (const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    current_pose_ = odom_msg->pose.pose;
}

void mcl::scan_callback (const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    if (!map_ || particles_.empty ()) return;
    
    motion_update (current_pose_, last_pose_);
    sensor_update (*scan_msg);
    resample_particles ();
    
    
    geometry_msgs::msg::PoseStamped estimated;
    estimated.pose            = estimate_pose ();
    estimated.header.frame_id = "map";
    estimated.header.stamp    = this->now ();
    
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    pose_array.header.stamp    = this->now ();
    for (const auto& p : particles_) {
        geometry_msgs::msg::Pose pose;
        pose.position.x  = p.x;
        pose.position.y  = p.y;
        pose.orientation = tf2::toMsg (tf2::Quaternion (0, 0, sin (p.theta / 2.0), cos (p.theta / 2.0)));
        pose_array.poses.push_back (pose);
    }
    particles_pub_->publish (pose_array);
    
    // publish tf
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp            = this->now ();
    transform.header.frame_id         = "map";
    transform.child_frame_id          = "base_link";
    transform.transform.translation.x = estimated.pose.position.x;
    transform.transform.translation.y = estimated.pose.position.y;
    transform.transform.rotation.z    = estimated.pose.orientation.z;
    transform.transform.rotation.w    = estimated.pose.orientation.w;
    tf_broadcaster_->sendTransform (transform);
    
    if(! is_converged()) {
        RCLCPP_WARN (this->get_logger (), "Particles not converged");
        estimated.pose.position.x = last_estimated_pose_.pose.position.x + (current_pose_.position.x - last_pose_.position.x);
        estimated.pose.position.y = last_estimated_pose_.pose.position.y + (current_pose_.position.y - last_pose_.position.y);
        estimated.pose.orientation.z = last_estimated_pose_.pose.orientation.z + (current_pose_.orientation.z - last_pose_.orientation.z);
        estimated.pose.orientation.w = last_estimated_pose_.pose.orientation.w + (current_pose_.orientation.w - last_pose_.orientation.w);
    }
    
    pose_pub_->publish (estimated);
    
    if(!last_estimated_pose_.header.stamp.sec) {
        last_estimated_pose_ = estimated;
    } else {
        double dt = (rclcpp::Time(estimated.header.stamp) - rclcpp::Time(last_estimated_pose_.header.stamp)).seconds();
        geometry_msgs::msg::TwistStamped twist;
        twist.header.frame_id = "base_link";
        twist.header.stamp    = this->now ();
        twist.twist.linear.x  = (estimated.pose.position.x-last_estimated_pose_.pose.position.x)/dt;
        twist.twist.linear.y  = (estimated.pose.position.y-last_estimated_pose_.pose.position.y)/dt;
        twist.twist.angular.z = (estimated.pose.orientation.z-last_estimated_pose_.pose.orientation.z)/dt;
        twist_pub_->publish (twist);
        last_estimated_pose_ = estimated;
    }
    
    last_pose_ = current_pose_;
}

void mcl::initialize_particles_gaussian (const geometry_msgs::msg::Pose& initial_pose) {
    if (!map_) return;
    particles_.clear ();

    // Define Gaussian distributions for x, y, and theta based on the initial pose
    std::normal_distribution<double> dist_x (initial_pose.position.x, gaussian_stddev_linear_);
    std::normal_distribution<double> dist_y (initial_pose.position.y, gaussian_stddev_linear_);
    tf2::Quaternion                  q;
    tf2::fromMsg (initial_pose.orientation, q);
    double                           initial_theta = tf2::getYaw (q);
    std::normal_distribution<double> dist_theta (initial_theta, gaussian_stddev_angle_);

    while (particles_.size () < static_cast<size_t> (num_particles_)) {
        double x = dist_x (rng_);
        double y = dist_y (rng_);
        if (!is_pose_valid (x, y)) continue;
        Particle p{x, y, dist_theta (rng_), 1.0 / num_particles_};
        particles_.push_back (p);
    }

    RCLCPP_INFO (this->get_logger (), "Initialized %zu particles", particles_.size ());
}

void mcl::motion_update (const geometry_msgs::msg::Pose& current, const geometry_msgs::msg::Pose& last) {
    double dx     = current.position.x - last.position.x;
    double dy     = current.position.y - last.position.y;
    double dtheta = tf2::getYaw (current.orientation) - tf2::getYaw (last.orientation);

    std::normal_distribution<double> noise_x (0.0, motion_noise_linear_);
    std::normal_distribution<double> noise_y (0.0, motion_noise_linear_);
    std::normal_distribution<double> noise_theta (0.0, motion_noise_angle_);

    double cos_last_theta = cos (tf2::getYaw (last.orientation));
    double sin_last_theta = sin (tf2::getYaw (last.orientation));

    for (auto& p : particles_) {
        // Transform the motion to the particle's frame of reference
        double local_dx = cos_last_theta * dx + sin_last_theta * dy;
        double local_dy = -sin_last_theta * dx + cos_last_theta * dy;

        double cos_theta = cos (p.theta);
        double sin_theta = sin (p.theta);

        // Apply the motion update with noise
        p.x += cos_theta * local_dx - sin_theta * local_dy + noise_x (rng_);
        p.y += sin_theta * local_dx + cos_theta * local_dy + noise_y (rng_);
        p.theta += dtheta + noise_theta (rng_);
    }
}

void mcl::sensor_update (const sensor_msgs::msg::LaserScan& scan) {
    double total_weight = 0.0;
    int score_max = 0;
    int score_min = 1000;
    for (auto& p : particles_) {
        p.weight = compute_likelihood (p, scan);
        total_weight += p.weight;
        if (p.weight > score_max) {
            score_max = p.weight;
        }
        if (p.weight < score_min) {
            score_min = p.weight;
        }
    }

    for (auto& p : particles_) {
        p.weight /= (total_weight + 1e-6);
    }
    // RCLCPP_INFO (this->get_logger (), "Max weight: %d, Min weight: %d, Average weight: %f", score_max, score_min, total_weight / particles_.size ());
}

double mcl::compute_likelihood (const Particle& p, const sensor_msgs::msg::LaserScan& scan) const {
    if (!map_ || distance_map_.empty ()) return 0.0;

    double       score     = 0.0;
    const double max_range = scan.range_max;
    const double sigma_hit = 0.2;
    const double z_hit     = 0.8;
    const double z_rand    = 0.2;

    for (size_t i = 0; i < scan.ranges.size (); i++) {
        double range = scan.ranges[i];
        if (std::isnan (range) || range > max_range) continue;

        double angle = scan.angle_min + i * scan.angle_increment + p.theta;
        double hit_x = p.x + range * cos (angle);
        double hit_y = p.y + range * sin (angle);

        int mx = static_cast<int> ((hit_x - map_->info.origin.position.x) / map_->info.resolution);
        int my = static_cast<int> ((hit_y - map_->info.origin.position.y) / map_->info.resolution);

        if (mx >= 0 && mx < static_cast<int> (map_->info.width) && my >= 0 && my < static_cast<int> (map_->info.height)) {
            int    index    = my * map_->info.width + mx;
            double distance = distance_map_[index];
            double prob_hit = exp (-0.5 * pow (distance / sigma_hit, 2)) / (sigma_hit * sqrt (2 * M_PI));
            score += z_hit * prob_hit + z_rand / max_range;
        } else {
            score += z_rand / max_range;
        }
    }
    return score;
}

void mcl::resample_particles () {
    std::vector<Particle> new_particles;
    std::vector<double>   cumulative;
    cumulative.reserve (particles_.size ());

    double sum = 0.0;
    for (const auto& p : particles_) {
        sum += p.weight;
        cumulative.push_back (sum);
    }

    std::uniform_real_distribution<double> dist (0.0, sum);

    for (int i = 0; i < num_particles_; ++i) {
        double   r        = dist (rng_);
        auto     it       = std::lower_bound (cumulative.begin (), cumulative.end (), r);
        int      index    = std::distance (cumulative.begin (), it);
        Particle selected = particles_[index];
        selected.weight   = 1.0 / num_particles_;
        new_particles.push_back (selected);
    }

    random_particle_map_num_ = 0;
    auto pose = estimate_pose ();

    std::uniform_real_distribution<double> dist_map_x (map_->info.origin.position.x, map_->info.origin.position.x +
    map_->info.resolution*map_->info.width); std::uniform_real_distribution<double> dist_map_y (map_->info.origin.position.y,
    map_->info.origin.position.y + map_->info.resolution*map_->info.height);

    for (int i = 0; i < random_particle_map_num_; ++i) {
        for(int j = 0; j < 4; j++){
            Particle p;
            p.x      = dist_map_x (rng_);
            p.y      = dist_map_y (rng_);
            p.weight  = 0;
            p.theta  = j * M_PI / 2 + std::asin(pose.orientation.z) * 2;
            new_particles.push_back (p);
        }
    }

    particles_ = std::move (new_particles);
}

geometry_msgs::msg::Pose mcl::estimate_pose () const {
    geometry_msgs::msg::Pose pose;

    double x = 0.0, y = 0.0, sin_sum = 0.0, cos_sum = 0.0;

    for (const auto& p : particles_) {
        x += p.x * p.weight;
        y += p.y * p.weight;
        sin_sum += sin (p.theta) * p.weight;
        cos_sum += cos (p.theta) * p.weight;
    }

    pose.position.x = x;
    pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY (0, 0, atan2 (sin_sum, cos_sum));
    pose.orientation = tf2::toMsg (q);

    return pose;
}

bool mcl::is_pose_valid (double x, double y) const {
    if (!map_) return false;
    int mx = static_cast<int> ((x - map_->info.origin.position.x) / map_->info.resolution);
    int my = static_cast<int> ((y - map_->info.origin.position.y) / map_->info.resolution);
    if (mx < 0 || mx >= static_cast<int> (map_->info.width) || my < 0 || my >= static_cast<int> (map_->info.height)) {
        return false;
    }
    int index = my * map_->info.width + mx;
    return map_->data[index] < 50;
}

bool mcl::get_transform (
    const std::string& target_frame, const std::string& source_frame, const rclcpp::Time& time,
    geometry_msgs::msg::TransformStamped& transform_out) const {
    try {
        transform_out = tf_buffer_->lookupTransform (target_frame, source_frame, time, rclcpp::Duration::from_seconds (0.1));
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN (this->get_logger (), "Transform lookup failed: %s", ex.what ());
        return false;
    }
}

bool mcl::is_converged() const {
    if (particles_.empty()) return false;

    double mean_x = 0, mean_y = 0, mean_theta = 0;
    for (const auto& p : particles_) {
        mean_x += p.x;
        mean_y += p.y;
        mean_theta += p.theta;
    }
    mean_x /= particles_.size();
    mean_y /= particles_.size();
    mean_theta /= particles_.size();

    double var_x = 0, var_y = 0, var_theta = 0;
    for (const auto& p : particles_) {
        var_x += (p.x - mean_x) * (p.x - mean_x);
        var_y += (p.y - mean_y) * (p.y - mean_y);
        var_theta += std::pow(std::atan2(std::sin(p.theta - mean_theta), std::cos(p.theta - mean_theta)), 2);
    }
    var_x /= particles_.size();
    var_y /= particles_.size();
    var_theta /= particles_.size();

    double position_threshold = std::pow(motion_noise_linear_ * 4, 2);
    double angle_threshold = std::pow(motion_noise_angle_ * 4, 2);

    if(var_x > position_threshold) {
        RCLCPP_WARN (this->get_logger (), "Particle x variance is too high: %f", var_x);
    }
    if(var_y > position_threshold) {
        RCLCPP_WARN (this->get_logger (), "Particle y variance is too high: %f", var_y);
    }
    if(var_theta > angle_threshold) {
        RCLCPP_WARN (this->get_logger (), "Particle theta variance is too high: %f", var_theta);
    }

    return var_x < position_threshold && var_y < position_threshold && var_theta < angle_threshold;
}


}  // namespace mcl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (mcl::mcl)
