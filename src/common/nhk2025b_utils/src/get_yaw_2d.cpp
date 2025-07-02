#include "nhk2025b_utils/get_yaw_2d.hpp"

namespace nhk2025b_utils {
double get_yaw_2d (const geometry_msgs::msg::Quaternion &orientation) {
    return std::atan2 (2.0 * (orientation.z * orientation.w), 1.0 - 2.0 * (orientation.z * orientation.z));
}
}  // namespace nhk2025b_utils
