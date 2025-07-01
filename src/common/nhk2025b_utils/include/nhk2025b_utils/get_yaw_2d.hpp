#ifndef __nhk2025b_utils_get_yaw_2d_hpp__
#define __nhk2025b_utils_get_yaw_2d_hpp__
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>

namespace nhk2025b_utils {
double get_yaw_2d (const geometry_msgs::msg::Quaternion &orientation);
}  // namespace nhk2025b_utils
#endif  // __nhk2025b_utils_get_yaw_2d_hpp__