#ifndef __nhk2025b_utils_get_box_size_hpp__
#define __nhk2025b_utils_get_box_size_hpp__

#include "nhk2025b_msgs/msg/box_info.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <stdexcept>
#include <geometry_msgs/msg/vector3.hpp>

namespace nhk2025b_utils {
/**
 * @brief Get the size of a box based on its BoxInfo.
 *
 * @param box_info The BoxInfo object containing the dimensions of the box.
 * @return The BoxSize object representing the size of the box.
 */
geometry_msgs::msg::Vector3 get_box_size (const nhk2025b_msgs::msg::BoxInfo& box_info);

};  // namespace nhk2025b_utils
#endif  // __nhk2025b_utils_get_box_size_hpp__