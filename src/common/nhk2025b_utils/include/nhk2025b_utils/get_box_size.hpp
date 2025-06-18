#ifndef __nhk2025b_utils_get_box_size_hpp__
#define __nhk2025b_utils_get_box_size_hpp__

#include "nhk2025b_msgs/BoxInfo.hpp"

namespace nhk2025b_utils {
/**
 * @brief Get the size of a box based on its BoxInfo.
 *
 * @param box_info The BoxInfo object containing the dimensions of the box.
 * @return The BoxSize object representing the size of the box.
 */
geometry_msgs::msg::Vector3 get_box_size (const nhk2025b_msgs::BoxInfo& box_info) {
    using nhk2025b_msgs::msg::BoxInfo;
    geometry_msgs::msg::Vector3 box_size;
    switch (box_info.type) {
        case BoxInfo::A:
            box_size.x = 0.3;
            box_size.y = 0.3;
            box_size.z = 0.3;
            return box_size;
        case BoxInfo::B:
            box_size.x = 0.4;
            box_size.y = 0.4;
            box_size.z = 0.4;
            return box_size;
        case BoxInfo::C:
            box_size.x = 0.5;
            box_size.y = 0.5;
            box_size.z = 0.5;
            return box_size;
        case BoxInfo::D:
            box_size.x = 0.8;
            box_size.y = 0.2;
            box_size.z = 0.2;
            return box_size;
        case BoxInfo::E:
            box_size.x = 1.0;
            box_size.y = 0.3;
            box_size.z = 0.3;
            return box_size;
        default:
            throw std::invalid_argument ("Unknown box type");
    }
}

}  // namespace nhk2025b_utils
#endif  // __nhk2025b_utils_get_box_size_hpp__