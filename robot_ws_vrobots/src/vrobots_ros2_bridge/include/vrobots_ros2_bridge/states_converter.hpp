#pragma once

#include <vector>
#include <optional>
#include <cstdint>
#include "vrobots_ros2_msg/msg/states.hpp"

class StatesConverter {
public:
    static std::optional<vrobots_ros2_msg::msg::States> convert(const std::vector<uint8_t>& data);
};
