#pragma once

#include <vector>
#include <string>
#include "vrobots_ros2_msg/msg/commands.hpp"
#include "vrobots_ros2_bridge/C000_commands/C000_commands_generated.h"
#include "flatbuffers/flatbuffers.h"

class CommandsConverter {
public:
  static std::vector<uint8_t> convert(const vrobots_ros2_msg::msg::Commands& msg) {
    flatbuffers::FlatBufferBuilder builder(1024);

    auto name = builder.CreateString(msg.name);
    
    // Arrays
    auto int_arr = builder.CreateVector(msg.int_arr);
    auto float_arr = builder.CreateVector(msg.float_arr);

    // Vec3
    auto vec3 = CreateVec3Msg(builder, msg.vec3.x, msg.vec3.y, msg.vec3.z);
    
    // Vec4
    auto vec4 = CreateVec4Msg(builder, msg.vec4.x, msg.vec4.y, msg.vec4.z, msg.vec4.w);

    // Vec3 Array
    std::vector<flatbuffers::Offset<Vec3Msg>> vec3_vec;
    for (const auto& v : msg.vec3_arr) {
        vec3_vec.push_back(CreateVec3Msg(builder, v.x, v.y, v.z));
    }
    auto vec3_arr = builder.CreateVector(vec3_vec);

    // Vec4 Array
    std::vector<flatbuffers::Offset<Vec4Msg>> vec4_vec;
    for (const auto& v : msg.vec4_arr) {
        vec4_vec.push_back(CreateVec4Msg(builder, v.x, v.y, v.z, v.w));
    }
    auto vec4_arr = builder.CreateVector(vec4_vec);

    CommandMsgBuilder cmd_builder(builder);
    cmd_builder.add_name(name);
    cmd_builder.add_timestamp(msg.timestamp);
    cmd_builder.add_cmd_id(msg.cmd_id);
    cmd_builder.add_sys_id(msg.sys_id);
    cmd_builder.add_int_val(msg.int_val);
    cmd_builder.add_float_val(msg.float_val);
    cmd_builder.add_int_arr(int_arr);
    cmd_builder.add_float_arr(float_arr);
    cmd_builder.add_vec3(vec3);
    cmd_builder.add_vec4(vec4);
    cmd_builder.add_vec3_arr(vec3_arr);
    cmd_builder.add_vec4_arr(vec4_arr);

    auto cmd_msg = cmd_builder.Finish();
    builder.Finish(cmd_msg, CommandMsgIdentifier());

    uint8_t* buf = builder.GetBufferPointer();
    int size = builder.GetSize();

    return std::vector<uint8_t>(buf, buf + size);
  }
};
