#include "vrobots_ros2_bridge/states_converter.hpp"
#include "vrobots_ros2_bridge/R000_states/R000_states_generated.h"

std::optional<vrobots_ros2_msg::msg::States> StatesConverter::convert(const std::vector<uint8_t>& data) {
    if (!StatesMsgBufferHasIdentifier(data.data())) {
        return std::nullopt;
    }

    auto fb_states = GetStatesMsg(data.data());
    auto msg = vrobots_ros2_msg::msg::States();

    // Helper to copy Vec3
    auto copy_vec3 = [](const Vec3Msg* src, vrobots_ros2_msg::msg::Vec3& dst) {
      if (src) {
        dst.x = src->x();
        dst.y = src->y();
        dst.z = src->z();
      }
    };

    // Helper to copy Vec4
    auto copy_vec4 = [](const Vec4Msg* src, vrobots_ros2_msg::msg::Vec4& dst) {
      if (src) {
        dst.x = src->x();
        dst.y = src->y();
        dst.z = src->z();
        dst.w = src->w();
      }
    };

    if (fb_states->name()) msg.name = fb_states->name()->str();
    msg.sys_id = fb_states->sys_id();
    msg.timestamp = fb_states->timestamp();

    copy_vec3(fb_states->lin_acc(), msg.lin_acc);
    copy_vec3(fb_states->lin_vel(), msg.lin_vel);
    copy_vec3(fb_states->lin_pos(), msg.lin_pos);
    msg.altitude = fb_states->altitude();

    copy_vec3(fb_states->ang_acc(), msg.ang_acc);
    copy_vec3(fb_states->ang_vel(), msg.ang_vel);
    copy_vec3(fb_states->euler(), msg.euler);
    copy_vec3(fb_states->euler_dot(), msg.euler_dot);
    copy_vec4(fb_states->quaternion(), msg.quaternion);

    if (fb_states->pwm()) msg.pwm.assign(fb_states->pwm()->begin(), fb_states->pwm()->end());
    if (fb_states->actuators()) msg.actuators.assign(fb_states->actuators()->begin(), fb_states->actuators()->end());
    
    copy_vec3(fb_states->force(), msg.force);
    copy_vec3(fb_states->torque(), msg.torque);

    copy_vec3(fb_states->accelerometer(), msg.accelerometer);
    copy_vec3(fb_states->gyroscope(), msg.gyroscope);
    copy_vec3(fb_states->magnetometer(), msg.magnetometer);
    msg.barometer = fb_states->barometer();
    msg.temperature = fb_states->temperature();
    copy_vec3(fb_states->gps_pos(), msg.gps_pos);
    copy_vec3(fb_states->gps_vel(), msg.gps_vel);

    msg.mass = fb_states->mass();
    copy_vec3(fb_states->cg(), msg.cg);
    if (fb_states->moment_arms()) msg.moment_arms.assign(fb_states->moment_arms()->begin(), fb_states->moment_arms()->end());
    copy_vec3(fb_states->moi_3x1(), msg.moi_3x1);
    if (fb_states->moi_3x3()) msg.moi_3x3.assign(fb_states->moi_3x3()->begin(), fb_states->moi_3x3()->end());
    if (fb_states->extra_props()) msg.extra_props.assign(fb_states->extra_props()->begin(), fb_states->extra_props()->end());

    // Collisions
    if (fb_states->collisions()) {
      for (const auto* col : *fb_states->collisions()) {
        vrobots_ros2_msg::msg::Collision col_msg;
        col_msg.timestamp = col->timestamp();
        col_msg.collision_type = col->collision_type();
        if (col->object_name()) col_msg.object_name = col->object_name()->str();
        copy_vec3(col->pos(), col_msg.pos);
        copy_vec3(col->vel(), col_msg.vel);
        copy_vec3(col->eul(), col_msg.eul);
        copy_vec3(col->angvel(), col_msg.angvel);
        msg.collisions.push_back(col_msg);
      }
    }

    if (fb_states->image_data_0()) msg.image_data_0.assign(fb_states->image_data_0()->begin(), fb_states->image_data_0()->end());
    if (fb_states->image_data_1()) msg.image_data_1.assign(fb_states->image_data_1()->begin(), fb_states->image_data_1()->end());

    return msg;
}
