#ifndef VROBOTS_ROS2_CONTROLLER_COMMAND_HELPER_HPP_
#define VROBOTS_ROS2_CONTROLLER_COMMAND_HELPER_HPP_

#include <vector>
#include <cstdint>

#include "vrobots_ros2_msg/msg/commands.hpp"
#include "vrobots_ros2_msg/msg/vec3.hpp"

namespace vrobots_ros2_controller {

struct VROBOTS_CMDS {
    // linear motion
    static const uint32_t SET_ACC = 1;
    static const uint32_t SET_VEL = 2;
    static const uint32_t SET_POS = 3;

    // angular motion
    static const uint32_t SET_ANGACC = 50;
    static const uint32_t SET_ANGVEL = 51;
    static const uint32_t SET_EULER = 52;
    static const uint32_t SET_EULER_DOT = 53;
    static const uint32_t SET_QUAT = 54;

    // mass
    static const uint32_t SET_MASS = 100;
    static const uint32_t SET_MOI_3X1 = 101;
    static const uint32_t SET_MOI_3X3 = 102;

    // set body forces and torques
    static const uint32_t SET_BODY_FORCE = 200;
    static const uint32_t SET_BODY_TORQUE = 201;
    static const uint32_t SET_BODY_FT = 202;
    static const uint32_t ADD_BODY_FORCE = 203;
    static const uint32_t ADD_BODY_TORQUE = 204;
    static const uint32_t ADD_BODY_FT = 205;

    // set acutators
    static const uint32_t SET_PWM = 300;
    static const uint32_t SET_MR_THROTTLE = 301;
    static const uint32_t SET_OMROVER = 302;
    static const uint32_t SET_HELI = 303;
    static const uint32_t SET_CAR = 304;
    static const uint32_t SET_MSD = 305;
    static const uint32_t SET_INVPEN = 306;
};

class CommandHelper {
public:
    static vrobots_ros2_msg::msg::Commands build_cmd_set_force_torque_body(
        int sys_id, double timestamp, float fx, float fy, float fz, float tx, float ty, float tz) 
    {
        auto msg = vrobots_ros2_msg::msg::Commands();
        msg.timestamp = timestamp;
        msg.cmd_id = VROBOTS_CMDS::SET_BODY_FT;
        msg.sys_id = sys_id;

        vrobots_ros2_msg::msg::Vec3 force;
        force.x = fx; force.y = fy; force.z = fz;

        vrobots_ros2_msg::msg::Vec3 torque;
        torque.x = tx; torque.y = ty; torque.z = tz;

        msg.vec3_arr.push_back(force);
        msg.vec3_arr.push_back(torque);
        return msg;
    }

    static vrobots_ros2_msg::msg::Commands build_cmd_msd(int sys_id, double timestamp, float pos) {
        auto msg = vrobots_ros2_msg::msg::Commands();
        msg.timestamp = timestamp;
        msg.cmd_id = VROBOTS_CMDS::SET_MSD;
        msg.sys_id = sys_id;
        msg.float_val = pos;
        return msg;
    }

    static vrobots_ros2_msg::msg::Commands build_cmd_invpen(
        int sys_id, double timestamp, float pos, float vel, float ang, float angvel) 
    {
        auto msg = vrobots_ros2_msg::msg::Commands();
        msg.timestamp = timestamp;
        msg.cmd_id = VROBOTS_CMDS::SET_INVPEN;
        msg.sys_id = sys_id;
        msg.float_arr = {pos, vel, ang, angvel};
        return msg;
    }

    static vrobots_ros2_msg::msg::Commands build_cmd_heli(int sys_id, double timestamp, float force) {
        auto msg = vrobots_ros2_msg::msg::Commands();
        msg.timestamp = timestamp;
        msg.cmd_id = VROBOTS_CMDS::SET_HELI;
        msg.sys_id = sys_id;
        msg.float_val = force;
        return msg;
    }

    static vrobots_ros2_msg::msg::Commands build_cmd_multirotor(
        int sys_id, double timestamp, const std::vector<int>& pwm) 
    {
        auto msg = vrobots_ros2_msg::msg::Commands();
        msg.timestamp = timestamp;
        msg.cmd_id = VROBOTS_CMDS::SET_PWM;
        msg.sys_id = sys_id;
        msg.int_arr = pwm;
        return msg;
    }

    static vrobots_ros2_msg::msg::Commands build_cmd_omrover(
        int sys_id, double timestamp, const std::vector<float>& actuators) 
    {
        auto msg = vrobots_ros2_msg::msg::Commands();
        msg.timestamp = timestamp;
        msg.cmd_id = VROBOTS_CMDS::SET_OMROVER;
        msg.sys_id = sys_id;
        msg.float_arr = actuators;
        return msg;
    }

    static vrobots_ros2_msg::msg::Commands build_cmd_car(
        int sys_id, double timestamp, float torque, float brake, float steer) 
    {
        auto msg = vrobots_ros2_msg::msg::Commands();
        msg.timestamp = timestamp;
        msg.cmd_id = VROBOTS_CMDS::SET_CAR;
        msg.sys_id = sys_id;
        msg.float_arr = {torque, brake, steer};
        return msg;
    }
};

} // namespace vrobots_ros2_controller

#endif // VROBOTS_ROS2_CONTROLLER_COMMAND_HELPER_HPP_
