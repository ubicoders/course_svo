#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vrobots_ros2_msg/msg/states.hpp"
#include "vrobots_ros2_msg/msg/commands.hpp"
#include "vrobots_ros2_msg/msg/vec3.hpp"

#include "vrobots_ros2_controller/command_helper.hpp"

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode() : Node("controller_node") {
    // Declare and get sys_id parameter
    this->declare_parameter("sys_id", 0);
    sys_id_ = this->get_parameter("sys_id").as_int();

    std::string left_topic = "/ros2/vr" + std::to_string(sys_id_) + "/cams/left/p720";
    std::string right_topic = "/ros2/vr" + std::to_string(sys_id_) + "/cams/right/p720";
    std::string states_topic = "/ros2/vr" + std::to_string(sys_id_) + "/states";
    std::string cmd_topic = "/ros2/vr" + std::to_string(sys_id_) + "/cmd";

    // Publisher
    cmd_pub_ = this->create_publisher<vrobots_ros2_msg::msg::Commands>(cmd_topic, 10);

    // Subscriptions
    left_cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        left_topic, 10,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "Received Left Image: %dx%d", msg->width, msg->height);
        });

    right_cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        right_topic, 10,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "Received Right Image: %dx%d", msg->width, msg->height);
        });

    states_sub_ = this->create_subscription<vrobots_ros2_msg::msg::States>(
        states_topic, 10,
        [this](const vrobots_ros2_msg::msg::States::SharedPtr msg) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Received States: ts=%.2f, pos=(%.2f, %.2f, %.2f)", 
                msg->timestamp, msg->lin_pos.x, msg->lin_pos.y, msg->lin_pos.z);
            float alt = -msg->lin_pos.z;
            if (alt < 15){
                auto msg = vrobots_ros2_controller::CommandHelper::build_cmd_multirotor(
                sys_id_, this->now().seconds() * 1000.0, {1501, 1501, 1501, 1501});
                cmd_pub_->publish(msg);
            }else {
                auto msg = vrobots_ros2_controller::CommandHelper::build_cmd_multirotor(
                sys_id_, this->now().seconds() * 1000.0, {1499, 1499, 1499, 1499});
                cmd_pub_->publish(msg);
            }

        });
    
    // Example timer to send commands (optional, currently sending dummy PWM)
    timer_ = this->create_wall_timer(20ms, [this]() {
        // Example: Send 1600 PWM to all motors
        // auto msg = vrobots_ros2_controller::CommandHelper::build_cmd_multirotor(
        //   sys_id_, this->now().seconds() * 1000.0, {1501, 1501, 1501, 1501});
        // cmd_pub_->publish(msg);
    });

    RCLCPP_INFO(this->get_logger(), "Controller started. Listening to system %d", sys_id_);
  }

private:
  int sys_id_;
  rclcpp::Publisher<vrobots_ros2_msg::msg::Commands>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_cam_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_cam_sub_;
  rclcpp::Subscription<vrobots_ros2_msg::msg::States>::SharedPtr states_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
