#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vrobots_ros2_msg/msg/commands.hpp"
#include "vrobots_ros2_bridge/commands_converter.hpp"
#include "zenoh.hxx"

using namespace std::chrono_literals;
using namespace zenoh;

class CmdBridgeNode : public rclcpp::Node {
public:
  CmdBridgeNode() 
      : Node("cmd_bridge_node"),
        session_(Session::open(Config::create_default())) {
    
    // Declare and get sys_id parameter
    this->declare_parameter("sys_id", 0);
    int sys_id = this->get_parameter("sys_id").as_int();

    std::string ros_topic = "/ros2/vr" + std::to_string(sys_id) + "/cmd";
    std::string zenoh_key = "vr/" + std::to_string(sys_id) + "/cmd";

    // Zenoh Publisher
    zenoh_publisher_.emplace(session_.declare_publisher(KeyExpr(zenoh_key)));

    // ROS 2 Subscriber
    subscription_ = this->create_subscription<vrobots_ros2_msg::msg::Commands>(
        ros_topic, 10,
        [this](const vrobots_ros2_msg::msg::Commands::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received command: %s (id: %d)", msg->name.c_str(), msg->cmd_id);
            auto data = CommandsConverter::convert(*msg);
            if (zenoh_publisher_) {
                try {
                    zenoh_publisher_->put(data);
                    // RCLCPP_INFO(this->get_logger(), "Published to Zenoh");
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to put data to Zenoh: %s", e.what());
                }
            }
        });

    RCLCPP_INFO(this->get_logger(), "Command Bridge started. Subscribing to %s, Publishing to %s", ros_topic.c_str(), zenoh_key.c_str());
  }

private:
  Session session_;
  std::optional<Publisher> zenoh_publisher_;
  rclcpp::Subscription<vrobots_ros2_msg::msg::Commands>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
