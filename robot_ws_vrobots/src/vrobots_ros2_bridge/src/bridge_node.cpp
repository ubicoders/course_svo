#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <optional>
#include "vrobots_ros2_bridge/states_converter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "vrobots_ros2_msg/msg/states.hpp"
#include "zenoh.hxx"

using namespace std::chrono_literals;
using namespace zenoh;

class VRobotsRos2Bridge : public rclcpp::Node {
public:
  VRobotsRos2Bridge() 
      : Node("vrobots_ros2_bridge"),
        session_(Session::open(Config::create_default())) {
    // Declare and get sys_id parameter
    this->declare_parameter("sys_id", 0);
    int sys_id = this->get_parameter("sys_id").as_int();

    std::string ros_topic = "/ros2/vr" + std::to_string(sys_id) + "/states";
    std::string zenoh_key = "vr/" + std::to_string(sys_id) + "/states";

    // ROS 2 Publisher
    publisher_ = this->create_publisher<vrobots_ros2_msg::msg::States>(
        ros_topic, 10);

    // Zenoh Subscriber
    subscriber_.emplace(session_.declare_subscriber(
        KeyExpr(zenoh_key),
        [this](const Sample& sample) {
          const auto& payload = sample.get_payload();
          auto data = payload.as_vector();

          auto msg_opt = StatesConverter::convert(data);
          if (msg_opt) {
            this->publisher_->publish(*msg_opt);
          }
        },
        closures::none
    ));
    
    RCLCPP_INFO(this->get_logger(), "Zenoh bridge started. Subscribing to %s, Publishing to %s", zenoh_key.c_str(), ros_topic.c_str());
  }

private:
  rclcpp::Publisher<vrobots_ros2_msg::msg::States>::SharedPtr publisher_;
  Session session_;
  std::optional<Subscriber<void>> subscriber_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VRobotsRos2Bridge>());
  rclcpp::shutdown();
  return 0;
}
