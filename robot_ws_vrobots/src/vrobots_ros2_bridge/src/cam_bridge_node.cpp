#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "iox2/iceoryx2.hpp"
#include "vrobots_ros2_bridge/message_data.hpp"

using namespace std::chrono_literals;

class CameraStream {
public:
    CameraStream(rclcpp::Node* node, 
                 const std::string& iceoryx_service_name, 
                 const std::string& ros_topic_name) 
        : node_(node) {
        
        // ROS 2 Publisher
        publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(ros_topic_name, 10);

        // Initialize Iceoryx2
        using namespace iox2;
        
        auto node_result = NodeBuilder().create<ServiceType::Ipc>();
        if (node_result.has_error()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create Iceoryx2 node for %s", iceoryx_service_name.c_str());
            return;
        }
        iox_node_ = std::make_unique<iox2::Node<ServiceType::Ipc>>(std::move(node_result.value()));

        auto service_result = iox_node_->service_builder(ServiceName::create(iceoryx_service_name.c_str()).expect("valid service name"))
                        .publish_subscribe<ImageData720p>()
                        .user_header<GenericHeader>()
                        .open_or_create();

        if (service_result.has_error()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open/create Iceoryx2 service for %s", iceoryx_service_name.c_str());
            return;
        }
        auto service = std::move(service_result.value());

        auto subscriber_result = service.subscriber_builder().create();
        if (subscriber_result.has_error()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create Iceoryx2 subscriber for %s", iceoryx_service_name.c_str());
            return;
        }
        subscriber_ = std::make_unique<Subscriber<ServiceType::Ipc, ImageData720p, GenericHeader>>(std::move(subscriber_result.value()));
        
        RCLCPP_INFO(node_->get_logger(), "Subscribed to Iceoryx2 service: %s", iceoryx_service_name.c_str());
    }

    void process() {
        if (!subscriber_) return;

        auto recv_result = subscriber_->receive();
        if (recv_result.has_error()) {
            // Only warn occasionally or if critical
            return;
        }

        auto sample_opt = std::move(recv_result.value());
        while (sample_opt.has_value()) {
            const auto& payload = sample_opt->payload();
            
            // Create ROS 2 message
            auto msg = sensor_msgs::msg::Image();
            msg.header.stamp = node_->now();
            msg.header.frame_id = "vrobots"; // Placeholder frame ID
            msg.height = 720;
            msg.width = 1280;
            msg.encoding = "rgba8";
            msg.is_bigendian = 0;
            msg.step = 1280 * 4;
            
            // Copy data with vertical flip
            msg.data.resize(payload.image_data.size());
            const size_t row_stride = 1280 * 4;
            const size_t height = 720;
            
            for (size_t y = 0; y < height; ++y) {
                const auto* src_row = payload.image_data.data() + y * row_stride;
                auto* dst_row = msg.data.data() + (height - 1 - y) * row_stride;
                std::memcpy(dst_row, src_row, row_stride);
            }

            publisher_->publish(msg);

            // Get next sample
            recv_result = subscriber_->receive();
            if (recv_result.has_error()) break;
            sample_opt = std::move(recv_result.value());
        }
    }

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    std::unique_ptr<iox2::Node<iox2::ServiceType::Ipc>> iox_node_;
    std::unique_ptr<iox2::Subscriber<iox2::ServiceType::Ipc, ImageData720p, GenericHeader>> subscriber_;
};

class CamBridgeNode : public rclcpp::Node {
public:
  CamBridgeNode() : rclcpp::Node("cam_bridge_node") {
    using namespace iox2;
    set_log_level_from_env_or(LogLevel::Info);

    // Declare and get sys_id parameter
    this->declare_parameter("sys_id", 0);
    int sys_id = this->get_parameter("sys_id").as_int();

    std::string left_service = "vr/" + std::to_string(sys_id) + "/cams/left/720p";
    std::string left_topic = "/ros2/vr" + std::to_string(sys_id) + "/cams/left/p720";
    
    std::string right_service = "vr/" + std::to_string(sys_id) + "/cams/right/720p";
    std::string right_topic = "/ros2/vr" + std::to_string(sys_id) + "/cams/right/p720";

    left_cam_ = std::make_unique<CameraStream>(this, left_service, left_topic);
    right_cam_ = std::make_unique<CameraStream>(this, right_service, right_topic);

    // Timer to poll Iceoryx2
    timer_ = this->create_wall_timer(
        10ms, std::bind(&CamBridgeNode::process_messages, this)); // 100Hz polling
  }

private:
  void process_messages() {
      if (left_cam_) left_cam_->process();
      if (right_cam_) right_cam_->process();
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<CameraStream> left_cam_;
  std::unique_ptr<CameraStream> right_cam_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CamBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
