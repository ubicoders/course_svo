#include "svo/svo_node.hpp"

namespace UbiSVO {

StereoVisualOdometry::StereoVisualOdometry(const rclcpp::NodeOptions &options)
    : Node(ROS2NodeConfig().node_name, options) {

  // Declare and get sys_id parameter
  this->declare_parameter<int>("sys_id", 1);
  int sys_id = this->get_parameter("sys_id").as_int();

  // Convert sys_id to system_id string format (e.g., 1 -> "vr1")
  std::string system_id = "vr" + std::to_string(sys_id);

  // Set topics based on system_id
  ros2NodeConfig.setTopicsFromSystemId(system_id);

  RCLCPP_INFO(this->get_logger(), "Using sys_id: %d (system_id: %s)", sys_id, system_id.c_str());
  RCLCPP_INFO(this->get_logger(), "Left topic: %s", ros2NodeConfig.input_img_left_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Right topic: %s", ros2NodeConfig.input_img_right_topic.c_str());

  // subscriber
  leftImageSub_ = std::make_shared<message_filters::Subscriber<Image>>(
      this, ros2NodeConfig.input_img_left_topic);
  rightImageSub_ = std::make_shared<message_filters::Subscriber<Image>>(
      this, ros2NodeConfig.input_img_right_topic);
  syncStereo_ =
      std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(
          ImageSyncPolicy(10), *leftImageSub_, *rightImageSub_);
  syncStereo_->registerCallback(std::bind(&StereoVisualOdometry::ImageCallback,
                                          this, std::placeholders::_1,
                                          std::placeholders::_2));

  // publisher
  debugImagePub_ = this->create_publisher<sensor_msgs::msg::Image>(
      ros2NodeConfig.output_img_debug_topic, 50);
  pointCloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      ros2NodeConfig.output_pc_topic, 10);
  pathPub_ = this->create_publisher<nav_msgs::msg::Path>(
      ros2NodeConfig.output_path_topic, 10);
  odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      ros2NodeConfig.output_odom_topic, 10);
  markerPub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      ros2NodeConfig.output_camera_frustum_topic, 10);

  double frequency = ros2NodeConfig.msg_pub_rate;
  auto interval = std::chrono::duration<double>(1.0 / frequency);
  timer_ =
      this->create_wall_timer(interval, [this]() -> void { viewer->update(); });

  // SVO components
  map = std::make_shared<Map>();
  stereoCam = std::make_shared<StereoGeometry>();
  backend = std::make_shared<Backend>(stereoCam, map);
  viewer = std::make_shared<Viewer>(map, stereoCam, this->get_clock(),
                                    debugImagePub_, pointCloudPub_, pathPub_,
                                    odomPub_, markerPub_);
  frontend = std::make_shared<Frontend>(stereoCam, map, backend, viewer);
}

void StereoVisualOdometry::ImageCallback(
    const Image::ConstSharedPtr &leftImage,
    const Image::ConstSharedPtr &rightImage) {
  // for each callback, create a frame
  auto frame = std::make_shared<Frame>();

  // convert to cv::Mat
  auto CVImageL = cv_bridge::toCvCopy(leftImage, leftImage->encoding)->image;
  auto CVImageR = cv_bridge::toCvCopy(rightImage, rightImage->encoding)->image;

  // convert to grayscale
  cv::cvtColor(CVImageL, frame->imageL, cv::COLOR_BGR2GRAY);
  cv::cvtColor(CVImageR, frame->imageR, cv::COLOR_BGR2GRAY);

  // call frontend and viewer
  frontend->step(frame);
  viewer->debugImageUpdate(frame);
}
} // namespace UbiSVO