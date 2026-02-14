#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>
#include <rerun/archetypes/image.hpp>
#include <rerun/archetypes/line_strips3d.hpp>
#include <rerun/archetypes/pinhole.hpp>
#include <rerun/archetypes/points3d.hpp>
#include <rerun/archetypes/scalars.hpp>
#include <rerun/archetypes/transform3d.hpp>
#include <rerun/components/radius.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>

class SvoRerunNode : public rclcpp::Node {
public:
  SvoRerunNode() : Node("svo_rerun_node") {
    // Initialize Rerun
    // "StereoVO" application name
    // Use connect() instead of spawn() to avoid display issues in headless
    // environments The user must run `rerun` separately.
    rec_stream_.connect_grpc().exit_on_failure();

    setup_blueprint();

    // Subscribers
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/ubicoders/debug_image", 10,
        std::bind(&SvoRerunNode::image_callback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/ubicoders/path", 10,
        std::bind(&SvoRerunNode::path_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ubicoders/odom", 10,
        std::bind(&SvoRerunNode::odom_callback, this, std::placeholders::_1));

    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ubicoders/point_cloud", 10,
        std::bind(&SvoRerunNode::pc_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SVO Rerun Node Initialized");
  }

private:
  rerun::RecordingStream rec_stream_ = rerun::RecordingStream("StereoVO");

  void setup_blueprint() {
    // Log static camera intrinsics (frustum)
    rec_stream_.log(
        "world/camera",
        rerun::archetypes::Pinhole::from_focal_length_and_resolution(
            {1623.86f, 1623.86f}, {1280.0f, 720.0f})
            .with_image_plane_distance(0.5f));
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      // Convert to OpenCV
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // Convert to RGB for Rerun
      cv::Mat rgb_image;
      cv::cvtColor(cv_ptr->image, rgb_image, cv::COLOR_BGR2RGB);

      // Log image
      // Rerun C++ expects raw data pointer or tensor
      // We need to pass dimensions and data
      std::vector<uint8_t> data(rgb_image.data,
                                rgb_image.data +
                                    (rgb_image.total() * rgb_image.elemSize()));

      rec_stream_.log("debug_image",
                      rerun::archetypes::Image::from_rgb24(
                          data, {static_cast<uint32_t>(msg->width),
                                 static_cast<uint32_t>(msg->height)}));

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
      return;
    }

    // Get latest pose
    auto latest_pose = msg->poses.back().pose;
    float x = latest_pose.position.x;
    float y = latest_pose.position.y;
    float z = latest_pose.position.z;
    RCLCPP_INFO(this->get_logger(), "Latest pose: x=%f, y=%f, z=%f", x, y, z);

    // Log position scalars (from Path - Keyframe sparse)
    rec_stream_.log("plot/x_keyframe",
                    rerun::archetypes::Scalars({static_cast<double>(x)}));
    rec_stream_.log("plot/y_keyframe",
                    rerun::archetypes::Scalars({static_cast<double>(y)}));
    rec_stream_.log("plot/z_keyframe",
                    rerun::archetypes::Scalars({static_cast<double>(z)}));

    // Camera pose is now handled by odom_callback for smooth tracking
    // We only log the trajectory line strip here

    // Log trajectory

    // Log trajectory
    // Rerun LineStrips3D
    std::vector<rerun::datatypes::Vec3D> points;
    points.reserve(msg->poses.size());
    for (const auto &p : msg->poses) {
      points.push_back({static_cast<float>(p.pose.position.x),
                        static_cast<float>(p.pose.position.y),
                        static_cast<float>(p.pose.position.z)});
    }

    // Log trajectory
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Logging path with %zu points", points.size());

    rec_stream_.log(
        "world/trajectory",
        rerun::archetypes::LineStrips3D(rerun::components::LineStrip3D(points))
            .with_colors(rerun::datatypes::Rgba32(0, 255, 0, 255))
            .with_radii(rerun::components::Radius::ui_points(2.0f)));
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    float z = msg->pose.pose.position.z;

    // Log camera pose from Odometry (Smooth, 30Hz)
    rec_stream_.log("world/camera",
                    rerun::archetypes::Transform3D(
                        rerun::datatypes::Vec3D{x, y, z},
                        rerun::datatypes::Quaternion{
                            static_cast<float>(msg->pose.pose.orientation.x),
                            static_cast<float>(msg->pose.pose.orientation.y),
                            static_cast<float>(msg->pose.pose.orientation.z),
                            static_cast<float>(msg->pose.pose.orientation.w)}));

    // Log live position scalars (Smooth)
    rec_stream_.log("plot/x",
                    rerun::archetypes::Scalars({static_cast<double>(x)}));
    rec_stream_.log("plot/y",
                    rerun::archetypes::Scalars({static_cast<double>(y)}));
    rec_stream_.log("plot/z",
                    rerun::archetypes::Scalars({static_cast<double>(z)}));

    // Log live trajectory (Blue)
    live_traj_points_.push_back({x, y, z});
    rec_stream_.log(
        "world/live_trajectory",
        rerun::archetypes::LineStrips3D(
            rerun::components::LineStrip3D(live_traj_points_))
            .with_colors(rerun::datatypes::Rgba32(0, 0, 255, 255))
            .with_radii(rerun::components::Radius::ui_points(5.0f)));
  }

  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Parse PointCloud2
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    // Colors often stored as RGB float or bytes. Usually "rgb" field.
    bool has_rgb = false;
    for (const auto &field : msg->fields) {
      if (field.name == "rgb") {
        has_rgb = true;
        break;
      }
    }

    std::vector<rerun::datatypes::Vec3D> points;
    std::vector<rerun::datatypes::Rgba32> colors;

    size_t n_points = msg->width * msg->height;
    points.reserve(n_points);
    if (has_rgb)
      colors.reserve(n_points);

    if (has_rgb) {
      sensor_msgs::PointCloud2ConstIterator<float> iter_rgb(*msg, "rgb");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
          continue;
        }
        points.push_back({*iter_x, *iter_y, *iter_z});

        // Decode RGB
        // float to bytes
        float rgb_val = *iter_rgb;
        // memcpy
        uint32_t rgb_int;
        std::memcpy(&rgb_int, &rgb_val, sizeof(uint32_t));
        // 0x00RRGGBB usually
        uint8_t r = (rgb_int >> 16) & 0xFF;
        uint8_t g = (rgb_int >> 8) & 0xFF;
        uint8_t b = (rgb_int) & 0xFF;

        colors.push_back(rerun::datatypes::Rgba32(r, g, b, 255));
      }
    } else {
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
          continue;
        }
        points.push_back({*iter_x, *iter_y, *iter_z});
        colors.push_back(rerun::datatypes::Rgba32(255, 255, 255, 255));
      }
    }

    rec_stream_.log(
        "world/map_points",
        rerun::archetypes::Points3D(points).with_colors(colors).with_radii(
            {0.1f}));
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;

  std::vector<rerun::datatypes::Vec3D> live_traj_points_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SvoRerunNode>());
  rclcpp::shutdown();
  return 0;
}
