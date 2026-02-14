#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace UbiSVO {

struct CameraConfig {
  double fx = 1623.86;
  double fy = 1623.86;
  double cx = 640;
  double cy = 360;
  double baseline = 0.2;
};

struct ROS2NodeConfig {
  std::string node_name = "stereo_visual_odometry";
  std::string system_id = "vr0"; // Default system ID

  // Topic names (will be set dynamically based on system_id)
  std::string input_img_left_topic;
  std::string input_img_right_topic;
  std::string output_img_debug_topic = "/ubicoders/debug_image";
  std::string output_pc_topic = "/ubicoders/point_cloud";
  std::string output_path_topic =
      "/ubicoders/path"; // Keyframe Trajectory (Sparse)
  std::string output_odom_topic = "/ubicoders/odom"; // Live Camera Pose (30Hz)
  std::string output_camera_frustum_topic =
      "/ubicoders/camera_frustum"; // Markers: Frustum (Keyframe) + Tracked Path
                                   // (Live History)
  std::string output_frame_name = "ubicoders_svo";

  double msg_pub_rate = 30.0;

  // Helper function to set topics based on system ID
  void setTopicsFromSystemId(const std::string& id) {
    system_id = id;
    input_img_left_topic = "/ros2/" + system_id + "/cams/left/p720";
    input_img_right_topic = "/ros2/" + system_id + "/cams/right/p720";
  }
};

} // namespace UbiSVO

#endif // __CONFIG_H__
