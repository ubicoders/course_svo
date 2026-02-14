#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace UbiSVO {

struct CameraConfig {
  double fx = 718.8560;
  double fy = 718.8560;
  double cx = 607.1928;
  double cy = 185.2157;
  double baseline = 0.537;

  // Factory method to create config from dataset preset
  static CameraConfig fromPreset(const std::string& dataset) {
    CameraConfig config;

    if (dataset == "kitti") {
      // KITTI
      config.fx = 718.8560;
      config.fy = 718.8560;
      config.cx = 607.1928;
      config.cy = 185.2157;
      config.baseline = 0.537;
    } else if (dataset == "euroc") {
      // EuroC (RECTIFIED)
      config.fx = 436.23459;
      config.fy = 436.23459;
      config.cx = 364.44123;
      config.cy = 256.95168;
      config.baseline = 0.11008;
    } else if (dataset == "custom") {
      // Custom VO
      config.fx = 1797.045;
      config.fy = 1797.045;
      config.cx = 720.013;
      config.cy = 540.010;
      config.baseline = 0.41332;
    } else {
      // Default to KITTI
      config.fx = 718.8560;
      config.fy = 718.8560;
      config.cx = 607.1928;
      config.cy = 185.2157;
      config.baseline = 0.537;
    }

    return config;
  }
};

struct ROS2NodeConfig {
  std::string node_name = "stereo_visual_odometry";
  std::string input_img_left_topic = "/stereo/image_left";
  std::string input_img_right_topic = "/stereo/image_right";
  std::string output_img_debug_topic = "/ubicoders/debug_image";
  std::string output_pc_topic = "/ubicoders/point_cloud";
  std::string output_path_topic = "/ubicoders/path";
  std::string output_marker_topic = "/ubicoders/camera_frustum";
  std::string output_frame_name = "ubicoders_svo";
  double msg_pub_rate = 20.0;
};

} // namespace UbiSVO

#endif // __CONFIG_H__
