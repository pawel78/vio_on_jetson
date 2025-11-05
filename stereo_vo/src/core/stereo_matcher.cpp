/**
 * @file stereo_matcher.cpp
 * @brief Implementation of StereoMatcher
 */

#include "core/stereo_matcher.hpp"
#include <iostream>

namespace stereo_vo {
namespace core {

StereoMatcher::StereoMatcher(const Config& config, const CameraParams& params)
    : config_(config), params_(params) {
  initializeMatcher();
}

void StereoMatcher::initializeMatcher() {
  if (config_.method == "sgbm" || config_.method == "cuda_sgbm") {
    // Create SGBM matcher
    int mode = config_.use_mode_HH ? cv::StereoSGBM::MODE_HH : cv::StereoSGBM::MODE_SGBM;
    
    sgbm_ = cv::StereoSGBM::create(
        config_.min_disparity,
        config_.num_disparities,
        config_.block_size,
        config_.p1,
        config_.p2,
        config_.disp12_max_diff,
        config_.pre_filter_cap,
        config_.uniqueness_ratio,
        config_.speckle_window_size,
        config_.speckle_range,
        mode
    );
    
#ifdef USE_CUDA
    if (config_.method == "cuda_sgbm") {
      // Initialize CUDA stereo matcher if available
      try {
        cuda_sgbm_ = cv::cuda::createStereoSGM(
            config_.min_disparity,
            config_.num_disparities,
            config_.p1,
            config_.p2
        );
        use_cuda_ = true;
        std::cout << "CUDA SGBM initialized successfully" << std::endl;
      } catch (const cv::Exception& e) {
        std::cerr << "Failed to initialize CUDA SGBM: " << e.what() << std::endl;
        use_cuda_ = false;
      }
    }
#endif
  }
}

cv::Mat StereoMatcher::computeDisparity(
    const cv::Mat& left_image,
    const cv::Mat& right_image) {
  
  cv::Mat disparity;
  
  if (left_image.empty() || right_image.empty()) {
    return disparity;
  }
  
  // Ensure images are grayscale
  cv::Mat left_gray = left_image;
  cv::Mat right_gray = right_image;
  
  if (left_image.channels() == 3) {
    cv::cvtColor(left_image, left_gray, cv::COLOR_BGR2GRAY);
  }
  if (right_image.channels() == 3) {
    cv::cvtColor(right_image, right_gray, cv::COLOR_BGR2GRAY);
  }
  
#ifdef USE_CUDA
  if (use_cuda_ && cuda_sgbm_) {
    cv::cuda::GpuMat d_left, d_right, d_disparity;
    d_left.upload(left_gray);
    d_right.upload(right_gray);
    
    cuda_sgbm_->compute(d_left, d_right, d_disparity);
    d_disparity.download(disparity);
  } else
#endif
  {
    sgbm_->compute(left_gray, right_gray, disparity);
  }
  
  return disparity;
}

Eigen::Vector3d StereoMatcher::triangulate(
    const cv::Point2f& left_point,
    float disparity) const {
  
  if (!isValidDisparity(disparity)) {
    return Eigen::Vector3d(0, 0, -1);  // Invalid depth
  }
  
  // Compute depth from disparity
  double depth = (params_.K_left(0, 0) * params_.baseline) / disparity;
  
  // Back-project to 3D
  double x = (left_point.x - params_.K_left(0, 2)) * depth / params_.K_left(0, 0);
  double y = (left_point.y - params_.K_left(1, 2)) * depth / params_.K_left(1, 1);
  double z = depth;
  
  return Eigen::Vector3d(x, y, z);
}

std::vector<Eigen::Vector3d> StereoMatcher::triangulatePoints(
    const std::vector<cv::Point2f>& left_points,
    const cv::Mat& disparity_map) const {
  
  std::vector<Eigen::Vector3d> points_3d;
  points_3d.reserve(left_points.size());
  
  if (disparity_map.empty()) {
    return points_3d;
  }
  
  for (const auto& pt : left_points) {
    int x = static_cast<int>(pt.x);
    int y = static_cast<int>(pt.y);
    
    // Check bounds
    if (x >= 0 && x < disparity_map.cols && y >= 0 && y < disparity_map.rows) {
      float disp = disparity_map.at<short>(y, x) / 16.0f;  // SGBM outputs 16x disparity
      Eigen::Vector3d point_3d = triangulate(pt, disp);
      points_3d.push_back(point_3d);
    } else {
      points_3d.push_back(Eigen::Vector3d(0, 0, -1));  // Invalid
    }
  }
  
  return points_3d;
}

bool StereoMatcher::isValidDisparity(float disparity) const {
  return disparity > static_cast<float>(config_.min_disparity) && 
         disparity < static_cast<float>(config_.min_disparity + config_.num_disparities);
}

}  // namespace core
}  // namespace stereo_vo
