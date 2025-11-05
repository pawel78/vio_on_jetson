/**
 * @file pose_estimator.cpp
 * @brief Implementation of PoseEstimator
 */

#include "core/pose_estimator.hpp"
#include <opencv2/calib3d.hpp>
#include <iostream>

namespace stereo_vo {
namespace core {

PoseEstimator::PoseEstimator(const Config& config)
    : config_(config) {
}

PoseEstimator::~PoseEstimator() = default;

Pose PoseEstimator::estimatePose(
    const std::vector<cv::Point2f>& points_2d,
    const std::vector<Eigen::Vector3d>& points_3d,
    const Eigen::Matrix3d& K,
    const Pose* initial_pose) {
  
  Pose estimated_pose;
  
  if (points_2d.size() != points_3d.size() || points_2d.size() < 4) {
    return estimated_pose;  // Invalid input
  }
  
  // Filter out invalid 3D points
  std::vector<cv::Point2f> valid_2d;
  std::vector<cv::Point3f> valid_3d;
  
  for (size_t i = 0; i < points_3d.size(); ++i) {
    if (points_3d[i].z() > 0) {  // Valid depth
      valid_2d.push_back(points_2d[i]);
      valid_3d.push_back(cv::Point3f(
          static_cast<float>(points_3d[i].x()),
          static_cast<float>(points_3d[i].y()),
          static_cast<float>(points_3d[i].z())
      ));
    }
  }
  
  if (valid_2d.size() < 4) {
    return estimated_pose;
  }
  
  // Solve PnP
  if (!solvePnPRansac(valid_2d, points_3d, K, estimated_pose)) {
    return estimated_pose;
  }
  
  // Refine with BA if enabled
  if (config_.use_ba && estimated_pose.is_valid) {
    estimated_pose = refinePose(estimated_pose, valid_2d, points_3d, K);
  }
  
  return estimated_pose;
}

bool PoseEstimator::solvePnPRansac(
    const std::vector<cv::Point2f>& points_2d,
    const std::vector<Eigen::Vector3d>& points_3d,
    const Eigen::Matrix3d& K,
    Pose& pose) {
  
  // Convert 3D points
  std::vector<cv::Point3f> object_points;
  for (const auto& pt : points_3d) {
    if (pt.z() > 0) {
      object_points.push_back(cv::Point3f(
          static_cast<float>(pt.x()),
          static_cast<float>(pt.y()),
          static_cast<float>(pt.z())
      ));
    }
  }
  
  if (object_points.size() < 4) {
    return false;
  }
  
  // Convert camera matrix
  cv::Mat K_cv;
  eigenToCV(K, K_cv);
  
  // Solve PnP with RANSAC
  cv::Mat rvec, tvec;
  std::vector<int> inliers_cv;
  
  bool success = cv::solvePnPRansac(
      object_points,
      points_2d,
      K_cv,
      cv::Mat(),  // No distortion (assuming rectified)
      rvec,
      tvec,
      false,      // useExtrinsicGuess
      config_.ransac_iterations,
      config_.ransac_thresh,
      config_.ransac_confidence,
      inliers_cv,
      cv::SOLVEPNP_EPNP
  );
  
  if (!success || inliers_cv.size() < static_cast<size_t>(config_.min_inliers)) {
    return false;
  }
  
  // Convert result to Eigen
  cvToEigen(rvec, tvec, pose);
  pose.is_valid = true;
  
  // Store inlier mask
  inlier_mask_.resize(points_2d.size(), false);
  for (int idx : inliers_cv) {
    if (idx >= 0 && idx < static_cast<int>(inlier_mask_.size())) {
      inlier_mask_[idx] = true;
    }
  }
  
  return true;
}

Pose PoseEstimator::refinePose(
    const Pose& pose,
    const std::vector<cv::Point2f>& points_2d,
    const std::vector<Eigen::Vector3d>& points_3d,
    const Eigen::Matrix3d& K) {
  
  // Simple refinement using iterative PnP
  // For full BA with GTSAM, see sliding_window.cpp
  
  Pose refined = pose;
  
  // Convert pose to rvec/tvec
  cv::Mat R_cv, rvec, tvec;
  cv::eigen2cv(pose.R, R_cv);
  cv::Rodrigues(R_cv, rvec);
  cv::eigen2cv(pose.t, tvec);
  
  // Convert camera matrix
  cv::Mat K_cv;
  eigenToCV(K, K_cv);
  
  // Convert 3D points
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> valid_2d;
  
  for (size_t i = 0; i < points_3d.size(); ++i) {
    if (points_3d[i].z() > 0 && i < inlier_mask_.size() && inlier_mask_[i]) {
      object_points.push_back(cv::Point3f(
          static_cast<float>(points_3d[i].x()),
          static_cast<float>(points_3d[i].y()),
          static_cast<float>(points_3d[i].z())
      ));
      valid_2d.push_back(points_2d[i]);
    }
  }
  
  if (object_points.size() < 4) {
    return refined;
  }
  
  // Refine using iterative PnP
  cv::solvePnP(
      object_points,
      valid_2d,
      K_cv,
      cv::Mat(),
      rvec,
      tvec,
      true,  // use extrinsic guess
      cv::SOLVEPNP_ITERATIVE
  );
  
  cvToEigen(rvec, tvec, refined);
  refined.is_valid = true;
  
  return refined;
}

int PoseEstimator::getInlierCount() const {
  return std::count(inlier_mask_.begin(), inlier_mask_.end(), true);
}

void PoseEstimator::eigenToCV(const Eigen::Matrix3d& K_eigen, cv::Mat& K_cv) {
  K_cv = cv::Mat::eye(3, 3, CV_64F);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      K_cv.at<double>(i, j) = K_eigen(i, j);
    }
  }
}

void PoseEstimator::cvToEigen(const cv::Mat& rvec, const cv::Mat& tvec, Pose& pose) {
  // Convert rotation vector to matrix
  cv::Mat R_cv;
  cv::Rodrigues(rvec, R_cv);
  
  // Convert to Eigen
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      pose.R(i, j) = R_cv.at<double>(i, j);
    }
    pose.t(i) = tvec.at<double>(i);
  }
}

}  // namespace core
}  // namespace stereo_vo
