/**
 * @file pose_estimator.hpp
 * @brief Pose estimation using PnP RANSAC and bundle adjustment
 */

#ifndef STEREO_VO_CORE_POSE_ESTIMATOR_HPP_
#define STEREO_VO_CORE_POSE_ESTIMATOR_HPP_

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace stereo_vo {
namespace core {

/**
 * @struct Pose
 * @brief 6-DOF pose representation
 */
struct Pose {
  Eigen::Matrix3d R;          ///< Rotation matrix
  Eigen::Vector3d t;          ///< Translation vector
  Eigen::Matrix<double, 6, 6> covariance;  ///< Pose covariance
  double timestamp{0.0};      ///< Timestamp
  bool is_valid{false};       ///< Whether pose is valid
  
  /// Convert to 4x4 transformation matrix
  Eigen::Matrix4d toMatrix() const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
  }
  
  /// Create pose from transformation matrix
  static Pose fromMatrix(const Eigen::Matrix4d& T) {
    Pose pose;
    pose.R = T.block<3, 3>(0, 0);
    pose.t = T.block<3, 1>(0, 3);
    pose.is_valid = true;
    return pose;
  }
};

/**
 * @class PoseEstimator
 * @brief Estimates camera pose using PnP with RANSAC and optional BA
 */
class PoseEstimator {
public:
  /**
   * @struct Config
   * @brief Configuration for pose estimation
   */
  struct Config {
    bool use_ransac{true};           ///< Use RANSAC for outlier rejection
    double ransac_thresh{2.0};       ///< RANSAC reprojection threshold (pixels)
    int ransac_iterations{100};      ///< Maximum RANSAC iterations
    double ransac_confidence{0.99};  ///< RANSAC confidence level
    int min_inliers{20};             ///< Minimum inliers for valid pose
    bool use_ba{true};               ///< Use bundle adjustment
    int ba_iterations{10};           ///< Bundle adjustment iterations
    bool use_huber{true};            ///< Use Huber loss in BA
    double huber_delta{1.0};         ///< Huber loss delta
  };

  explicit PoseEstimator(const Config& config = Config());
  ~PoseEstimator();

  /**
   * @brief Estimate pose from 2D-3D correspondences using PnP
   * @param points_2d 2D image points
   * @param points_3d Corresponding 3D points
   * @param K Camera intrinsic matrix
   * @param initial_pose Optional initial pose estimate
   * @return Estimated pose
   */
  Pose estimatePose(
      const std::vector<cv::Point2f>& points_2d,
      const std::vector<Eigen::Vector3d>& points_3d,
      const Eigen::Matrix3d& K,
      const Pose* initial_pose = nullptr);

  /**
   * @brief Refine pose using bundle adjustment
   * @param pose Initial pose estimate
   * @param points_2d 2D observations
   * @param points_3d 3D landmarks
   * @param K Camera intrinsic matrix
   * @return Refined pose
   */
  Pose refinePose(
      const Pose& pose,
      const std::vector<cv::Point2f>& points_2d,
      const std::vector<Eigen::Vector3d>& points_3d,
      const Eigen::Matrix3d& K);

  /**
   * @brief Get inlier mask from last PnP estimation
   */
  const std::vector<bool>& getInlierMask() const { return inlier_mask_; }

  /**
   * @brief Get number of inliers from last estimation
   */
  int getInlierCount() const;

private:
  Config config_;
  std::vector<bool> inlier_mask_;
  
  bool solvePnPRansac(
      const std::vector<cv::Point2f>& points_2d,
      const std::vector<Eigen::Vector3d>& points_3d,
      const Eigen::Matrix3d& K,
      Pose& pose);
      
  void eigenToCV(const Eigen::Matrix3d& K_eigen, cv::Mat& K_cv);
  void cvToEigen(const cv::Mat& rvec, const cv::Mat& tvec, Pose& pose);
};

}  // namespace core
}  // namespace stereo_vo

#endif  // STEREO_VO_CORE_POSE_ESTIMATOR_HPP_
