/**
 * @file stereo_matcher.hpp
 * @brief Stereo matching and 3D triangulation
 */

#ifndef STEREO_VO_CORE_STEREO_MATCHER_HPP_
#define STEREO_VO_CORE_STEREO_MATCHER_HPP_

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace stereo_vo {
namespace core {

/**
 * @class StereoMatcher
 * @brief Performs stereo matching (SGBM) and triangulates 3D points
 */
class StereoMatcher {
public:
  /**
   * @struct Config
   * @brief Configuration for stereo matching
   */
  struct Config {
    std::string method{"sgbm"};    ///< "sgbm" or "cuda_sgbm"
    int min_disparity{0};          ///< Minimum disparity
    int num_disparities{128};      ///< Number of disparities (must be divisible by 16)
    int block_size{7};             ///< Block size for matching
    int p1{8};                     ///< Penalty for small disparity changes
    int p2{32};                    ///< Penalty for large disparity changes
    int disp12_max_diff{1};        ///< Maximum allowed difference in left-right check
    int pre_filter_cap{63};        ///< Truncation value for prefilter
    int uniqueness_ratio{10};      ///< Uniqueness ratio percentage
    int speckle_window_size{100};  ///< Maximum speckle size
    int speckle_range{32};         ///< Maximum disparity variation in speckle
    bool use_mode_HH{true};        ///< Use full-scale two-pass algorithm
  };

  /**
   * @struct CameraParams
   * @brief Camera calibration parameters
   */
  struct CameraParams {
    Eigen::Matrix3d K_left;     ///< Left camera intrinsic matrix
    Eigen::Matrix3d K_right;    ///< Right camera intrinsic matrix
    Eigen::Vector<double, 5> D_left;   ///< Left distortion coefficients
    Eigen::Vector<double, 5> D_right;  ///< Right distortion coefficients
    Eigen::Matrix3d R;          ///< Rotation matrix (left to right)
    Eigen::Vector3d t;          ///< Translation vector (left to right)
    double baseline;            ///< Baseline distance in meters
  };

  explicit StereoMatcher(const Config& config, const CameraParams& params);
  ~StereoMatcher() = default;

  /**
   * @brief Compute disparity map from stereo pair
   * @param left_image Rectified left image (grayscale)
   * @param right_image Rectified right image (grayscale)
   * @return Disparity map (CV_16S, divide by 16 for actual disparity)
   */
  cv::Mat computeDisparity(const cv::Mat& left_image, const cv::Mat& right_image);

  /**
   * @brief Triangulate 3D point from stereo match
   * @param left_point Point in left image
   * @param disparity Disparity value at that point
   * @return 3D point in left camera frame
   */
  Eigen::Vector3d triangulate(const cv::Point2f& left_point, float disparity) const;

  /**
   * @brief Triangulate multiple 3D points
   * @param left_points Points in left image
   * @param disparity_map Disparity map
   * @return Vector of 3D points (invalid points have z < 0)
   */
  std::vector<Eigen::Vector3d> triangulatePoints(
      const std::vector<cv::Point2f>& left_points,
      const cv::Mat& disparity_map) const;

  /**
   * @brief Get camera parameters
   */
  const CameraParams& getCameraParams() const { return params_; }

private:
  Config config_;
  CameraParams params_;
  cv::Ptr<cv::StereoSGBM> sgbm_;
  
#ifdef USE_CUDA
  cv::Ptr<cv::cuda::StereoSGM> cuda_sgbm_;
  bool use_cuda_{false};
#endif

  void initializeMatcher();
  bool isValidDisparity(float disparity) const;
};

}  // namespace core
}  // namespace stereo_vo

#endif  // STEREO_VO_CORE_STEREO_MATCHER_HPP_
