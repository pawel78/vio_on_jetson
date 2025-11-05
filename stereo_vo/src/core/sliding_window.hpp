/**
 * @file sliding_window.hpp
 * @brief Sliding window bundle adjustment using GTSAM
 */

#ifndef STEREO_VO_CORE_SLIDING_WINDOW_HPP_
#define STEREO_VO_CORE_SLIDING_WINDOW_HPP_

#include "pose_estimator.hpp"
#include "imu_preintegrator.hpp"
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <map>

namespace stereo_vo {
namespace core {

/**
 * @struct Landmark
 * @brief 3D landmark with observations
 */
struct Landmark {
  int id;
  Eigen::Vector3d position;
  std::vector<int> observed_frames;  ///< Frame IDs that observe this landmark
  bool is_valid{true};
};

/**
 * @struct KeyFrame
 * @brief Keyframe in sliding window
 */
struct KeyFrame {
  int id;
  Pose pose;
  double timestamp;
  std::vector<cv::Point2f> observations_2d;
  std::vector<int> landmark_ids;
};

/**
 * @class SlidingWindow
 * @brief Manages sliding window and performs local bundle adjustment
 */
class SlidingWindow {
public:
  /**
   * @struct Config
   * @brief Configuration for sliding window BA
   */
  struct Config {
    int window_size{7};              ///< Number of keyframes in window
    int min_observations{3};         ///< Minimum observations per landmark
    bool use_imu{false};             ///< Include IMU constraints
    bool use_robust_loss{true};      ///< Use Huber/Cauchy loss
    double huber_delta{1.0};         ///< Huber loss threshold
    int max_iterations{20};          ///< Maximum BA iterations
    double convergence_threshold{1e-4}; ///< Convergence threshold
  };

  explicit SlidingWindow(const Config& config = Config());
  ~SlidingWindow();

  /**
   * @brief Add new keyframe to window
   * @param keyframe New keyframe to add
   */
  void addKeyFrame(const KeyFrame& keyframe);

  /**
   * @brief Add landmark observation
   * @param landmark_id Landmark ID
   * @param frame_id Frame ID
   * @param observation 2D observation
   */
  void addObservation(int landmark_id, int frame_id, const cv::Point2f& observation);

  /**
   * @brief Perform bundle adjustment on current window
   * @param K Camera intrinsic matrix
   * @return True if optimization succeeded
   */
  bool optimize(const Eigen::Matrix3d& K);

  /**
   * @brief Get optimized pose for a keyframe
   * @param frame_id Frame ID
   * @return Optimized pose (or null if not found)
   */
  const Pose* getPose(int frame_id) const;

  /**
   * @brief Get current window size
   */
  size_t getWindowSize() const { return keyframes_.size(); }

  /**
   * @brief Clear all keyframes and landmarks
   */
  void reset();

  /**
   * @brief Get all landmarks
   */
  const std::map<int, Landmark>& getLandmarks() const { return landmarks_; }

private:
  Config config_;
  std::deque<KeyFrame> keyframes_;
  std::map<int, Landmark> landmarks_;
  int next_frame_id_{0};
  
  void marginalizeOldFrames();
  void removeInvalidLandmarks();
  
#ifdef USE_GTSAM
  class GTSAMOptimizer;
  std::unique_ptr<GTSAMOptimizer> optimizer_;
  bool optimizeWithGTSAM(const Eigen::Matrix3d& K);
#else
  bool optimizeWithCeres(const Eigen::Matrix3d& K);
#endif
};

}  // namespace core
}  // namespace stereo_vo

#endif  // STEREO_VO_CORE_SLIDING_WINDOW_HPP_
