/**
 * @file feature_tracker.hpp
 * @brief ORB feature detection and pyramidal LK tracking for visual odometry
 */

#ifndef STEREO_VO_CORE_FEATURE_TRACKER_HPP_
#define STEREO_VO_CORE_FEATURE_TRACKER_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace stereo_vo {
namespace core {

/**
 * @struct TrackedFeature
 * @brief Represents a feature point tracked across frames
 */
struct TrackedFeature {
  cv::Point2f point;         ///< Current 2D position in image
  cv::Point2f prev_point;    ///< Previous 2D position
  cv::Point3f point_3d;      ///< 3D position from stereo triangulation
  int track_id;              ///< Unique ID for this track
  int lifetime;              ///< Number of frames tracked
  bool is_valid;             ///< Whether track is still valid
  float quality;             ///< Track quality score
};

/**
 * @class FeatureTracker
 * @brief Detects and tracks ORB features using pyramidal Lucas-Kanade
 */
class FeatureTracker {
public:
  /**
   * @struct Config
   * @brief Configuration parameters for feature tracking
   */
  struct Config {
    int max_features{1500};        ///< Maximum number of features to track
    int lk_levels{3};              ///< Number of pyramid levels for LK
    int lk_win_size{21};           ///< Window size for LK
    float min_track_quality{0.01}; ///< Minimum quality for tracking
    float ransac_thresh{2.0};      ///< RANSAC threshold in pixels
    int min_track_lifetime{3};     ///< Minimum frames to consider reliable
  };

  explicit FeatureTracker(const Config& config = Config());
  ~FeatureTracker() = default;

  /**
   * @brief Detect new ORB features in image
   * @param image Input grayscale image
   * @return Detected keypoints
   */
  std::vector<cv::KeyPoint> detectFeatures(const cv::Mat& image);

  /**
   * @brief Track features from previous frame to current
   * @param prev_image Previous grayscale image
   * @param curr_image Current grayscale image
   * @param prev_features Features from previous frame
   * @return Tracked features with updated positions
   */
  std::vector<TrackedFeature> trackFeatures(
      const cv::Mat& prev_image,
      const cv::Mat& curr_image,
      const std::vector<TrackedFeature>& prev_features);

  /**
   * @brief Filter outliers using epipolar constraint
   * @param features Input features
   * @param F Fundamental matrix
   * @return Filtered features (inliers only)
   */
  std::vector<TrackedFeature> filterOutliers(
      const std::vector<TrackedFeature>& features,
      const cv::Mat& F);

  /**
   * @brief Get current tracked features
   */
  const std::vector<TrackedFeature>& getTrackedFeatures() const {
    return tracked_features_;
  }

  /**
   * @brief Reset tracker state
   */
  void reset();

private:
  Config config_;
  cv::Ptr<cv::ORB> orb_detector_;
  std::vector<TrackedFeature> tracked_features_;
  int next_track_id_{0};
  
  void updateTrackLifetimes();
  void removeInvalidTracks();
};

}  // namespace core
}  // namespace stereo_vo

#endif  // STEREO_VO_CORE_FEATURE_TRACKER_HPP_
