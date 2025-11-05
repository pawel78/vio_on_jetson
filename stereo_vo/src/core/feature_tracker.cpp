/**
 * @file feature_tracker.cpp
 * @brief Implementation of FeatureTracker
 */

#include "core/feature_tracker.hpp"
#include <algorithm>
#include <opencv2/video/tracking.hpp>

namespace stereo_vo {
namespace core {

FeatureTracker::FeatureTracker(const Config& config)
    : config_(config) {
  // Initialize ORB detector
  orb_detector_ = cv::ORB::create(
      config_.max_features,
      1.2f,  // scale factor
      8,     // nlevels
      31,    // edge threshold
      0,     // first level
      2,     // WTA_K
      cv::ORB::HARRIS_SCORE,
      31,    // patch size
      20     // fast threshold
  );
}

std::vector<cv::KeyPoint> FeatureTracker::detectFeatures(const cv::Mat& image) {
  std::vector<cv::KeyPoint> keypoints;
  
  if (image.empty() || image.type() != CV_8UC1) {
    return keypoints;
  }
  
  orb_detector_->detect(image, keypoints);
  
  // Sort by response and keep top features
  if (keypoints.size() > static_cast<size_t>(config_.max_features)) {
    std::partial_sort(keypoints.begin(), 
                     keypoints.begin() + config_.max_features,
                     keypoints.end(),
                     [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                       return a.response > b.response;
                     });
    keypoints.resize(config_.max_features);
  }
  
  return keypoints;
}

std::vector<TrackedFeature> FeatureTracker::trackFeatures(
    const cv::Mat& prev_image,
    const cv::Mat& curr_image,
    const std::vector<TrackedFeature>& prev_features) {
  
  std::vector<TrackedFeature> tracked;
  
  if (prev_image.empty() || curr_image.empty() || prev_features.empty()) {
    return tracked;
  }
  
  // Extract previous points
  std::vector<cv::Point2f> prev_points;
  prev_points.reserve(prev_features.size());
  for (const auto& feat : prev_features) {
    if (feat.is_valid) {
      prev_points.push_back(feat.point);
    }
  }
  
  if (prev_points.empty()) {
    return tracked;
  }
  
  // Track using pyramidal LK
  std::vector<cv::Point2f> curr_points;
  std::vector<uchar> status;
  std::vector<float> error;
  
  cv::calcOpticalFlowPyrLK(
      prev_image, curr_image,
      prev_points, curr_points,
      status, error,
      cv::Size(config_.lk_win_size, config_.lk_win_size),
      config_.lk_levels,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01)
  );
  
  // Build tracked features
  size_t idx = 0;
  for (size_t i = 0; i < prev_features.size(); ++i) {
    if (!prev_features[i].is_valid) continue;
    
    if (idx < status.size() && status[idx]) {
      TrackedFeature feat = prev_features[i];
      feat.prev_point = feat.point;
      feat.point = curr_points[idx];
      feat.lifetime++;
      feat.quality = 1.0f / (1.0f + error[idx]);
      
      // Check bounds
      if (feat.point.x >= 0 && feat.point.x < curr_image.cols &&
          feat.point.y >= 0 && feat.point.y < curr_image.rows &&
          feat.quality > config_.min_track_quality) {
        tracked.push_back(feat);
      }
    }
    idx++;
  }
  
  return tracked;
}

std::vector<TrackedFeature> FeatureTracker::filterOutliers(
    const std::vector<TrackedFeature>& features,
    const cv::Mat& F) {
  
  std::vector<TrackedFeature> inliers;
  
  if (features.empty() || F.empty()) {
    return features;
  }
  
  for (const auto& feat : features) {
    // Compute epipolar constraint error
    cv::Mat pt1 = (cv::Mat_<double>(3, 1) << feat.prev_point.x, feat.prev_point.y, 1.0);
    cv::Mat pt2 = (cv::Mat_<double>(3, 1) << feat.point.x, feat.point.y, 1.0);
    
    cv::Mat error = pt2.t() * F * pt1;
    double epipolar_error = std::abs(error.at<double>(0, 0));
    
    if (epipolar_error < config_.ransac_thresh) {
      inliers.push_back(feat);
    }
  }
  
  return inliers;
}

void FeatureTracker::reset() {
  tracked_features_.clear();
  next_track_id_ = 0;
}

void FeatureTracker::updateTrackLifetimes() {
  for (auto& feat : tracked_features_) {
    if (feat.is_valid) {
      feat.lifetime++;
    }
  }
}

void FeatureTracker::removeInvalidTracks() {
  tracked_features_.erase(
      std::remove_if(tracked_features_.begin(), tracked_features_.end(),
                     [](const TrackedFeature& f) { return !f.is_valid; }),
      tracked_features_.end()
  );
}

}  // namespace core
}  // namespace stereo_vo
