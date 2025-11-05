/**
 * @file test_feature_tracker.cpp
 * @brief Unit tests for FeatureTracker
 */

#include <gtest/gtest.h>
#include "core/feature_tracker.hpp"
#include <opencv2/opencv.hpp>

using namespace stereo_vo::core;

class FeatureTrackerTest : public ::testing::Test {
protected:
  void SetUp() override {
    config_.max_features = 100;
    config_.lk_levels = 3;
    tracker_ = std::make_unique<FeatureTracker>(config_);
    
    // Create a simple test image
    test_image_ = cv::Mat::zeros(480, 640, CV_8UC1);
    
    // Add some features (corners)
    for (int i = 0; i < 10; ++i) {
      cv::circle(test_image_, cv::Point(100 + i * 50, 100 + i * 30), 5, cv::Scalar(255), -1);
    }
  }
  
  FeatureTracker::Config config_;
  std::unique_ptr<FeatureTracker> tracker_;
  cv::Mat test_image_;
};

TEST_F(FeatureTrackerTest, DetectFeatures) {
  auto keypoints = tracker_->detectFeatures(test_image_);
  
  EXPECT_GT(keypoints.size(), 0);
  EXPECT_LE(keypoints.size(), static_cast<size_t>(config_.max_features));
}

TEST_F(FeatureTrackerTest, DetectFeaturesEmptyImage) {
  cv::Mat empty_image;
  auto keypoints = tracker_->detectFeatures(empty_image);
  
  EXPECT_EQ(keypoints.size(), 0);
}

TEST_F(FeatureTrackerTest, TrackFeatures) {
  // Create two similar images
  cv::Mat prev_image = test_image_.clone();
  cv::Mat curr_image = test_image_.clone();
  
  // Detect features in first image
  auto keypoints = tracker_->detectFeatures(prev_image);
  
  std::vector<TrackedFeature> prev_features;
  for (size_t i = 0; i < keypoints.size(); ++i) {
    TrackedFeature feat;
    feat.point = keypoints[i].pt;
    feat.prev_point = feat.point;
    feat.track_id = static_cast<int>(i);
    feat.lifetime = 0;
    feat.is_valid = true;
    feat.quality = 1.0f;
    prev_features.push_back(feat);
  }
  
  // Track features
  auto tracked = tracker_->trackFeatures(prev_image, curr_image, prev_features);
  
  // Since images are identical, most features should track successfully
  EXPECT_GT(tracked.size(), 0);
}

TEST_F(FeatureTrackerTest, Reset) {
  tracker_->reset();
  
  EXPECT_EQ(tracker_->getTrackedFeatures().size(), 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
