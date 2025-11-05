/**
 * @file test_pose_estimator.cpp
 * @brief Unit tests for PoseEstimator
 */

#include <gtest/gtest.h>
#include "core/pose_estimator.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace stereo_vo::core;

class PoseEstimatorTest : public ::testing::Test {
protected:
  void SetUp() override {
    config_.ransac_thresh = 2.0;
    config_.min_inliers = 4;
    estimator_ = std::make_unique<PoseEstimator>(config_);
    
    // Create simple camera matrix
    K_ << 500, 0, 320,
          0, 500, 240,
          0, 0, 1;
  }
  
  PoseEstimator::Config config_;
  std::unique_ptr<PoseEstimator> estimator_;
  Eigen::Matrix3d K_;
};

TEST_F(PoseEstimatorTest, EstimatePoseIdentity) {
  // Create synthetic 3D points
  std::vector<Eigen::Vector3d> points_3d;
  std::vector<cv::Point2f> points_2d;
  
  for (int i = 0; i < 10; ++i) {
    Eigen::Vector3d pt3d(i * 0.1, i * 0.1, 1.0);
    points_3d.push_back(pt3d);
    
    // Project to 2D (identity pose)
    Eigen::Vector3d proj = K_ * pt3d;
    points_2d.push_back(cv::Point2f(proj.x() / proj.z(), proj.y() / proj.z()));
  }
  
  // Estimate pose (should be close to identity)
  Pose estimated = estimator_->estimatePose(points_2d, points_3d, K_);
  
  EXPECT_TRUE(estimated.is_valid);
  EXPECT_GT(estimator_->getInlierCount(), 0);
}

TEST_F(PoseEstimatorTest, EstimatePoseInsufficientPoints) {
  std::vector<Eigen::Vector3d> points_3d;
  std::vector<cv::Point2f> points_2d;
  
  // Only 2 points (insufficient)
  points_3d.push_back(Eigen::Vector3d(0, 0, 1));
  points_3d.push_back(Eigen::Vector3d(1, 1, 1));
  points_2d.push_back(cv::Point2f(320, 240));
  points_2d.push_back(cv::Point2f(420, 340));
  
  Pose estimated = estimator_->estimatePose(points_2d, points_3d, K_);
  
  EXPECT_FALSE(estimated.is_valid);
}

TEST_F(PoseEstimatorTest, PoseToMatrix) {
  Pose pose;
  pose.R = Eigen::Matrix3d::Identity();
  pose.t = Eigen::Vector3d(1, 2, 3);
  pose.is_valid = true;
  
  Eigen::Matrix4d T = pose.toMatrix();
  
  EXPECT_DOUBLE_EQ(T(0, 3), 1.0);
  EXPECT_DOUBLE_EQ(T(1, 3), 2.0);
  EXPECT_DOUBLE_EQ(T(2, 3), 3.0);
  EXPECT_DOUBLE_EQ(T(3, 3), 1.0);
}

TEST_F(PoseEstimatorTest, PoseFromMatrix) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 3) = 1.0;
  T(1, 3) = 2.0;
  T(2, 3) = 3.0;
  
  Pose pose = Pose::fromMatrix(T);
  
  EXPECT_TRUE(pose.is_valid);
  EXPECT_DOUBLE_EQ(pose.t.x(), 1.0);
  EXPECT_DOUBLE_EQ(pose.t.y(), 2.0);
  EXPECT_DOUBLE_EQ(pose.t.z(), 3.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
