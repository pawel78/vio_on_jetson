/**
 * @file sliding_window.cpp
 * @brief Implementation of SlidingWindow bundle adjustment
 */

#include "core/sliding_window.hpp"
#include <algorithm>
#include <iostream>

#ifdef USE_GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#endif

namespace stereo_vo {
namespace core {

SlidingWindow::SlidingWindow(const Config& config)
    : config_(config) {
#ifdef USE_GTSAM
  optimizer_ = std::make_unique<GTSAMOptimizer>();
#endif
}

SlidingWindow::~SlidingWindow() = default;

void SlidingWindow::addKeyFrame(const KeyFrame& keyframe) {
  keyframes_.push_back(keyframe);
  
  // Marginalize old frames if window is full
  if (keyframes_.size() > static_cast<size_t>(config_.window_size)) {
    marginalizeOldFrames();
  }
}

void SlidingWindow::addObservation(
    int landmark_id,
    int frame_id,
    const cv::Point2f& observation) {
  
  auto& landmark = landmarks_[landmark_id];
  if (landmark.id == 0) {
    landmark.id = landmark_id;
  }
  
  landmark.observed_frames.push_back(frame_id);
  
  // Add observation to corresponding keyframe
  for (auto& kf : keyframes_) {
    if (kf.id == frame_id) {
      kf.observations_2d.push_back(observation);
      kf.landmark_ids.push_back(landmark_id);
      break;
    }
  }
}

bool SlidingWindow::optimize(const Eigen::Matrix3d& K) {
#ifdef USE_GTSAM
  return optimizeWithGTSAM(K);
#else
  std::cerr << "GTSAM not available, skipping BA optimization" << std::endl;
  return false;
#endif
}

#ifdef USE_GTSAM

class SlidingWindow::GTSAMOptimizer {
public:
  bool optimize(
      const std::deque<KeyFrame>& keyframes,
      std::map<int, Landmark>& landmarks,
      const Eigen::Matrix3d& K,
      const SlidingWindow::Config& config) {
    
    using gtsam::symbol_shorthand::X;  // Pose
    using gtsam::symbol_shorthand::L;  // Landmark
    
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_estimate;
    
    // Camera calibration
    auto cal3 = boost::make_shared<gtsam::Cal3_S2>(
        K(0, 0), K(1, 1), 0.0, K(0, 2), K(1, 2)
    );
    
    // Add pose prior for first frame (anchor)
    if (!keyframes.empty()) {
      const auto& first_kf = keyframes.front();
      gtsam::Pose3 first_pose(
          gtsam::Rot3(first_kf.pose.R),
          gtsam::Point3(first_kf.pose.t.x(), first_kf.pose.t.y(), first_kf.pose.t.z())
      );
      
      auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.001, 0.001, 0.001, 0.01, 0.01, 0.01).finished()
      );
      graph.addPrior(X(first_kf.id), first_pose, prior_noise);
      initial_estimate.insert(X(first_kf.id), first_pose);
    }
    
    // Add all poses
    for (const auto& kf : keyframes) {
      if (kf.id == keyframes.front().id) continue;  // Already added
      
      gtsam::Pose3 pose(
          gtsam::Rot3(kf.pose.R),
          gtsam::Point3(kf.pose.t.x(), kf.pose.t.y(), kf.pose.t.z())
      );
      initial_estimate.insert(X(kf.id), pose);
    }
    
    // Add landmarks
    for (const auto& [lm_id, landmark] : landmarks) {
      if (!landmark.is_valid || 
          landmark.observed_frames.size() < static_cast<size_t>(config.min_observations)) {
        continue;
      }
      
      gtsam::Point3 point(
          landmark.position.x(),
          landmark.position.y(),
          landmark.position.z()
      );
      initial_estimate.insert(L(lm_id), point);
    }
    
    // Add projection factors
    auto measurement_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
    if (config.use_robust_loss) {
      auto huber = gtsam::noiseModel::mEstimator::Huber::Create(config.huber_delta);
      measurement_noise = gtsam::noiseModel::Robust::Create(huber, measurement_noise);
    }
    
    for (const auto& kf : keyframes) {
      for (size_t i = 0; i < kf.landmark_ids.size(); ++i) {
        int lm_id = kf.landmark_ids[i];
        const auto& obs = kf.observations_2d[i];
        
        if (landmarks.find(lm_id) == landmarks.end() || !landmarks[lm_id].is_valid) {
          continue;
        }
        
        gtsam::Point2 measured(obs.x, obs.y);
        graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
            measured, measurement_noise, X(kf.id), L(lm_id), cal3
        );
      }
    }
    
    // Optimize
    try {
      gtsam::LevenbergMarquardtParams params;
      params.maxIterations = config.max_iterations;
      params.relativeErrorTol = config.convergence_threshold;
      params.absoluteErrorTol = config.convergence_threshold;
      
      gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
      gtsam::Values result = optimizer.optimize();
      
      // Update poses and landmarks with optimized values
      for (auto& kf : const_cast<std::deque<KeyFrame>&>(keyframes)) {
        if (result.exists(X(kf.id))) {
          gtsam::Pose3 optimized_pose = result.at<gtsam::Pose3>(X(kf.id));
          kf.pose.R = optimized_pose.rotation().matrix();
          kf.pose.t = optimized_pose.translation().vector();
        }
      }
      
      for (auto& [lm_id, landmark] : landmarks) {
        if (result.exists(L(lm_id))) {
          gtsam::Point3 optimized_point = result.at<gtsam::Point3>(L(lm_id));
          landmark.position = optimized_point.vector();
        }
      }
      
      return true;
      
    } catch (const std::exception& e) {
      std::cerr << "GTSAM optimization failed: " << e.what() << std::endl;
      return false;
    }
  }
};

bool SlidingWindow::optimizeWithGTSAM(const Eigen::Matrix3d& K) {
  if (!optimizer_) {
    return false;
  }
  
  return optimizer_->optimize(keyframes_, landmarks_, K, config_);
}

#endif  // USE_GTSAM

const Pose* SlidingWindow::getPose(int frame_id) const {
  for (const auto& kf : keyframes_) {
    if (kf.id == frame_id) {
      return &kf.pose;
    }
  }
  return nullptr;
}

void SlidingWindow::reset() {
  keyframes_.clear();
  landmarks_.clear();
  next_frame_id_ = 0;
}

void SlidingWindow::marginalizeOldFrames() {
  if (keyframes_.size() <= static_cast<size_t>(config_.window_size)) {
    return;
  }
  
  // Remove oldest frame
  int removed_frame_id = keyframes_.front().id;
  keyframes_.pop_front();
  
  // Remove landmarks only observed by removed frame
  for (auto it = landmarks_.begin(); it != landmarks_.end();) {
    auto& frames = it->second.observed_frames;
    frames.erase(
        std::remove(frames.begin(), frames.end(), removed_frame_id),
        frames.end()
    );
    
    if (frames.size() < static_cast<size_t>(config_.min_observations)) {
      it = landmarks_.erase(it);
    } else {
      ++it;
    }
  }
}

void SlidingWindow::removeInvalidLandmarks() {
  for (auto it = landmarks_.begin(); it != landmarks_.end();) {
    if (!it->second.is_valid || 
        it->second.observed_frames.size() < static_cast<size_t>(config_.min_observations)) {
      it = landmarks_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace core
}  // namespace stereo_vo
