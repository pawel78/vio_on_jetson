/**
 * @file imu_preintegrator.cpp
 * @brief Implementation of IMUPreintegrator
 */

#include "core/imu_preintegrator.hpp"
#include <algorithm>

namespace stereo_vo {
namespace core {

IMUPreintegrator::IMUPreintegrator(const Config& config)
    : config_(config) {
}

void IMUPreintegrator::addMeasurement(const IMUMeasurement& measurement) {
  measurements_.push_back(measurement);
  
  // Keep buffer size limited
  if (measurements_.size() > kMaxBufferSize) {
    measurements_.pop_front();
  }
}

PreintegratedIMU IMUPreintegrator::preintegrate(double t0, double t1) {
  PreintegratedIMU result;
  result.covariance = Eigen::Matrix<double, 9, 9>::Zero();
  
  if (!config_.use_preintegration || measurements_.empty()) {
    return result;
  }
  
  // Find measurements in time range [t0, t1]
  auto it_start = std::lower_bound(
      measurements_.begin(), measurements_.end(), t0,
      [](const IMUMeasurement& m, double t) { return m.timestamp < t; }
  );
  
  auto it_end = std::upper_bound(
      measurements_.begin(), measurements_.end(), t1,
      [](double t, const IMUMeasurement& m) { return t < m.timestamp; }
  );
  
  if (it_start == measurements_.end() || it_end == measurements_.begin()) {
    return result;
  }
  
  // Preintegrate measurements
  for (auto it = it_start; it != it_end && std::next(it) != it_end; ++it) {
    integrateStep(*it, *std::next(it), result);
  }
  
  result.dt = t1 - t0;
  result.is_valid = true;
  
  return result;
}

void IMUPreintegrator::integrateStep(
    const IMUMeasurement& prev,
    const IMUMeasurement& curr,
    PreintegratedIMU& result) {
  
  double dt = curr.timestamp - prev.timestamp;
  if (dt <= 0 || dt > 1.0) {  // Sanity check
    return;
  }
  
  // Remove bias
  Eigen::Vector3d accel = curr.accel - config_.accel_bias;
  Eigen::Vector3d gyro = curr.gyro - config_.gyro_bias;
  
  // Integrate rotation (simple euler integration)
  Eigen::Vector3d omega = gyro * dt;
  Eigen::Quaterniond dq;
  double theta = omega.norm();
  if (theta > 1e-6) {
    Eigen::Vector3d axis = omega / theta;
    dq = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
  } else {
    dq = Eigen::Quaterniond::Identity();
  }
  
  result.delta_q = result.delta_q * dq;
  result.delta_q.normalize();
  
  // Integrate velocity
  Eigen::Vector3d accel_world = result.delta_q * accel;
  result.delta_v += (accel_world + config_.gravity) * dt;
  
  // Integrate position
  result.delta_p += result.delta_v * dt + 0.5 * (accel_world + config_.gravity) * dt * dt;
  
  // Update covariance (simplified)
  double accel_noise_sq = config_.accel_noise.squaredNorm();
  double gyro_noise_sq = config_.gyro_noise.squaredNorm();
  
  result.covariance.block<3, 3>(0, 0) += Eigen::Matrix3d::Identity() * accel_noise_sq * dt * dt;
  result.covariance.block<3, 3>(3, 3) += Eigen::Matrix3d::Identity() * accel_noise_sq * dt;
  result.covariance.block<3, 3>(6, 6) += Eigen::Matrix3d::Identity() * gyro_noise_sq * dt;
}

void IMUPreintegrator::reset() {
  measurements_.clear();
}

void IMUPreintegrator::updateBias(
    const Eigen::Vector3d& accel_bias,
    const Eigen::Vector3d& gyro_bias) {
  config_.accel_bias = accel_bias;
  config_.gyro_bias = gyro_bias;
}

void IMUPreintegrator::removeOldMeasurements(double timestamp) {
  while (!measurements_.empty() && measurements_.front().timestamp < timestamp) {
    measurements_.pop_front();
  }
}

}  // namespace core
}  // namespace stereo_vo
