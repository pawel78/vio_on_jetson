/**
 * @file imu_preintegrator.hpp
 * @brief IMU preintegration for visual-inertial odometry
 */

#ifndef STEREO_VO_CORE_IMU_PREINTEGRATOR_HPP_
#define STEREO_VO_CORE_IMU_PREINTEGRATOR_HPP_

#include <Eigen/Dense>
#include <vector>
#include <deque>

namespace stereo_vo {
namespace core {

/**
 * @struct IMUMeasurement
 * @brief Single IMU measurement
 */
struct IMUMeasurement {
  double timestamp;
  Eigen::Vector3d accel;      ///< Linear acceleration (m/s^2)
  Eigen::Vector3d gyro;       ///< Angular velocity (rad/s)
};

/**
 * @struct PreintegratedIMU
 * @brief Preintegrated IMU measurements between two poses
 */
struct PreintegratedIMU {
  double dt{0.0};                           ///< Total time interval
  Eigen::Vector3d delta_p{0, 0, 0};        ///< Preintegrated position
  Eigen::Vector3d delta_v{0, 0, 0};        ///< Preintegrated velocity
  Eigen::Quaterniond delta_q{1, 0, 0, 0};  ///< Preintegrated rotation
  Eigen::Matrix<double, 9, 9> covariance;  ///< Preintegration covariance
  bool is_valid{false};
};

/**
 * @class IMUPreintegrator
 * @brief Preintegrates IMU measurements for VIO
 */
class IMUPreintegrator {
public:
  /**
   * @struct Config
   * @brief IMU configuration parameters
   */
  struct Config {
    Eigen::Vector3d accel_noise{0.01, 0.01, 0.01};      ///< Accelerometer noise
    Eigen::Vector3d gyro_noise{0.001, 0.001, 0.001};    ///< Gyroscope noise
    Eigen::Vector3d accel_bias{0, 0, 0};                ///< Accelerometer bias
    Eigen::Vector3d gyro_bias{0, 0, 0};                 ///< Gyroscope bias
    Eigen::Vector3d gravity{0, 0, -9.81};               ///< Gravity vector
    bool use_preintegration{true};                       ///< Enable preintegration
  };

  explicit IMUPreintegrator(const Config& config = Config());
  ~IMUPreintegrator() = default;

  /**
   * @brief Add IMU measurement to buffer
   */
  void addMeasurement(const IMUMeasurement& measurement);

  /**
   * @brief Preintegrate IMU measurements between two timestamps
   * @param t0 Start timestamp
   * @param t1 End timestamp
   * @return Preintegrated measurements
   */
  PreintegratedIMU preintegrate(double t0, double t1);

  /**
   * @brief Reset preintegrator state
   */
  void reset();

  /**
   * @brief Update IMU biases
   */
  void updateBias(const Eigen::Vector3d& accel_bias, const Eigen::Vector3d& gyro_bias);

  /**
   * @brief Get configuration
   */
  const Config& getConfig() const { return config_; }

private:
  Config config_;
  std::deque<IMUMeasurement> measurements_;
  static constexpr size_t kMaxBufferSize = 1000;
  
  void integrateStep(
      const IMUMeasurement& prev,
      const IMUMeasurement& curr,
      PreintegratedIMU& result);
      
  void removeOldMeasurements(double timestamp);
};

}  // namespace core
}  // namespace stereo_vo

#endif  // STEREO_VO_CORE_IMU_PREINTEGRATOR_HPP_
