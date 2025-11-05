/**
 * @file lsm9ds0_node.cpp
 * @brief LSM9DS0 9-DOF IMU driver for ROS 2
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace lsm9ds0 {

// LSM9DS0 Register addresses
constexpr uint8_t WHO_AM_I_G = 0x0F;
constexpr uint8_t WHO_AM_I_XM = 0x0F;
constexpr uint8_t CTRL_REG1_G = 0x20;
constexpr uint8_t CTRL_REG4_G = 0x23;
constexpr uint8_t OUT_X_L_G = 0x28;
constexpr uint8_t CTRL_REG1_XM = 0x20;
constexpr uint8_t CTRL_REG2_XM = 0x21;
constexpr uint8_t OUT_X_L_A = 0x28;
constexpr uint8_t OUT_X_L_M = 0x08;

// WHO_AM_I values
constexpr uint8_t WHO_AM_I_G_VALUE = 0xD4;
constexpr uint8_t WHO_AM_I_XM_VALUE = 0x49;

class LSM9DS0Node : public rclcpp::Node {
public:
  LSM9DS0Node() : Node("lsm9ds0_node") {
    // Declare parameters
    declare_parameter("i2c_bus", 7);
    declare_parameter("imu_addr_gyro", 0x6B);
    declare_parameter("imu_addr_mag", 0x1D);
    declare_parameter("imu_rate_hz", 200);
    declare_parameter("accel_range_g", 2);
    declare_parameter("gyro_range_dps", 245);
    declare_parameter("frame_id", "imu_link");
    
    // Bias and scale parameters
    declare_parameter("accel_bias_x", 0.0);
    declare_parameter("accel_bias_y", 0.0);
    declare_parameter("accel_bias_z", 0.0);
    declare_parameter("gyro_bias_x", 0.0);
    declare_parameter("gyro_bias_y", 0.0);
    declare_parameter("gyro_bias_z", 0.0);
    declare_parameter("accel_scale_x", 1.0);
    declare_parameter("accel_scale_y", 1.0);
    declare_parameter("accel_scale_z", 1.0);
    declare_parameter("gyro_scale_x", 1.0);
    declare_parameter("gyro_scale_y", 1.0);
    declare_parameter("gyro_scale_z", 1.0);
    
    // Orientation (RPY in degrees)
    declare_parameter("imu_orientation_rpy_deg", std::vector<double>{0.0, 0.0, 0.0});
    
    // Noise parameters
    declare_parameter("accel_noise", 0.01);
    declare_parameter("gyro_noise", 0.001);
    
    // Load parameters
    loadParameters();
    
    // Initialize I2C
    if (!initializeI2C()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize I2C");
      rclcpp::shutdown();
      return;
    }
    
    // Verify WHO_AM_I
    if (!verifyDevice()) {
      RCLCPP_ERROR(get_logger(), "Device verification failed");
      rclcpp::shutdown();
      return;
    }
    
    // Configure sensor
    configureSensor();
    
    // Publishers
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 100);
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("/imu/temp", 10);
    
    // Self-test service
    self_test_srv_ = create_service<std_srvs::srv::Trigger>(
        "/imu/self_test",
        std::bind(&LSM9DS0Node::selfTestCallback, this,
                  std::placeholders::_1, std::placeholders::_2)
    );
    
    // Timer for reading IMU data
    auto period = std::chrono::microseconds(static_cast<int>(1e6 / imu_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&LSM9DS0Node::readAndPublish, this));
    
    RCLCPP_INFO(get_logger(), "LSM9DS0 IMU node initialized at %d Hz", imu_rate_hz_);
  }
  
  ~LSM9DS0Node() {
    if (i2c_fd_gyro_ >= 0) close(i2c_fd_gyro_);
    if (i2c_fd_mag_ >= 0) close(i2c_fd_mag_);
  }

private:
  void loadParameters() {
    i2c_bus_ = get_parameter("i2c_bus").as_int();
    addr_gyro_ = get_parameter("imu_addr_gyro").as_int();
    addr_mag_ = get_parameter("imu_addr_mag").as_int();
    imu_rate_hz_ = get_parameter("imu_rate_hz").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    
    // Load calibration
    accel_bias_ << 
        get_parameter("accel_bias_x").as_double(),
        get_parameter("accel_bias_y").as_double(),
        get_parameter("accel_bias_z").as_double();
    
    gyro_bias_ <<
        get_parameter("gyro_bias_x").as_double(),
        get_parameter("gyro_bias_y").as_double(),
        get_parameter("gyro_bias_z").as_double();
    
    accel_scale_ <<
        get_parameter("accel_scale_x").as_double(),
        get_parameter("accel_scale_y").as_double(),
        get_parameter("accel_scale_z").as_double();
    
    gyro_scale_ <<
        get_parameter("gyro_scale_x").as_double(),
        get_parameter("gyro_scale_y").as_double(),
        get_parameter("gyro_scale_z").as_double();
    
    // Load orientation
    auto rpy_deg = get_parameter("imu_orientation_rpy_deg").as_double_array();
    if (rpy_deg.size() == 3) {
      double roll = rpy_deg[0] * M_PI / 180.0;
      double pitch = rpy_deg[1] * M_PI / 180.0;
      double yaw = rpy_deg[2] * M_PI / 180.0;
      
      orientation_ = Eigen::Quaterniond(
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
      );
    } else {
      orientation_ = Eigen::Quaterniond::Identity();
    }
    
    // Noise
    accel_noise_ = get_parameter("accel_noise").as_double();
    gyro_noise_ = get_parameter("gyro_noise").as_double();
  }
  
  bool initializeI2C() {
    std::string i2c_device = "/dev/i2c-" + std::to_string(i2c_bus_);
    
    // Open gyro/accel device
    i2c_fd_gyro_ = open(i2c_device.c_str(), O_RDWR);
    if (i2c_fd_gyro_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open I2C bus %d: %s", 
                   i2c_bus_, strerror(errno));
      return false;
    }
    
    if (ioctl(i2c_fd_gyro_, I2C_SLAVE, addr_gyro_) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to set I2C slave address 0x%02X", addr_gyro_);
      return false;
    }
    
    // Open magnetometer device
    i2c_fd_mag_ = open(i2c_device.c_str(), O_RDWR);
    if (i2c_fd_mag_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open I2C bus for mag");
      return false;
    }
    
    if (ioctl(i2c_fd_mag_, I2C_SLAVE, addr_mag_) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to set I2C slave address 0x%02X", addr_mag_);
      return false;
    }
    
    RCLCPP_INFO(get_logger(), "I2C initialized on bus %d", i2c_bus_);
    return true;
  }
  
  bool verifyDevice() {
    uint8_t who_am_i_g = readRegister(i2c_fd_gyro_, WHO_AM_I_G);
    uint8_t who_am_i_xm = readRegister(i2c_fd_mag_, WHO_AM_I_XM);
    
    RCLCPP_INFO(get_logger(), "WHO_AM_I Gyro: 0x%02X (expected 0x%02X)", 
                who_am_i_g, WHO_AM_I_G_VALUE);
    RCLCPP_INFO(get_logger(), "WHO_AM_I XM: 0x%02X (expected 0x%02X)", 
                who_am_i_xm, WHO_AM_I_XM_VALUE);
    
    if (who_am_i_g != WHO_AM_I_G_VALUE) {
      RCLCPP_ERROR(get_logger(), "Gyro WHO_AM_I mismatch!");
      return false;
    }
    
    if (who_am_i_xm != WHO_AM_I_XM_VALUE) {
      RCLCPP_ERROR(get_logger(), "Accel/Mag WHO_AM_I mismatch!");
      return false;
    }
    
    return true;
  }
  
  void configureSensor() {
    // Configure gyroscope: 200 Hz ODR, all axes enabled
    writeRegister(i2c_fd_gyro_, CTRL_REG1_G, 0x6F);  // 200 Hz, normal mode, all axes
    writeRegister(i2c_fd_gyro_, CTRL_REG4_G, 0x00);  // ±245 dps
    
    // Configure accelerometer: 200 Hz ODR
    writeRegister(i2c_fd_mag_, CTRL_REG1_XM, 0x67);  // 200 Hz, all axes enabled
    writeRegister(i2c_fd_mag_, CTRL_REG2_XM, 0x00);  // ±2g
    
    RCLCPP_INFO(get_logger(), "Sensor configured");
  }
  
  void readAndPublish() {
    auto timestamp = now();
    
    // Read gyroscope (angular velocity)
    Eigen::Vector3d gyro_raw = readGyro();
    Eigen::Vector3d gyro = (gyro_raw.cwiseProduct(gyro_scale_) - gyro_bias_);
    
    // Read accelerometer (linear acceleration)
    Eigen::Vector3d accel_raw = readAccel();
    Eigen::Vector3d accel = (accel_raw.cwiseProduct(accel_scale_) - accel_bias_);
    
    // Apply orientation transform
    gyro = orientation_ * gyro;
    accel = orientation_ * accel;
    
    // Publish IMU message
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = timestamp;
    imu_msg.header.frame_id = frame_id_;
    
    imu_msg.angular_velocity.x = gyro.x();
    imu_msg.angular_velocity.y = gyro.y();
    imu_msg.angular_velocity.z = gyro.z();
    
    imu_msg.linear_acceleration.x = accel.x();
    imu_msg.linear_acceleration.y = accel.y();
    imu_msg.linear_acceleration.z = accel.z();
    
    // No orientation estimate from IMU alone
    imu_msg.orientation.w = 0.0;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation_covariance[0] = -1.0;  // No orientation data
    
    // Covariances
    double gyro_var = gyro_noise_ * gyro_noise_;
    double accel_var = accel_noise_ * accel_noise_;
    
    for (int i = 0; i < 3; ++i) {
      imu_msg.angular_velocity_covariance[i * 4] = gyro_var;
      imu_msg.linear_acceleration_covariance[i * 4] = accel_var;
    }
    
    imu_pub_->publish(imu_msg);
  }
  
  Eigen::Vector3d readGyro() {
    uint8_t data[6];
    readRegisters(i2c_fd_gyro_, OUT_X_L_G | 0x80, data, 6);
    
    int16_t x = (int16_t)((data[1] << 8) | data[0]);
    int16_t y = (int16_t)((data[3] << 8) | data[2]);
    int16_t z = (int16_t)((data[5] << 8) | data[4]);
    
    // Convert to rad/s (245 dps range)
    double scale = 245.0 / 32768.0 * M_PI / 180.0;
    return Eigen::Vector3d(x * scale, y * scale, z * scale);
  }
  
  Eigen::Vector3d readAccel() {
    uint8_t data[6];
    readRegisters(i2c_fd_mag_, OUT_X_L_A | 0x80, data, 6);
    
    int16_t x = (int16_t)((data[1] << 8) | data[0]);
    int16_t y = (int16_t)((data[3] << 8) | data[2]);
    int16_t z = (int16_t)((data[5] << 8) | data[4]);
    
    // Convert to m/s^2 (±2g range)
    double scale = 2.0 / 32768.0 * 9.81;
    return Eigen::Vector3d(x * scale, y * scale, z * scale);
  }
  
  uint8_t readRegister(int fd, uint8_t reg) {
    uint8_t value = 0;
    if (write(fd, &reg, 1) != 1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                           "Failed to write register address");
      return 0;
    }
    if (read(fd, &value, 1) != 1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Failed to read register");
      return 0;
    }
    return value;
  }
  
  void readRegisters(int fd, uint8_t reg, uint8_t* data, size_t len) {
    if (write(fd, &reg, 1) != 1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Failed to write register address");
      return;
    }
    if (read(fd, data, len) != static_cast<ssize_t>(len)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Failed to read registers");
    }
  }
  
  void writeRegister(int fd, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    if (write(fd, data, 2) != 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Failed to write register");
    }
  }
  
  void selfTestCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    (void)request;  // Unused
    
    bool success = verifyDevice();
    response->success = success;
    
    if (success) {
      response->message = "WHO_AM_I verification passed: Gyro=0xD4, XM=0x49";
    } else {
      response->message = "WHO_AM_I verification failed";
    }
  }
  
  // I2C file descriptors
  int i2c_fd_gyro_{-1};
  int i2c_fd_mag_{-1};
  
  // Parameters
  int i2c_bus_;
  int addr_gyro_;
  int addr_mag_;
  int imu_rate_hz_;
  std::string frame_id_;
  
  // Calibration
  Eigen::Vector3d accel_bias_;
  Eigen::Vector3d gyro_bias_;
  Eigen::Vector3d accel_scale_;
  Eigen::Vector3d gyro_scale_;
  Eigen::Quaterniond orientation_;
  
  double accel_noise_;
  double gyro_noise_;
  
  // ROS interfaces
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr self_test_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lsm9ds0

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lsm9ds0::LSM9DS0Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
