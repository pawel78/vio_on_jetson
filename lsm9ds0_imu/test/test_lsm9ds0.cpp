/**
 * @file test_lsm9ds0.cpp
 * @brief Unit tests for LSM9DS0 IMU
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Simple test to ensure headers compile
TEST(LSM9DS0Test, BasicTest) {
  EXPECT_TRUE(true);
}

TEST(LSM9DS0Test, IMUMessageTest) {
  sensor_msgs::msg::Imu imu_msg;
  
  imu_msg.angular_velocity.x = 0.1;
  imu_msg.angular_velocity.y = 0.2;
  imu_msg.angular_velocity.z = 0.3;
  
  imu_msg.linear_acceleration.x = 0.0;
  imu_msg.linear_acceleration.y = 0.0;
  imu_msg.linear_acceleration.z = 9.81;
  
  EXPECT_DOUBLE_EQ(imu_msg.angular_velocity.x, 0.1);
  EXPECT_DOUBLE_EQ(imu_msg.linear_acceleration.z, 9.81);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
