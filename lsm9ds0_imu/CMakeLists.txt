cmake_minimum_required(VERSION 3.8)
project(lsm9ds0_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

# LSM9DS0 node
add_executable(lsm9ds0_node src/lsm9ds0_node.cpp)
ament_target_dependencies(lsm9ds0_node
  rclcpp
  sensor_msgs
  std_srvs
)
target_include_directories(lsm9ds0_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS lsm9ds0_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  ament_add_gtest(test_lsm9ds0 test/test_lsm9ds0.cpp)
  target_link_libraries(test_lsm9ds0)
  ament_target_dependencies(test_lsm9ds0 rclcpp sensor_msgs)
endif()

ament_package()
