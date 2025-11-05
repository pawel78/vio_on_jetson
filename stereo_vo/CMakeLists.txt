cmake_minimum_required(VERSION 3.8)
project(stereo_vo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -DNDEBUG)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Optional: GTSAM for bundle adjustment
find_package(GTSAM QUIET)
if(GTSAM_FOUND)
  add_definitions(-DUSE_GTSAM)
  message(STATUS "GTSAM found, enabling BA optimization")
endif()

# Optional: CUDA for accelerated stereo matching
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  find_package(CUDA QUIET)
  if(CUDA_FOUND)
    add_definitions(-DUSE_CUDA)
    message(STATUS "CUDA found, enabling GPU acceleration")
  endif()
endif()

# Core library sources
set(CORE_SOURCES
  src/core/feature_tracker.cpp
  src/core/stereo_matcher.cpp
  src/core/pose_estimator.cpp
  src/core/imu_preintegrator.cpp
  src/core/sliding_window.cpp
)

# Core library
add_library(stereo_vo_core ${CORE_SOURCES})
target_include_directories(stereo_vo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(stereo_vo_core
  ${OpenCV_LIBRARIES}
)

if(GTSAM_FOUND)
  target_include_directories(stereo_vo_core PUBLIC ${GTSAM_INCLUDE_DIR})
  target_link_libraries(stereo_vo_core gtsam)
endif()

# VO ROS node
add_executable(vo_node src/ros/vo_node.cpp)
ament_target_dependencies(vo_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_transport
  cv_bridge
)
target_include_directories(vo_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(vo_node
  stereo_vo_core
  ${OpenCV_LIBRARIES}
)

# Install targets
install(TARGETS vo_node stereo_vo_core
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
  
  ament_add_gtest(test_feature_tracker test/test_feature_tracker.cpp)
  target_link_libraries(test_feature_tracker stereo_vo_core)
  target_include_directories(test_feature_tracker PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  
  ament_add_gtest(test_pose_estimator test/test_pose_estimator.cpp)
  target_link_libraries(test_pose_estimator stereo_vo_core)
  target_include_directories(test_pose_estimator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
endif()

ament_package()
