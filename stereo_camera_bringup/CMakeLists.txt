cmake_minimum_required(VERSION 3.8)
project(stereo_camera_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Check for GStreamer (for Argus support)
find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(GSTREAMER gstreamer-1.0)
  pkg_check_modules(GSTREAMER_APP gstreamer-app-1.0)
endif()

# Argus stereo node
if(GSTREAMER_FOUND AND GSTREAMER_APP_FOUND)
  add_executable(argus_stereo_node src/argus_stereo_node.cpp)
  ament_target_dependencies(argus_stereo_node
    rclcpp
    sensor_msgs
    camera_info_manager
    image_transport
    cv_bridge
  )
  target_include_directories(argus_stereo_node PRIVATE
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(argus_stereo_node
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )
  
  install(TARGETS argus_stereo_node
    DESTINATION lib/${PROJECT_NAME}
  )
  
  message(STATUS "Building Argus stereo node with GStreamer support")
else()
  message(WARNING "GStreamer not found, Argus stereo node will not be built")
endif()

# V4L2 stereo node
add_executable(v4l2_stereo_node src/v4l2_stereo_node.cpp)
ament_target_dependencies(v4l2_stereo_node
  rclcpp
  sensor_msgs
  camera_info_manager
  image_transport
  cv_bridge
)
target_include_directories(v4l2_stereo_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(v4l2_stereo_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS v4l2_stereo_node
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

# Install scripts
install(PROGRAMS
  scripts/verify_sync.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
