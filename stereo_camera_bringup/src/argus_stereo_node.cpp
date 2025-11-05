/**
 * @file argus_stereo_node.cpp
 * @brief Stereo camera node using nvarguscamerasrc (GStreamer/Argus)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <thread>
#include <atomic>

namespace stereo_camera {

class ArgusStereoNode : public rclcpp::Node {
public:
  ArgusStereoNode() : Node("argus_stereo_node") {
    // Declare parameters
    declare_parameter("sensor_id_left", 0);
    declare_parameter("sensor_id_right", 1);
    declare_parameter("width", 1280);
    declare_parameter("height", 720);
    declare_parameter("fps", 30);
    declare_parameter("left_camera_info_url", "");
    declare_parameter("right_camera_info_url", "");
    
    // Load parameters
    sensor_id_left_ = get_parameter("sensor_id_left").as_int();
    sensor_id_right_ = get_parameter("sensor_id_right").as_int();
    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    fps_ = get_parameter("fps").as_int();
    
    // Initialize camera info managers
    left_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, "left_camera", get_parameter("left_camera_info_url").as_string());
    right_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, "right_camera", get_parameter("right_camera_info_url").as_string());
    
    // Publishers
    left_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/stereo/left/image_raw", 10);
    right_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/stereo/right/image_raw", 10);
    left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "/stereo/left/camera_info", 10);
    right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "/stereo/right/camera_info", 10);
    
    // Initialize GStreamer
    gst_init(nullptr, nullptr);
    
    // Start capture threads
    running_ = true;
    left_thread_ = std::thread(&ArgusStereoNode::captureLeft, this);
    right_thread_ = std::thread(&ArgusStereoNode::captureRight, this);
    
    RCLCPP_INFO(get_logger(), "Argus stereo node started");
    RCLCPP_INFO(get_logger(), "Left sensor: %d, Right sensor: %d", 
                sensor_id_left_, sensor_id_right_);
  }
  
  ~ArgusStereoNode() {
    running_ = false;
    if (left_thread_.joinable()) left_thread_.join();
    if (right_thread_.joinable()) right_thread_.join();
  }

private:
  void captureLeft() {
    captureCamera(sensor_id_left_, left_image_pub_, left_info_pub_, 
                  left_info_manager_, "left");
  }
  
  void captureRight() {
    captureCamera(sensor_id_right_, right_image_pub_, right_info_pub_,
                  right_info_manager_, "right");
  }
  
  void captureCamera(
      int sensor_id,
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub,
      rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub,
      std::shared_ptr<camera_info_manager::CameraInfoManager> info_manager,
      const std::string& name) {
    
    // Build GStreamer pipeline
    std::stringstream pipeline_ss;
    pipeline_ss << "nvarguscamerasrc sensor-id=" << sensor_id 
                << " ! video/x-raw(memory:NVMM), width=" << width_ 
                << ", height=" << height_ 
                << ", framerate=" << fps_ << "/1"
                << " ! nvvidconv ! video/x-raw, format=BGRx"
                << " ! videoconvert ! video/x-raw, format=BGR"
                << " ! appsink name=sink";
    
    std::string pipeline_str = pipeline_ss.str();
    RCLCPP_INFO(get_logger(), "%s camera pipeline: %s", name.c_str(), pipeline_str.c_str());
    
    // Create OpenCV VideoCapture with GStreamer
    cv::VideoCapture cap(pipeline_str, cv::CAP_GSTREAMER);
    
    if (!cap.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s camera (sensor_id=%d)", 
                   name.c_str(), sensor_id);
      return;
    }
    
    RCLCPP_INFO(get_logger(), "%s camera opened successfully", name.c_str());
    
    cv::Mat frame;
    while (running_ && rclcpp::ok()) {
      if (!cap.read(frame) || frame.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "%s camera: failed to read frame", name.c_str());
        continue;
      }
      
      // Convert to grayscale for stereo matching
      cv::Mat gray;
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      
      // Get current timestamp (synchronized)
      auto timestamp = now();
      
      // Publish image
      auto image_msg = cv_bridge::CvImage(
          std_msgs::msg::Header(), "mono8", gray).toImageMsg();
      image_msg->header.stamp = timestamp;
      image_msg->header.frame_id = name + "_camera_optical_frame";
      image_pub->publish(*image_msg);
      
      // Publish camera info
      auto info_msg = info_manager->getCameraInfo();
      info_msg.header.stamp = timestamp;
      info_msg.header.frame_id = name + "_camera_optical_frame";
      info_msg.width = width_;
      info_msg.height = height_;
      info_pub->publish(info_msg);
    }
    
    cap.release();
  }
  
  int sensor_id_left_;
  int sensor_id_right_;
  int width_;
  int height_;
  int fps_;
  
  std::shared_ptr<camera_info_manager::CameraInfoManager> left_info_manager_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> right_info_manager_;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
  
  std::atomic<bool> running_;
  std::thread left_thread_;
  std::thread right_thread_;
};

}  // namespace stereo_camera

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<stereo_camera::ArgusStereoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
