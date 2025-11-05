/**
 * @file v4l2_stereo_node.cpp
 * @brief Stereo camera node using V4L2
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <thread>
#include <atomic>
#include <filesystem>

namespace stereo_camera {

class V4L2StereoNode : public rclcpp::Node {
public:
  V4L2StereoNode() : Node("v4l2_stereo_node") {
    // Declare parameters
    declare_parameter("left_device", "/dev/video0");
    declare_parameter("right_device", "/dev/video1");
    declare_parameter("width", 1280);
    declare_parameter("height", 720);
    declare_parameter("fps", 30);
    declare_parameter("left_camera_info_url", "");
    declare_parameter("right_camera_info_url", "");
    declare_parameter("auto_discover", true);
    
    // Load parameters
    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    fps_ = get_parameter("fps").as_int();
    bool auto_discover = get_parameter("auto_discover").as_bool();
    
    // Discover or use specified devices
    if (auto_discover) {
      discoverDevices();
    } else {
      left_device_ = get_parameter("left_device").as_string();
      right_device_ = get_parameter("right_device").as_string();
    }
    
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
    
    // Start capture threads
    running_ = true;
    left_thread_ = std::thread(&V4L2StereoNode::captureLeft, this);
    right_thread_ = std::thread(&V4L2StereoNode::captureRight, this);
    
    RCLCPP_INFO(get_logger(), "V4L2 stereo node started");
    RCLCPP_INFO(get_logger(), "Left device: %s, Right device: %s", 
                left_device_.c_str(), right_device_.c_str());
  }
  
  ~V4L2StereoNode() {
    running_ = false;
    if (left_thread_.joinable()) left_thread_.join();
    if (right_thread_.joinable()) right_thread_.join();
  }

private:
  void discoverDevices() {
    // Simple device discovery - check /dev/video* devices
    std::vector<std::string> devices;
    
    for (int i = 0; i < 10; ++i) {
      std::string device = "/dev/video" + std::to_string(i);
      if (std::filesystem::exists(device)) {
        devices.push_back(device);
      }
    }
    
    if (devices.size() >= 2) {
      left_device_ = devices[0];
      right_device_ = devices[1];
      RCLCPP_INFO(get_logger(), "Auto-discovered devices: %s, %s", 
                  left_device_.c_str(), right_device_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Could not auto-discover devices, using defaults");
      left_device_ = "/dev/video0";
      right_device_ = "/dev/video1";
    }
  }
  
  void captureLeft() {
    captureCamera(left_device_, left_image_pub_, left_info_pub_, 
                  left_info_manager_, "left");
  }
  
  void captureRight() {
    captureCamera(right_device_, right_image_pub_, right_info_pub_,
                  right_info_manager_, "right");
  }
  
  void captureCamera(
      const std::string& device,
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub,
      rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub,
      std::shared_ptr<camera_info_manager::CameraInfoManager> info_manager,
      const std::string& name) {
    
    // Open V4L2 device with OpenCV
    cv::VideoCapture cap(device, cv::CAP_V4L2);
    
    if (!cap.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s camera at %s", 
                   name.c_str(), device.c_str());
      return;
    }
    
    // Set camera parameters
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap.set(cv::CAP_PROP_FPS, fps_);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    
    RCLCPP_INFO(get_logger(), "%s camera opened: %dx%d @ %d fps", 
                name.c_str(), width_, height_, fps_);
    
    cv::Mat frame;
    while (running_ && rclcpp::ok()) {
      if (!cap.read(frame) || frame.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "%s camera: failed to read frame", name.c_str());
        continue;
      }
      
      // Convert to grayscale for stereo matching
      cv::Mat gray;
      if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      } else {
        gray = frame.clone();
      }
      
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
      info_msg.width = gray.cols;
      info_msg.height = gray.rows;
      info_pub->publish(info_msg);
    }
    
    cap.release();
  }
  
  std::string left_device_;
  std::string right_device_;
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
  auto node = std::make_shared<stereo_camera::V4L2StereoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
