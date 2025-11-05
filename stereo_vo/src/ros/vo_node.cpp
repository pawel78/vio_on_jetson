/**
 * @file vo_node.cpp
 * @brief Main ROS 2 node for stereo visual odometry
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include "core/feature_tracker.hpp"
#include "core/stereo_matcher.hpp"
#include "core/pose_estimator.hpp"
#include "core/imu_preintegrator.hpp"
#include "core/sliding_window.hpp"

namespace stereo_vo {
namespace ros {

class VONode : public rclcpp::Node {
public:
  VONode() : Node("vo_node") {
    // Declare parameters
    declareParameters();
    loadParameters();
    
    // Initialize algorithms
    initializeAlgorithms();
    
    // Setup publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/vo/odom", 10);
    quality_pub_ = create_publisher<std_msgs::msg::Float32>("/vo/quality", 10);
    
    if (publish_debug_) {
      tracks_pub_ = create_publisher<sensor_msgs::msg::Image>("/vo/tracks", 10);
    }
    
    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // Setup subscribers
    left_image_sub_.subscribe(this, "/stereo/left/image_raw");
    right_image_sub_.subscribe(this, "/stereo/right/image_raw");
    left_info_sub_.subscribe(this, "/stereo/left/camera_info");
    right_info_sub_.subscribe(this, "/stereo/right/camera_info");
    
    // Synchronize stereo pair
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), left_image_sub_, right_image_sub_, left_info_sub_, right_info_sub_
    );
    sync_->registerCallback(&VONode::stereoCallback, this);
    
    // IMU subscriber (if enabled)
    if (use_imu_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          "/imu/data", 100,
          std::bind(&VONode::imuCallback, this, std::placeholders::_1)
      );
    }
    
    // Reset service
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "/vo/reset",
        std::bind(&VONode::resetCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    RCLCPP_INFO(get_logger(), "Stereo VO node initialized");
  }

private:
  void declareParameters() {
    declare_parameter("use_imu", false);
    declare_parameter("max_features", 1500);
    declare_parameter("lk_levels", 3);
    declare_parameter("ransac_reproj_thresh_px", 2.0);
    declare_parameter("ba_window_size", 7);
    declare_parameter("stereo_method", "sgbm");
    declare_parameter("publish_debug_images", true);
    declare_parameter("frame_id_odom", "odom");
    declare_parameter("frame_id_base", "base_link");
  }
  
  void loadParameters() {
    use_imu_ = get_parameter("use_imu").as_bool();
    publish_debug_ = get_parameter("publish_debug_images").as_bool();
    frame_id_odom_ = get_parameter("frame_id_odom").as_string();
    frame_id_base_ = get_parameter("frame_id_base").as_string();
    
    // Feature tracker config
    tracker_config_.max_features = get_parameter("max_features").as_int();
    tracker_config_.lk_levels = get_parameter("lk_levels").as_int();
    tracker_config_.ransac_thresh = get_parameter("ransac_reproj_thresh_px").as_double();
    
    // Stereo matcher config
    matcher_config_.method = get_parameter("stereo_method").as_string();
    
    // Pose estimator config
    pose_config_.ransac_thresh = get_parameter("ransac_reproj_thresh_px").as_double();
    
    // Sliding window config
    window_config_.window_size = get_parameter("ba_window_size").as_int();
    window_config_.use_imu = use_imu_;
  }
  
  void initializeAlgorithms() {
    feature_tracker_ = std::make_unique<core::FeatureTracker>(tracker_config_);
    pose_estimator_ = std::make_unique<core::PoseEstimator>(pose_config_);
    sliding_window_ = std::make_unique<core::SlidingWindow>(window_config_);
    
    if (use_imu_) {
      imu_preintegrator_ = std::make_unique<core::IMUPreintegrator>();
    }
    
    // Stereo matcher will be initialized after receiving camera info
  }
  
  void stereoCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr& left_image,
      const sensor_msgs::msg::Image::ConstSharedPtr& right_image,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info) {
    
    auto start_time = std::chrono::steady_clock::now();
    
    // Initialize stereo matcher if needed
    if (!stereo_matcher_) {
      initializeStereoMatcher(left_info, right_info);
      if (!stereo_matcher_) {
        RCLCPP_WARN(get_logger(), "Failed to initialize stereo matcher");
        return;
      }
    }
    
    // Convert images
    cv_bridge::CvImagePtr left_cv, right_cv;
    try {
      left_cv = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::MONO8);
      right_cv = cv_bridge::toCvCopy(right_image, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    
    // Process frame
    processFrame(left_cv->image, right_cv->image, left_image->header.stamp);
    
    // Measure timing
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    frame_count_++;
    if (frame_count_ % 30 == 0) {
      RCLCPP_INFO(get_logger(), "VO processing time: %ld ms", duration.count());
    }
  }
  
  void initializeStereoMatcher(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info) {
    
    core::StereoMatcher::CameraParams params;
    
    // Left camera
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        params.K_left(i, j) = left_info->k[i * 3 + j];
      }
    }
    
    // Right camera
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        params.K_right(i, j) = right_info->k[i * 3 + j];
      }
    }
    
    // Distortion
    for (int i = 0; i < 5; ++i) {
      params.D_left(i) = left_info->d[i];
      params.D_right(i) = right_info->d[i];
    }
    
    // Baseline from P matrix
    params.baseline = std::abs(right_info->p[3] / right_info->p[0]);
    
    // Identity rotation (assuming rectified)
    params.R = Eigen::Matrix3d::Identity();
    params.t = Eigen::Vector3d(params.baseline, 0, 0);
    
    stereo_matcher_ = std::make_unique<core::StereoMatcher>(matcher_config_, params);
    camera_K_ = params.K_left;
    
    RCLCPP_INFO(get_logger(), "Stereo matcher initialized with baseline: %.3f m", params.baseline);
  }
  
  void processFrame(const cv::Mat& left_image, const cv::Mat& right_image, const rclcpp::Time& timestamp) {
    if (!initialized_) {
      initializeTracking(left_image, right_image, timestamp);
      return;
    }
    
    // Track features
    auto tracked = feature_tracker_->trackFeatures(prev_left_image_, left_image, prev_features_);
    
    if (tracked.size() < 20) {
      RCLCPP_WARN(get_logger(), "Too few tracked features: %zu", tracked.size());
      initializeTracking(left_image, right_image, timestamp);
      return;
    }
    
    // Compute disparity and triangulate
    cv::Mat disparity = stereo_matcher_->computeDisparity(left_image, right_image);
    
    std::vector<cv::Point2f> points_2d;
    std::vector<Eigen::Vector3d> points_3d;
    
    for (auto& feat : tracked) {
      points_2d.push_back(feat.point);
      feat.point_3d = cv::Point3f(0, 0, -1);  // Default invalid
      
      int x = static_cast<int>(feat.point.x);
      int y = static_cast<int>(feat.point.y);
      
      if (x >= 0 && x < disparity.cols && y >= 0 && y < disparity.rows) {
        float disp = disparity.at<short>(y, x) / 16.0f;
        Eigen::Vector3d pt3d = stereo_matcher_->triangulate(feat.point, disp);
        if (pt3d.z() > 0) {
          feat.point_3d = cv::Point3f(pt3d.x(), pt3d.y(), pt3d.z());
          points_3d.push_back(pt3d);
        }
      }
    }
    
    // Estimate pose
    if (points_3d.size() < 20) {
      RCLCPP_WARN(get_logger(), "Too few 3D points: %zu", points_3d.size());
      initializeTracking(left_image, right_image, timestamp);
      return;
    }
    
    core::Pose delta_pose = pose_estimator_->estimatePose(points_2d, points_3d, camera_K_, &current_pose_);
    
    if (!delta_pose.is_valid) {
      RCLCPP_WARN(get_logger(), "Pose estimation failed");
      initializeTracking(left_image, right_image, timestamp);
      return;
    }
    
    // Update current pose
    Eigen::Matrix4d T_current = current_pose_.toMatrix();
    Eigen::Matrix4d T_delta = delta_pose.toMatrix();
    Eigen::Matrix4d T_new = T_current * T_delta;
    current_pose_ = core::Pose::fromMatrix(T_new);
    current_pose_.timestamp = timestamp.seconds();
    
    // Publish odometry
    publishOdometry(timestamp);
    
    // Update for next frame
    prev_left_image_ = left_image.clone();
    prev_features_ = tracked;
    
    // Publish debug visualization
    if (publish_debug_) {
      publishTrackVisualization(left_image, tracked, timestamp);
    }
  }
  
  void initializeTracking(const cv::Mat& left_image, const cv::Mat& right_image, const rclcpp::Time& timestamp) {
    auto keypoints = feature_tracker_->detectFeatures(left_image);
    
    prev_features_.clear();
    for (size_t i = 0; i < keypoints.size(); ++i) {
      core::TrackedFeature feat;
      feat.point = keypoints[i].pt;
      feat.prev_point = feat.point;
      feat.track_id = i;
      feat.lifetime = 0;
      feat.is_valid = true;
      feat.quality = keypoints[i].response;
      prev_features_.push_back(feat);
    }
    
    prev_left_image_ = left_image.clone();
    
    if (!initialized_) {
      // Initialize pose at origin
      current_pose_.R = Eigen::Matrix3d::Identity();
      current_pose_.t = Eigen::Vector3d::Zero();
      current_pose_.is_valid = true;
      current_pose_.timestamp = timestamp.seconds();
      initialized_ = true;
      
      RCLCPP_INFO(get_logger(), "VO initialized with %zu features", prev_features_.size());
    }
  }
  
  void publishOdometry(const rclcpp::Time& timestamp) {
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = timestamp;
    odom_msg.header.frame_id = frame_id_odom_;
    odom_msg.child_frame_id = frame_id_base_;
    
    // Position
    odom_msg.pose.pose.position.x = current_pose_.t.x();
    odom_msg.pose.pose.position.y = current_pose_.t.y();
    odom_msg.pose.pose.position.z = current_pose_.t.z();
    
    // Orientation (convert rotation matrix to quaternion)
    Eigen::Quaterniond q(current_pose_.R);
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    
    // Publish odometry
    odom_pub_->publish(odom_msg);
    
    // Broadcast TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
    tf_msg.child_frame_id = frame_id_base_;
    tf_msg.transform.translation.x = current_pose_.t.x();
    tf_msg.transform.translation.y = current_pose_.t.y();
    tf_msg.transform.translation.z = current_pose_.t.z();
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
    
    tf_broadcaster_->sendTransform(tf_msg);
    
    // Publish quality metric
    auto quality_msg = std_msgs::msg::Float32();
    quality_msg.data = static_cast<float>(pose_estimator_->getInlierCount()) / 
                       static_cast<float>(prev_features_.size());
    quality_pub_->publish(quality_msg);
  }
  
  void publishTrackVisualization(
      const cv::Mat& image,
      const std::vector<core::TrackedFeature>& features,
      const rclcpp::Time& timestamp) {
    
    cv::Mat viz_image;
    cv::cvtColor(image, viz_image, cv::COLOR_GRAY2BGR);
    
    for (const auto& feat : features) {
      if (!feat.is_valid) continue;
      
      // Draw feature point
      cv::circle(viz_image, feat.point, 3, cv::Scalar(0, 255, 0), -1);
      
      // Draw track
      cv::line(viz_image, feat.prev_point, feat.point, cv::Scalar(255, 0, 0), 1);
    }
    
    // Publish as ROS image
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", viz_image).toImageMsg();
    msg->header.stamp = timestamp;
    tracks_pub_->publish(*msg);
  }
  
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    if (!imu_preintegrator_) return;
    
    core::IMUMeasurement measurement;
    measurement.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    measurement.accel = Eigen::Vector3d(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );
    measurement.gyro = Eigen::Vector3d(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z
    );
    
    imu_preintegrator_->addMeasurement(measurement);
  }
  
  void resetCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    (void)request;  // Unused
    
    initialized_ = false;
    feature_tracker_->reset();
    sliding_window_->reset();
    if (imu_preintegrator_) {
      imu_preintegrator_->reset();
    }
    
    current_pose_.R = Eigen::Matrix3d::Identity();
    current_pose_.t = Eigen::Vector3d::Zero();
    current_pose_.is_valid = false;
    
    response->success = true;
    response->message = "VO reset successfully";
    
    RCLCPP_INFO(get_logger(), "VO reset");
  }
  
  // Configuration
  bool use_imu_{false};
  bool publish_debug_{true};
  std::string frame_id_odom_{"odom"};
  std::string frame_id_base_{"base_link"};
  
  core::FeatureTracker::Config tracker_config_;
  core::StereoMatcher::Config matcher_config_;
  core::PoseEstimator::Config pose_config_;
  core::SlidingWindow::Config window_config_;
  
  // Algorithm components
  std::unique_ptr<core::FeatureTracker> feature_tracker_;
  std::unique_ptr<core::StereoMatcher> stereo_matcher_;
  std::unique_ptr<core::PoseEstimator> pose_estimator_;
  std::unique_ptr<core::SlidingWindow> sliding_window_;
  std::unique_ptr<core::IMUPreintegrator> imu_preintegrator_;
  
  // State
  bool initialized_{false};
  core::Pose current_pose_;
  cv::Mat prev_left_image_;
  std::vector<core::TrackedFeature> prev_features_;
  Eigen::Matrix3d camera_K_;
  int frame_count_{0};
  
  // ROS interfaces
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>;
  
  message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> right_info_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracks_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr quality_pub_;
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

}  // namespace ros
}  // namespace stereo_vo

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<stereo_vo::ros::VONode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
