#ifndef IBVS_CONTROLLER_CPP__CAMERA_CONTROLLER_HPP_
#define IBVS_CONTROLLER_CPP__CAMERA_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <memory>
#include <vector>

namespace ibvs_controller
{

class CameraController : public rclcpp::Node
{
public:
  explicit CameraController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callbacks
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // Main control loop
  void controlLoop();

  // IBVS methods (to be implemented by students)
  std::vector<cv::KeyPoint> extractFeatures(const cv::Mat& image);
  cv::Mat computeInteractionMatrix(
    const std::vector<cv::KeyPoint>& features,
    const cv::Mat& depth_image);
  geometry_msgs::msg::Twist computeVelocityCommand(
    const std::vector<cv::KeyPoint>& current_features,
    const std::vector<cv::KeyPoint>& desired_features,
    const cv::Mat& interaction_matrix);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Timer for control loop
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Image data
  cv::Mat current_image_;
  cv::Mat current_depth_;
  cv::Mat camera_matrix_;
  bool camera_info_received_;

  // Feature detector (configurable)
  cv::Ptr<cv::Feature2D> feature_detector_;

  // Control parameters
  double lambda_gain_;  // IBVS control gain

  // Depth handling for Gazebo bug workaround
  float default_depth_;  // Default depth in meters
  float last_valid_depth_;  // Store last valid depth

  // Desired features (to be set by students)
  std::vector<cv::KeyPoint> desired_features_;
  cv::Mat desired_descriptors_;

  // Helper method for safe depth access
  float getDepthAtPixel(int x, int y);
};

// TODO: Students can define helper classes here
// For example: FeatureExtractor, IBVSController, etc.

}  // namespace ibvs_controller

#endif  // IBVS_CONTROLLER_CPP__CAMERA_CONTROLLER_HPP_