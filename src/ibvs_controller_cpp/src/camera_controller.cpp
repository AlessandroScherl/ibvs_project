#include "ibvs_controller_cpp/camera_controller.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace ibvs_controller
{

CameraController::CameraController(const rclcpp::NodeOptions & options)
: Node("camera_controller", options),
  camera_info_received_(false),
  lambda_gain_(0.1),
  default_depth_(1.5f),
  last_valid_depth_(1.5f)
{
  // Declare parameters
  this->declare_parameter("lambda_gain", 0.1);

  // Get parameters
  lambda_gain_ = this->get_parameter("lambda_gain").as_double();

  // TODO: Students should initialize their chosen feature detector here
  // Example: feature_detector_ = cv::ORB::create();
  // Note: feature_detector_ is declared but not initialized - this must be done.

  // Create subscribers
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/ibvs/image_raw", 10,
    std::bind(&CameraController::imageCallback, this, std::placeholders::_1));

  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/ibvs/depth/image_raw", 10,
    std::bind(&CameraController::depthCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/ibvs/camera_info", 10,
    std::bind(&CameraController::cameraInfoCallback, this, std::placeholders::_1));

  // Create publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/ibvs/cmd_vel", 10);

  // Create control timer (10 Hz)
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CameraController::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Camera Controller (C++) Node Started");
  RCLCPP_INFO(this->get_logger(), "Lambda gain: %.2f", lambda_gain_);
  RCLCPP_WARN(this->get_logger(), "Note: Using depth workaround for Gazebo Sim 8.x depth sensor limitations");
  RCLCPP_WARN(this->get_logger(), "TODO: Students must initialize feature detector!");
}

void CameraController::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    current_image_ = cv_ptr->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void CameraController::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::Mat depth_raw = cv_ptr->image;

    // Handle Gazebo depth sensor bug: replace -inf/inf with last valid or default depth
    current_depth_ = depth_raw.clone();

    // Count valid depths
    int valid_count = 0;
    float sum_valid = 0;

    for (int i = 0; i < depth_raw.rows; i++) {
      for (int j = 0; j < depth_raw.cols; j++) {
        float depth_value = depth_raw.at<float>(i, j);
        if (std::isfinite(depth_value) && depth_value > 0) {
          valid_count++;
          sum_valid += depth_value;
        } else {
          // Replace invalid with last valid depth
          current_depth_.at<float>(i, j) = last_valid_depth_;
        }
      }
    }

    // Update last valid depth if we have valid readings
    if (valid_count > 0) {
      last_valid_depth_ = sum_valid / valid_count;
    }

    // Log occasionally if many invalid values
    float invalid_percent = 100.0f * (1.0f - float(valid_count) / (depth_raw.rows * depth_raw.cols));
    if (invalid_percent > 10) {
      RCLCPP_DEBUG(this->get_logger(), "Depth: %.1f%% invalid, using %.2fm",
                   invalid_percent, last_valid_depth_);
    }

  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void CameraController::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!camera_info_received_) {
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        camera_matrix_.at<double>(i, j) = msg->k[i * 3 + j];
      }
    }
    camera_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera matrix received");
  }
}

void CameraController::controlLoop()
{
  if (current_image_.empty() || !camera_info_received_) {
    return;
  }

  // Extract features from current image
  auto current_features = extractFeatures(current_image_);

  // Create velocity command
  geometry_msgs::msg::Twist cmd;

  // TODO: Students should implement the actual IBVS control here
  // For now, just publish zero velocity as an example

  // Example implementation structure:
  // 1. Match current features with desired features
  // 2. Compute feature error
  // 3. Compute interaction matrix
  // 4. Apply control law: v = -lambda * L^+ * e

  cmd.linear.x = 0.0;   // Forward/backward
  cmd.linear.y = 0.0;   // Left/right
  cmd.linear.z = 0.0;   // Up/down
  cmd.angular.x = 0.0;  // Roll
  cmd.angular.y = 0.0;  // Pitch
  cmd.angular.z = 0.0;  // Yaw

  // Publish command
  cmd_vel_pub_->publish(cmd);
}

std::vector<cv::KeyPoint> CameraController::extractFeatures(const cv::Mat& image)
{
  std::vector<cv::KeyPoint> keypoints;

  // TODO: Students should implement feature extraction here
  // Steps:
  // 1. Convert image to grayscale if needed
  // 2. Initialize and configure feature detector (e.g., cv::ORB::create())
  // 3. Detect keypoints and compute descriptors
  // 4. Return keypoints for use in control loop

  // Example skeleton (students must complete):
  // cv::Mat gray;
  // if (image.channels() == 3) {
  //   cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  // }
  // feature_detector_->detect(gray, keypoints);

  return keypoints;
}

cv::Mat CameraController::computeInteractionMatrix(
  const std::vector<cv::KeyPoint>& features,
  const cv::Mat& depth_image)
{
  // TODO: Students should implement the interaction matrix computation
  // This is a placeholder that returns an empty matrix

  int num_features = features.size();
  cv::Mat L(2 * num_features, 6, CV_64F, cv::Scalar(0));

  // For each feature, compute its contribution to the interaction matrix
  // L = [ -1/Z   0   x/Z   xy   -(1+x²)   y  ]
  //     [  0   -1/Z  y/Z  1+y²    -xy    -x  ]

  return L;
}

geometry_msgs::msg::Twist CameraController::computeVelocityCommand(
  const std::vector<cv::KeyPoint>& current_features,
  const std::vector<cv::KeyPoint>& desired_features,
  const cv::Mat& interaction_matrix)
{
  geometry_msgs::msg::Twist cmd;

  // TODO: Students should implement the velocity computation
  // v = -lambda * L^+ * e
  // where L^+ is the pseudoinverse of L, and e is the feature error

  return cmd;
}

float CameraController::getDepthAtPixel(int x, int y)
{
  // Return default if no depth image
  if (current_depth_.empty()) {
    return default_depth_;
  }

  // Ensure coordinates are within bounds
  x = std::max(0, std::min(x, current_depth_.cols - 1));
  y = std::max(0, std::min(y, current_depth_.rows - 1));

  float depth_value = current_depth_.at<float>(y, x);

  // Return valid depth or fallback
  return std::isfinite(depth_value) ? depth_value : last_valid_depth_;
}

// TODO: Students can implement helper classes here if needed
// For example: FeatureExtractor, IBVSController, etc.

}  // namespace ibvs_controller
