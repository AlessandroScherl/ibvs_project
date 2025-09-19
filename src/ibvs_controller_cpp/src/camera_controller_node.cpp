#include "ibvs_controller_cpp/camera_controller.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ibvs_controller::CameraController>();

  RCLCPP_INFO(node->get_logger(),
    "IBVS Camera Controller (C++) started. Waiting for images...");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
