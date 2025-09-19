cmake_minimum_required(VERSION 3.8)
project(ibvs_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

# Add library for IBVS controller
add_library(ibvs_controller_lib
  src/camera_controller.cpp
)

target_include_directories(ibvs_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(ibvs_controller_lib
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "cv_bridge"
  "image_transport"
  "tf2"
  "tf2_ros"
)

target_link_libraries(ibvs_controller_lib
  ${OpenCV_LIBS}
)

add_executable(camera_controller_node src/camera_controller_node.cpp)
target_include_directories(camera_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(camera_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(camera_controller_node
  ibvs_controller_lib
)

ament_target_dependencies(
  camera_controller_node
  "rclcpp"
)

install(TARGETS camera_controller_node ibvs_controller_lib
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
