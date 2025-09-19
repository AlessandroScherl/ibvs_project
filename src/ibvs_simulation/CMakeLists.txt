cmake_minimum_required(VERSION 3.8)
project(ibvs_simulation)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch worlds models config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
