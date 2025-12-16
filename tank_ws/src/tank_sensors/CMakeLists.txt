cmake_minimum_required(VERSION 3.8)
project(tank_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Install directories
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  src/pointcloud_frame_fixer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
