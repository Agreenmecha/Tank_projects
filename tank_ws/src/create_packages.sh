#!/bin/bash
# Helper script to create basic package.xml files

create_package() {
    local pkg_name=$1
    local description=$2
    local deps=$3
    
    cat > ${pkg_name}/package.xml << PKGEOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>${pkg_name}</name>
  <version>0.1.0</version>
  <description>${description}</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

${deps}
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
PKGEOF

    cat > ${pkg_name}/CMakeLists.txt << CMAKEEOF
cmake_minimum_required(VERSION 3.8)
project(${pkg_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Install directories
install(DIRECTORY
  launch config
  DESTINATION share/\${PROJECT_NAME}/
)

ament_package()
CMAKEEOF
}

# Create packages
create_package "tank_localization" "State estimation: Point-LIO + GNSS fusion" "  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tank_msgs</depend>"

create_package "tank_perception" "Perception: LiDAR + camera terrain classification" "  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>pcl_ros</depend>
  <depend>tank_msgs</depend>"

create_package "tank_navigation" "Navigation: DWA planner + reverse mode" "  <depend>rclcpp</depend>
  <depend>nav2_core</depend>
  <depend>geometry_msgs</depend>
  <depend>tank_msgs</depend>"

create_package "tank_control" "Motor control: ODrive interface + safety" "  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>tank_msgs</depend>"

create_package "tank_sensors" "Sensor drivers and wrappers" "  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>"

create_package "tank_description" "URDF and robot description" "  <depend>urdf</depend>
  <depend>xacro</depend>"

create_package "tank_utils" "Utilities: logging, monitoring, tools" "  <depend>rclpy</depend>
  <depend>tank_msgs</depend>"

create_package "tank_odrive_can" "Optional: ODrive CAN Bus interface" "  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tank_msgs</depend>"

echo "Package files created!"
