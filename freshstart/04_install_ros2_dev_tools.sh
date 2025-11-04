#!/bin/bash
################################################################################
# Install ROS 2 Development Tools and Dependencies
# Additional packages needed for Tank project
################################################################################

set -e

echo "Installing ROS 2 development tools and dependencies..."
echo ""

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "✗ ROS 2 not found. Please run 03_install_ros2.sh first."
    exit 1
fi

# Install colcon extensions
echo "Installing colcon build tools..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin

# Install ROS 2 message packages
echo ""
echo "Installing ROS 2 message packages..."
sudo apt install -y \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-std-msgs

# Install PCL and PCL-ROS (for LiDAR)
echo ""
echo "Installing PCL and PCL-ROS..."
sudo apt install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    libpcl-dev

# Install RTCM messages (for GNSS)
echo ""
echo "Installing RTCM messages (for GNSS)..."
sudo apt install -y \
    ros-humble-rtcm-msgs

# Install robot localization (for EKF fusion)
echo ""
echo "Installing robot_localization..."
sudo apt install -y \
    ros-humble-robot-localization

# Install navigation2 (for future path planning)
echo ""
echo "Installing Navigation2..."
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# Install TF2 and transforms
echo ""
echo "Installing TF2 packages..."
sudo apt install -y \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs

# Install RViz plugins
echo ""
echo "Installing RViz plugins..."
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rviz-default-plugins

# Install diagnostic tools
echo ""
echo "Installing diagnostic tools..."
sudo apt install -y \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-ros2bag \
    ros-humble-rosbag2-storage-default-plugins

# Set up colcon mixins
echo ""
echo "Setting up colcon mixins..."
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml || true
colcon mixin update default || true

# Configure ROS 2 domain ID
echo ""
echo "Configuring ROS 2 network settings..."
if ! grep -q 'ROS_DOMAIN_ID' ~/.bashrc; then
    echo '' >> ~/.bashrc
    echo '# ROS 2 Network Configuration' >> ~/.bashrc
    echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
    echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
    echo "Added ROS_DOMAIN_ID and RMW_IMPLEMENTATION to ~/.bashrc"
fi

echo ""
echo "✓ ROS 2 development tools and dependencies installed successfully"
echo ""
echo "Installed packages:"
echo "  • colcon build tools"
echo "  • sensor_msgs, geometry_msgs, nav_msgs"
echo "  • PCL and PCL-ROS (for LiDAR)"
echo "  • RTCM messages (for GNSS)"
echo "  • robot_localization (for EKF)"
echo "  • Navigation2 (for path planning)"
echo "  • TF2 (for transforms)"
echo "  • RViz2 and plugins"
echo "  • Diagnostic and debugging tools"

