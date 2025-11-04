#!/bin/bash
################################################################################
# Install ROS 2 Humble
# For Ubuntu 22.04 (Jetson JetPack 6.2.1)
################################################################################

set -e

echo "Installing ROS 2 Humble..."
echo ""

# Check if ROS 2 is already installed
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "⚠ ROS 2 Humble appears to be already installed"
    read -p "Reinstall? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Skipping ROS 2 installation"
        exit 0
    fi
fi

# Ensure locale supports UTF-8
echo "Setting up locale..."
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
echo ""
echo "Adding ROS 2 repository..."
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists
echo ""
echo "Updating package lists..."
sudo apt update

# Upgrade existing packages (optional but recommended)
echo ""
echo "Upgrading system packages..."
sudo apt upgrade -y

# Install ROS 2 Humble Desktop (includes RViz, demos, tutorials)
echo ""
echo "Installing ROS 2 Humble (this may take 10-15 minutes)..."
sudo apt install -y ros-humble-desktop

# Install ROS 2 development tools
echo ""
echo "Installing ROS 2 development packages..."
sudo apt install -y \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialize rosdep
echo ""
echo "Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Add ROS 2 sourcing to bashrc
echo ""
echo "Configuring environment..."
if ! grep -q '/opt/ros/humble/setup.bash' ~/.bashrc; then
    echo '' >> ~/.bashrc
    echo '# ROS 2 Humble' >> ~/.bashrc
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
    echo "Added ROS 2 Humble to ~/.bashrc"
fi

# Source ROS 2 for current session
source /opt/ros/humble/setup.bash

# Verify installation
echo ""
echo "Verifying ROS 2 installation..."
if command -v ros2 &> /dev/null; then
    echo "✓ ROS 2 installed successfully"
    ROS2_VERSION=$(ros2 --version 2>&1 | head -1)
    echo "  $ROS2_VERSION"
else
    echo "✗ ROS 2 installation verification failed"
    exit 1
fi

echo ""
echo "✓ ROS 2 Humble installation complete"
echo ""
echo "Note: Open a new terminal or run 'source ~/.bashrc' to use ROS 2 commands"

