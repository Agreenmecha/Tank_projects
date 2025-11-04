#!/bin/bash
################################################################################
# Cleanup Script
# Remove installed packages and reset for fresh installation
################################################################################

echo "⚠️  WARNING: This will remove installed packages and reset the environment"
echo ""
echo "This will:"
echo "  • Remove ROS 2 Humble packages"
echo "  • Remove ODrive installation"
echo "  • Clean workspace build artifacts"
echo "  • Remove custom configurations from ~/.bashrc"
echo ""
read -p "Continue with cleanup? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cleanup cancelled."
    exit 0
fi

echo ""
echo "Starting cleanup..."

# Remove ROS 2 packages
echo ""
echo "Removing ROS 2 packages..."
sudo apt remove -y ros-humble-* || true
sudo apt autoremove -y

# Remove ROS 2 repository
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# Remove ODrive
echo ""
echo "Removing ODrive..."
pip3 uninstall -y odrive || true

# Clean workspace
echo ""
echo "Cleaning workspace..."
if [ -d ~/Tank_projects/tank_ws ]; then
    rm -rf ~/Tank_projects/tank_ws/build
    rm -rf ~/Tank_projects/tank_ws/install
    rm -rf ~/Tank_projects/tank_ws/log
    echo "✓ Workspace build artifacts removed"
fi

# Backup bashrc
echo ""
echo "Backing up ~/.bashrc..."
cp ~/.bashrc ~/.bashrc.backup.$(date +%Y%m%d_%H%M%S)

# Remove ROS 2 sourcing from bashrc
echo ""
echo "Removing ROS 2 configurations from ~/.bashrc..."
sed -i '/# ROS 2 Humble/d' ~/.bashrc
sed -i '\|source /opt/ros/humble/setup.bash|d' ~/.bashrc
sed -i '/# Tank project workspace/d' ~/.bashrc
sed -i '\|source ~/Tank_projects/tank_ws/install/setup.bash|d' ~/.bashrc
sed -i '/# ROS 2 Network Configuration/d' ~/.bashrc
sed -i '/export ROS_DOMAIN_ID/d' ~/.bashrc
sed -i '/export RMW_IMPLEMENTATION/d' ~/.bashrc

echo ""
echo "✓ Cleanup complete"
echo ""
echo "Backup of ~/.bashrc saved to ~/.bashrc.backup.*"
echo ""
echo "To reinstall, run:"
echo "  cd ~/Tank_projects/freshstart"
echo "  ./00_install_all.sh"

