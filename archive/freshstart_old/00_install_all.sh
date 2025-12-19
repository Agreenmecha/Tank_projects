#!/bin/bash
################################################################################
# Master Installation Script for Tank Autonomous Navigation Project
# Jetson Orin Nano - JetPack 6.2.1
################################################################################

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="$SCRIPT_DIR/installation.log"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "================================================================================"
echo "  Tank Autonomous Navigation Project - Fresh Installation"
echo "  Jetson Orin Nano - JetPack 6.2.1"
echo "================================================================================"
echo -e "${NC}"
echo ""
echo "This script will install:"
echo "  • System prerequisites and development tools"
echo "  • ODrive 0.5.4 motor controller software"
echo "  • ROS 2 Humble with development tools"
echo "  • Unitree L2 LiDAR drivers"
echo "  • u-blox ZED-F9P GNSS drivers"
echo "  • Project workspace and configuration"
echo ""
echo "Estimated time: 45-60 minutes"
echo "Log file: $LOG_FILE"
echo ""
read -p "Continue with installation? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Installation cancelled."
    exit 1
fi

# Start logging
exec > >(tee -a "$LOG_FILE")
exec 2>&1

echo ""
echo "Installation started at $(date)"
echo ""

# Function to run a script and check result
run_script() {
    local script=$1
    local description=$2
    
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}STEP: ${description}${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    
    if [ -f "$SCRIPT_DIR/$script" ]; then
        chmod +x "$SCRIPT_DIR/$script"
        if bash "$SCRIPT_DIR/$script"; then
            echo -e "${GREEN}✓ $description completed successfully${NC}"
            echo ""
        else
            echo -e "${RED}✗ $description failed!${NC}"
            echo -e "${RED}Check log: $LOG_FILE${NC}"
            exit 1
        fi
    else
        echo -e "${RED}✗ Script not found: $script${NC}"
        exit 1
    fi
}

# Run installation scripts in order
run_script "01_install_system_prerequisites.sh" "System Prerequisites"
run_script "02_install_odrive.sh" "ODrive Installation"
run_script "03_install_ros2.sh" "ROS 2 Humble Installation"
run_script "04_install_ros2_dev_tools.sh" "ROS 2 Development Tools"
run_script "05_install_sensor_drivers.sh" "Sensor Drivers"
run_script "06_setup_workspace.sh" "Project Workspace Setup"
run_script "07_configure_hardware.sh" "Hardware Configuration"

echo ""
echo -e "${GREEN}================================================================================${NC}"
echo -e "${GREEN}  Installation Complete!${NC}"
echo -e "${GREEN}================================================================================${NC}"
echo ""
echo "Next steps:"
echo ""
echo "  1. Restart your terminal or run:"
echo "     source ~/.bashrc"
echo ""
echo "  2. Connect hardware:"
echo "     • Dual Unitree L2 LiDARs (via network switch)"
echo "     • ZED-F9P GNSS (via USB)"
echo "     • ODrive motor controller (via USB)"
echo ""
echo "  3. Configure LiDARs (via serial):"
echo "     Front: IP 192.168.2.62, Target 192.168.2.100:6201"
echo "     Rear:  IP 192.168.2.63, Target 192.168.2.100:6202"
echo ""
echo "  4. Configure GNSS:"
echo "     python3 ~/Tank_projects/configure_gnss_ubx.py"
echo ""
echo "  5. Test systems:"
echo "     ~/Tank_projects/test_dual_lidar.sh"
echo "     ~/Tank_projects/test_gnss.sh"
echo ""
echo "Documentation:"
echo "  ~/Tank_projects/PROJECT_STATUS.md"
echo "  ~/Tank_projects/QUICK_REFERENCE.md"
echo "  ~/Tank_projects/LIDAR_SETUP_COMPLETE.md"
echo "  ~/Tank_projects/GNSS_SETUP_COMPLETE.md"
echo ""
echo "Installation log saved to: $LOG_FILE"
echo "Installation completed at $(date)"
echo ""

