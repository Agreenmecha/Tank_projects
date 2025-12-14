#!/bin/bash
#
# Quick start script - just runs the launch file
# For a clean restart, use clean_start_viz.sh instead
#

cd "$(dirname "$0")/../../.."
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tankbot remote_viz.launch.py

