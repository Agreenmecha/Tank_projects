#!/bin/bash
# Wrapper to run teleop with proper permissions
cd /home/aaronjet/Tank_projects/scripts/odrive
sg dialout -c "./teleop_keyboard.py"

