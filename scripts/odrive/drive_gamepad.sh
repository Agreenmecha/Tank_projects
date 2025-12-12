#!/bin/bash
# Gamepad teleop wrapper - handles permissions
cd "$(dirname "$0")"
sg dialout -c "python3 teleop_gamepad.py"

