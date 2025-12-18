#!/bin/bash
# Wrapper script for easy rover SSH access

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="$SCRIPT_DIR/rover_ssh_bridge.py"

# Make sure the Python script is executable
chmod +x "$PYTHON_SCRIPT"

# Pass all arguments to the Python script
python3 "$PYTHON_SCRIPT" "$@"

