#!/bin/bash
# Setup script for Rover SSH Bridge

echo "=== Rover SSH Bridge Setup ==="
echo ""

# Check if paramiko is installed
if python3 -c "import paramiko" 2>/dev/null; then
    echo "✓ paramiko is already installed"
else
    echo "Installing paramiko..."
    
    # Try different installation methods
    if command -v pip3 &> /dev/null; then
        pip3 install --user paramiko
    elif command -v pip &> /dev/null; then
        pip install --user paramiko
    else
        echo "Installing via apt..."
        sudo apt update
        sudo apt install -y python3-paramiko
    fi
    
    # Verify installation
    if python3 -c "import paramiko" 2>/dev/null; then
        echo "✓ paramiko installed successfully"
    else
        echo "✗ Failed to install paramiko"
        echo "  Please install manually: sudo apt install python3-paramiko"
        exit 1
    fi
fi

echo ""
echo "=== Configuration ==="
echo "Edit the rover connection details:"
echo "  nano $PWD/rover_config.json"
echo ""
echo "Set your rover's:"
echo "  - IP address (host)"
echo "  - Username"
echo "  - SSH key path or password"
echo ""
echo "=== Testing ==="
echo "After configuration, test the connection:"
echo "  ./rover_ssh_wrapper.sh test"
echo ""
echo "Setup complete!"

