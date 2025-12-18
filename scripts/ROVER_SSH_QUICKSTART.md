# Rover SSH Bridge - Quick Start

## ✓ Setup Complete!

The SSH bridge is configured and tested. Connection details:
- **Host**: 192.168.2.100 (tankviaxiaomi)
- **User**: aaronjet
- **Auth**: Password-based (configured in rover_config.json)

## Quick Commands

### From Terminal (Desktop)

```bash
cd /home/aaron/Tank_projects/scripts

# Execute commands on rover
./rover_ssh_wrapper.sh exec "command here"

# Read rover files
./rover_ssh_wrapper.sh read /path/to/file

# Write to rover
./rover_ssh_wrapper.sh write /path/to/file "content"

# List rover directories
./rover_ssh_wrapper.sh ls /home/aaronjet

# Upload files to rover
./rover_ssh_wrapper.sh upload local_file.py /home/aaronjet/remote_file.py

# Download from rover
./rover_ssh_wrapper.sh download /home/aaronjet/file.txt ./local_file.txt

# Interactive shell
./rover_ssh_wrapper.sh shell
```

## For Cursor AI

I (Cursor) can now directly access your rover using these commands:

```bash
# Check rover status
./rover_ssh_wrapper.sh exec "uptime && df -h"

# Check ROS nodes (with sourced environment)
./rover_ssh_wrapper.sh exec "source /opt/ros/humble/setup.bash && ros2 node list"

# Deploy scripts
./rover_ssh_wrapper.sh upload new_script.py /home/aaronjet/Tank_projects/scripts/new_script.py

# Read rover configs
./rover_ssh_wrapper.sh read /home/aaronjet/Tank_projects/config.yaml

# Monitor logs
./rover_ssh_wrapper.sh exec "tail -n 50 /var/log/syslog"
```

## Python API

```python
from rover_ssh_bridge import RoverSSHBridge

bridge = RoverSSHBridge()
bridge.connect()

# Execute
exit_code, stdout, stderr = bridge.run_command("ls -la")
print(stdout)

# Files
content = bridge.read_file("/home/aaronjet/config.yaml")
bridge.write_file("/home/aaronjet/test.txt", "content")

# Transfer
bridge.upload_file("local.py", "/home/aaronjet/remote.py")
bridge.download_file("/home/aaronjet/log.txt", "./rover.log")

bridge.disconnect()
```

## Common Tasks

### Deploy a ROS 2 Node

```bash
# Upload the node
./rover_ssh_wrapper.sh upload my_node.py /home/aaronjet/Tank_projects/tank_ws/src/my_package/my_node.py

# Build workspace
./rover_ssh_wrapper.sh exec "cd ~/Tank_projects/tank_ws && colcon build --packages-select my_package"

# Run the node
./rover_ssh_wrapper.sh exec "source ~/Tank_projects/tank_ws/install/setup.bash && ros2 run my_package my_node"
```

### Check Rover Environment

```bash
./rover_ssh_wrapper.sh exec "env | grep ROS"
./rover_ssh_wrapper.sh exec "printenv | grep -E 'ROS|AMENT'"
```

### Monitor System Resources

```bash
./rover_ssh_wrapper.sh exec "free -h && df -h && uptime"
```

### Access Tank Projects

```bash
# List project files
./rover_ssh_wrapper.sh ls /home/aaronjet/Tank_projects

# Read URDF
./rover_ssh_wrapper.sh read /home/aaronjet/Tank_projects/tank_ws/src/tank_description/urdf/tank.urdf

# Check scripts
./rover_ssh_wrapper.sh ls /home/aaronjet/Tank_projects/scripts/odrive
```

## Security Note

The password is stored in `rover_config.json`. Ensure this file is:
- Not committed to git (add to .gitignore)
- Readable only by you: `chmod 600 rover_config.json`

To set up key-based auth (more secure):

```bash
# Copy your desktop SSH key to rover
ssh-copy-id -i ~/.ssh/id_rsa.pub aaronjet@192.168.2.100

# Update rover_config.json
{
  "use_key": true,
  "password": ""
}
```

## Tested & Working ✓

- Connection to rover
- Command execution
- File read/write
- Directory listing
- File upload/download
- Access to Tank_projects workspace

