# Rover SSH Bridge

This tool allows Cursor AI (and you) to remotely access your rover via SSH for executing commands, reading/writing files, and managing remote operations.

## Setup

### 1. Install Dependencies

```bash
pip3 install paramiko
```

### 2. Configure Connection

Edit `rover_config.json` with your rover's details:

```json
{
  "host": "192.168.1.100",      # Your rover's IP address
  "port": 22,                    # SSH port
  "username": "jetson",          # SSH username
  "password": "",                # Leave empty if using SSH keys
  "key_file": "~/.ssh/id_rsa",  # Path to your SSH private key
  "use_key": true,               # Set to false to use password
  "timeout": 10                  # Connection timeout in seconds
}
```

### 3. Set Up SSH Keys (Recommended)

If you haven't already set up SSH key authentication:

```bash
# Generate SSH key (if you don't have one)
ssh-keygen -t rsa -b 4096

# Copy key to rover
ssh-copy-id jetson@192.168.1.100
```

## Usage

### Test Connection

```bash
./rover_ssh_wrapper.sh test
```

### Execute Commands

```bash
# Simple command
./rover_ssh_wrapper.sh exec "ls -la"

# Check ROS environment
./rover_ssh_wrapper.sh exec "echo \$ROS_DOMAIN_ID"

# Start a ROS node
./rover_ssh_wrapper.sh exec "ros2 run my_package my_node"
```

### File Operations

```bash
# Read a remote file
./rover_ssh_wrapper.sh read /home/jetson/config.yaml

# Write content to a remote file
./rover_ssh_wrapper.sh write /home/jetson/test.txt "Hello from desktop!"

# List directory contents
./rover_ssh_wrapper.sh ls /home/jetson/ros2_ws/src

# Upload a file
./rover_ssh_wrapper.sh upload local_file.py /home/jetson/scripts/remote_file.py

# Download a file
./rover_ssh_wrapper.sh download /home/jetson/logs/latest.log ./rover_logs.txt
```

### Interactive Shell

```bash
./rover_ssh_wrapper.sh shell
```

## Using from Python

You can also import and use the bridge directly in Python scripts:

```python
from rover_ssh_bridge import RoverSSHBridge

# Initialize and connect
bridge = RoverSSHBridge()
bridge.connect()

# Run commands
exit_code, stdout, stderr = bridge.run_command("ros2 topic list")
print(stdout)

# Read files
config = bridge.read_file("/home/jetson/config.yaml")

# Write files
bridge.write_file("/home/jetson/test.txt", "Content here")

# Upload/download
bridge.upload_file("local.txt", "/home/jetson/remote.txt")
bridge.download_file("/home/jetson/log.txt", "local_log.txt")

# Cleanup
bridge.disconnect()
```

## For Cursor AI

When Cursor needs to access the rover, it can:

1. Execute commands: `./rover_ssh_wrapper.sh exec "command"`
2. Read rover files: `./rover_ssh_wrapper.sh read /path/to/file`
3. Write rover files: `./rover_ssh_wrapper.sh write /path/to/file "content"`
4. Upload scripts: `./rover_ssh_wrapper.sh upload local_script.py /home/jetson/scripts/`

## Examples

### Check Rover Status

```bash
./rover_ssh_wrapper.sh exec "uptime"
./rover_ssh_wrapper.sh exec "df -h"
./rover_ssh_wrapper.sh exec "ros2 node list"
```

### Deploy a Python Script

```bash
./rover_ssh_wrapper.sh upload ./my_script.py /home/jetson/scripts/my_script.py
./rover_ssh_wrapper.sh exec "chmod +x /home/jetson/scripts/my_script.py"
./rover_ssh_wrapper.sh exec "python3 /home/jetson/scripts/my_script.py"
```

### Read Logs

```bash
./rover_ssh_wrapper.sh read /var/log/syslog | tail -n 50
./rover_ssh_wrapper.sh download /home/jetson/ros2_ws/log/latest/latest.log ./rover_ros_log.txt
```

### Manage ROS 2 Nodes

```bash
# List active nodes
./rover_ssh_wrapper.sh exec "ros2 node list"

# Check topics
./rover_ssh_wrapper.sh exec "ros2 topic list"

# Echo a topic (with timeout)
./rover_ssh_wrapper.sh exec "timeout 5 ros2 topic echo /cmd_vel" --timeout 10
```

## Troubleshooting

### Connection Refused

- Verify rover is powered on and network is accessible
- Check IP address in `rover_config.json`
- Ensure SSH is running on rover: `systemctl status ssh`

### Authentication Failed

- If using keys: Verify key is added to rover's `~/.ssh/authorized_keys`
- If using password: Set `"use_key": false` and add password to config

### Permission Denied

- Check SSH username is correct
- Verify file permissions on the rover for read/write operations

### Timeout Issues

- Increase timeout value in `rover_config.json`
- For long-running commands, use `--timeout` parameter

