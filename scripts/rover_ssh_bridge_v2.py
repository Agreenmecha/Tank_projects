#!/usr/bin/env python3
"""
Enhanced SSH Bridge for Rover Access - Version 2
Adds: Session management, background processes, tmux integration, and daemon control
"""

import paramiko
import json
import os
import sys
import argparse
import time
from pathlib import Path
from typing import Optional, Tuple, List

class RoverSSHBridgeV2:
    def __init__(self, config_path: str = None):
        """Initialize SSH bridge with configuration."""
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), 'rover_config.json')
        
        self.config_path = config_path
        self.config = self._load_config()
        self.client = None
        self.sftp = None
        self.sessions = {}  # Track tmux sessions
        
    def _load_config(self) -> dict:
        """Load SSH configuration from JSON file."""
        try:
            with open(self.config_path, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"Config file not found: {self.config_path}")
            sys.exit(1)
    
    def connect(self) -> bool:
        """Establish SSH connection to the rover."""
        try:
            self.client = paramiko.SSHClient()
            self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            
            connect_kwargs = {
                'hostname': self.config['host'],
                'port': self.config['port'],
                'username': self.config['username'],
                'timeout': self.config.get('timeout', 10)
            }
            
            # Use key or password authentication
            if self.config.get('use_key', False) and self.config.get('key_file'):
                key_file = os.path.expanduser(self.config['key_file'])
                if os.path.exists(key_file):
                    connect_kwargs['key_filename'] = key_file
                else:
                    print(f"Warning: Key file not found: {key_file}")
            
            if self.config.get('password'):
                connect_kwargs['password'] = self.config['password']
                connect_kwargs['look_for_keys'] = False
                connect_kwargs['allow_agent'] = False
            
            self.client.connect(**connect_kwargs)
            self.sftp = self.client.open_sftp()
            print(f"✓ Connected to rover at {self.config['host']}")
            return True
            
        except Exception as e:
            print(f"✗ Connection error: {e}")
            return False
    
    def disconnect(self):
        """Close SSH connection."""
        if self.sftp:
            self.sftp.close()
        if self.client:
            self.client.close()
    
    def run_command(self, command: str, timeout: int = 30) -> Tuple[int, str, str]:
        """Execute a command on the rover. Returns: (exit_code, stdout, stderr)"""
        if not self.client:
            raise ConnectionError("Not connected to rover. Call connect() first.")
        
        try:
            stdin, stdout, stderr = self.client.exec_command(command, timeout=timeout)
            exit_code = stdout.channel.recv_exit_status()
            stdout_str = stdout.read().decode('utf-8')
            stderr_str = stderr.read().decode('utf-8')
            return exit_code, stdout_str, stderr_str
        except Exception as e:
            return -1, "", str(e)
    
    # ========== NEW: Session Management ==========
    
    def tmux_create_session(self, session_name: str, command: str = None, 
                           source_ros: bool = True) -> bool:
        """Create a persistent tmux session on the rover."""
        if not self.client:
            raise ConnectionError("Not connected to rover.")
        
        # Check if tmux is installed
        exit_code, _, _ = self.run_command("which tmux")
        if exit_code != 0:
            print("✗ tmux not installed on rover. Installing...")
            self.run_command("sudo apt-get update && sudo apt-get install -y tmux", timeout=60)
        
        # Check if session already exists
        exit_code, stdout, _ = self.run_command(f"tmux has-session -t {session_name} 2>/dev/null")
        if exit_code == 0:
            print(f"⚠ Session '{session_name}' already exists")
            return False
        
        # Build environment setup
        env_setup = ""
        if source_ros:
            env_setup = "export ROS_DOMAIN_ID=42 && source ~/Tank_projects/tank_ws/install/setup.bash && "
        
        # Create session with command
        if command:
            full_cmd = f"tmux new-session -d -s {session_name} '{env_setup}{command}'"
        else:
            full_cmd = f"tmux new-session -d -s {session_name}"
        
        exit_code, stdout, stderr = self.run_command(full_cmd)
        if exit_code == 0:
            print(f"✓ Created tmux session '{session_name}'")
            self.sessions[session_name] = True
            return True
        else:
            print(f"✗ Failed to create session: {stderr}")
            return False
    
    def tmux_list_sessions(self) -> List[str]:
        """List all active tmux sessions."""
        exit_code, stdout, _ = self.run_command("tmux list-sessions 2>/dev/null")
        if exit_code != 0:
            return []
        return [line.split(':')[0] for line in stdout.strip().split('\n') if line]
    
    def tmux_send_keys(self, session_name: str, keys: str) -> bool:
        """Send keys to a tmux session."""
        cmd = f"tmux send-keys -t {session_name} '{keys}' Enter"
        exit_code, _, _ = self.run_command(cmd)
        return exit_code == 0
    
    def tmux_get_output(self, session_name: str, lines: int = 50) -> str:
        """Capture output from a tmux session."""
        cmd = f"tmux capture-pane -t {session_name} -p -S -{lines}"
        _, stdout, _ = self.run_command(cmd)
        return stdout
    
    def tmux_kill_session(self, session_name: str) -> bool:
        """Kill a tmux session."""
        exit_code, _, _ = self.run_command(f"tmux kill-session -t {session_name}")
        if exit_code == 0:
            print(f"✓ Killed session '{session_name}'")
            self.sessions.pop(session_name, None)
            return True
        return False
    
    # ========== NEW: ROS-Specific Helpers ==========
    
    def ros_launch(self, session_name: str, package: str, launch_file: str, 
                   args: dict = None) -> bool:
        """Launch a ROS2 launch file in a persistent tmux session."""
        launch_args = ""
        if args:
            launch_args = " ".join([f"{k}:={v}" for k, v in args.items()])
        
        command = f"ros2 launch {package} {launch_file} {launch_args}"
        return self.tmux_create_session(session_name, command, source_ros=True)
    
    def ros_node_run(self, session_name: str, package: str, executable: str,
                     args: str = "") -> bool:
        """Run a ROS2 node in a persistent tmux session."""
        command = f"ros2 run {package} {executable} {args}"
        return self.tmux_create_session(session_name, command, source_ros=True)
    
    def ros_topic_list(self) -> List[str]:
        """List all active ROS topics."""
        _, stdout, _ = self.run_command(
            "export ROS_DOMAIN_ID=42 && source ~/Tank_projects/tank_ws/install/setup.bash && "
            "ros2 topic list"
        )
        return [line.strip() for line in stdout.split('\n') if line.strip()]
    
    def ros_topic_hz(self, topic: str) -> Optional[float]:
        """Get the publishing rate of a topic."""
        cmd = (
            f"export ROS_DOMAIN_ID=42 && source ~/Tank_projects/tank_ws/install/setup.bash && "
            f"timeout 3 ros2 topic hz {topic} 2>&1 | grep 'average rate' | awk '{{print $3}}'"
        )
        _, stdout, _ = self.run_command(cmd, timeout=10)
        try:
            return float(stdout.strip())
        except:
            return None
    
    def ros_node_list(self) -> List[str]:
        """List all active ROS nodes."""
        _, stdout, _ = self.run_command(
            "export ROS_DOMAIN_ID=42 && source ~/Tank_projects/tank_ws/install/setup.bash && "
            "ros2 node list"
        )
        return [line.strip() for line in stdout.split('\n') if line.strip()]
    
    # ========== NEW: Process Management ==========
    
    def get_process_list(self, filter_str: str = None) -> List[dict]:
        """Get list of running processes, optionally filtered."""
        cmd = "ps aux"
        if filter_str:
            cmd += f" | grep '{filter_str}' | grep -v grep"
        
        _, stdout, _ = self.run_command(cmd)
        processes = []
        for line in stdout.strip().split('\n'):
            if line:
                parts = line.split(None, 10)
                if len(parts) >= 11:
                    processes.append({
                        'user': parts[0],
                        'pid': parts[1],
                        'cpu': parts[2],
                        'mem': parts[3],
                        'command': parts[10]
                    })
        return processes
    
    def kill_process(self, pid: int, signal: str = 'TERM') -> bool:
        """Kill a process by PID."""
        exit_code, _, _ = self.run_command(f"kill -{signal} {pid}")
        return exit_code == 0
    
    def systemd_status(self, service: str) -> str:
        """Get status of a systemd service."""
        _, stdout, _ = self.run_command(f"systemctl status {service} 2>&1")
        return stdout
    
    # ========== NEW: System Monitoring ==========
    
    def get_system_stats(self) -> dict:
        """Get system resource usage."""
        stats = {}
        
        # CPU usage
        _, stdout, _ = self.run_command("top -bn1 | grep 'Cpu(s)' | awk '{print $2}'")
        stats['cpu_usage'] = stdout.strip()
        
        # Memory usage
        _, stdout, _ = self.run_command("free -h | grep Mem | awk '{print $3\"/\"$2}'")
        stats['memory'] = stdout.strip()
        
        # Disk usage
        _, stdout, _ = self.run_command("df -h / | tail -1 | awk '{print $3\"/\"$2\" (\"$5\")}}'")
        stats['disk'] = stdout.strip()
        
        # Uptime
        _, stdout, _ = self.run_command("uptime -p")
        stats['uptime'] = stdout.strip()
        
        # Temperature (Jetson-specific)
        _, stdout, _ = self.run_command("cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null")
        try:
            temp_c = int(stdout.strip()) / 1000.0
            stats['temperature'] = f"{temp_c:.1f}°C"
        except:
            stats['temperature'] = "N/A"
        
        return stats
    
    # ========== File Operations (from V1) ==========
    
    def read_file(self, remote_path: str) -> Optional[str]:
        """Read a file from the rover."""
        if not self.sftp:
            raise ConnectionError("Not connected to rover.")
        try:
            with self.sftp.file(remote_path, 'r') as f:
                return f.read().decode('utf-8')
        except:
            return None
    
    def write_file(self, remote_path: str, content: str) -> bool:
        """Write a file to the rover."""
        if not self.sftp:
            raise ConnectionError("Not connected to rover.")
        try:
            with self.sftp.file(remote_path, 'w') as f:
                f.write(content.encode('utf-8'))
            return True
        except:
            return False


def main():
    parser = argparse.ArgumentParser(description='Enhanced SSH Bridge for Rover Access V2')
    parser.add_argument('--config', '-c', help='Path to config file', default=None)
    
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Session management
    session_create = subparsers.add_parser('session-create', help='Create tmux session')
    session_create.add_argument('name', help='Session name')
    session_create.add_argument('--command', '-cmd', help='Command to run in session')
    session_create.add_argument('--no-ros', action='store_true', help='Don\'t source ROS')
    
    session_list = subparsers.add_parser('session-list', help='List tmux sessions')
    
    session_output = subparsers.add_parser('session-output', help='Get session output')
    session_output.add_argument('name', help='Session name')
    session_output.add_argument('--lines', '-n', type=int, default=50, help='Number of lines')
    
    session_kill = subparsers.add_parser('session-kill', help='Kill tmux session')
    session_kill.add_argument('name', help='Session name')
    
    # ROS operations
    ros_launch_cmd = subparsers.add_parser('ros-launch', help='Launch ROS2 launch file')
    ros_launch_cmd.add_argument('session', help='Session name')
    ros_launch_cmd.add_argument('package', help='Package name')
    ros_launch_cmd.add_argument('launch_file', help='Launch file')
    ros_launch_cmd.add_argument('--args', '-a', help='Launch arguments (key:=value key2:=value2)')
    
    ros_topics = subparsers.add_parser('ros-topics', help='List ROS topics')
    
    ros_nodes = subparsers.add_parser('ros-nodes', help='List ROS nodes')
    
    ros_hz = subparsers.add_parser('ros-hz', help='Get topic publish rate')
    ros_hz.add_argument('topic', help='Topic name')
    
    # Process management
    ps_list = subparsers.add_parser('ps', help='List processes')
    ps_list.add_argument('--filter', '-f', help='Filter string')
    
    kill = subparsers.add_parser('kill', help='Kill process')
    kill.add_argument('pid', type=int, help='Process ID')
    
    # System monitoring
    stats = subparsers.add_parser('stats', help='Get system stats')
    
    # Legacy commands
    exec_cmd = subparsers.add_parser('exec', help='Execute command')
    exec_cmd.add_argument('cmd', help='Command to execute')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return
    
    bridge = RoverSSHBridgeV2(args.config)
    if not bridge.connect():
        sys.exit(1)
    
    try:
        if args.command == 'session-create':
            bridge.tmux_create_session(args.name, args.command, not args.no_ros)
        
        elif args.command == 'session-list':
            sessions = bridge.tmux_list_sessions()
            if sessions:
                print("Active tmux sessions:")
                for s in sessions:
                    print(f"  - {s}")
            else:
                print("No active sessions")
        
        elif args.command == 'session-output':
            output = bridge.tmux_get_output(args.name, args.lines)
            print(output)
        
        elif args.command == 'session-kill':
            bridge.tmux_kill_session(args.name)
        
        elif args.command == 'ros-launch':
            launch_args = {}
            if args.args:
                for arg in args.args.split():
                    if ':=' in arg:
                        k, v = arg.split(':=', 1)
                        launch_args[k] = v
            bridge.ros_launch(args.session, args.package, args.launch_file, launch_args)
        
        elif args.command == 'ros-topics':
            topics = bridge.ros_topic_list()
            for topic in topics:
                print(topic)
        
        elif args.command == 'ros-nodes':
            nodes = bridge.ros_node_list()
            for node in nodes:
                print(node)
        
        elif args.command == 'ros-hz':
            hz = bridge.ros_topic_hz(args.topic)
            if hz:
                print(f"{args.topic}: {hz:.2f} Hz")
            else:
                print(f"Could not get rate for {args.topic}")
        
        elif args.command == 'ps':
            processes = bridge.get_process_list(args.filter)
            for p in processes:
                print(f"{p['pid']:>6} {p['cpu']:>5}% {p['mem']:>5}% {p['command']}")
        
        elif args.command == 'kill':
            if bridge.kill_process(args.pid):
                print(f"✓ Killed process {args.pid}")
            else:
                print(f"✗ Failed to kill process {args.pid}")
        
        elif args.command == 'stats':
            stats = bridge.get_system_stats()
            print("=== Rover System Stats ===")
            print(f"CPU:         {stats.get('cpu_usage', 'N/A')}")
            print(f"Memory:      {stats.get('memory', 'N/A')}")
            print(f"Disk:        {stats.get('disk', 'N/A')}")
            print(f"Temperature: {stats.get('temperature', 'N/A')}")
            print(f"Uptime:      {stats.get('uptime', 'N/A')}")
        
        elif args.command == 'exec':
            exit_code, stdout, stderr = bridge.run_command(args.cmd)
            print(stdout, end='')
            if stderr:
                print(stderr, end='', file=sys.stderr)
            sys.exit(exit_code)
    
    finally:
        bridge.disconnect()


if __name__ == '__main__':
    main()

