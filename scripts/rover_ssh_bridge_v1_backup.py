#!/usr/bin/env python3
"""
SSH Bridge for Rover Access
Allows Cursor AI to execute commands, read/write files, and manage terminals on the rover via SSH.
"""

import paramiko
import json
import os
import sys
import argparse
from pathlib import Path
from typing import Optional, Tuple

class RoverSSHBridge:
    def __init__(self, config_path: str = None):
        """Initialize SSH bridge with configuration."""
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), 'rover_config.json')
        
        self.config_path = config_path
        self.config = self._load_config()
        self.client = None
        self.sftp = None
        
    def _load_config(self) -> dict:
        """Load SSH configuration from JSON file."""
        try:
            with open(self.config_path, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"Config file not found: {self.config_path}")
            print("Creating template config file...")
            self._create_template_config()
            sys.exit(1)
    
    def _create_template_config(self):
        """Create a template configuration file."""
        template = {
            "host": "192.168.2.100",
            "port": 22,
            "username": "aaronjet",
            "password": "your_password_here",
            "key_file": "~/.ssh/id_rsa",
            "use_key": False,
            "timeout": 10
        }
        with open(self.config_path, 'w') as f:
            json.dump(template, f, indent=2)
        print(f"Template config created at: {self.config_path}")
        print("Please edit the file with your rover's SSH details.")
    
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
                    print(f"Using SSH key: {key_file}")
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
            
        except paramiko.AuthenticationException:
            print("✗ Authentication failed. Check your credentials.")
            return False
        except paramiko.SSHException as e:
            print(f"✗ SSH connection failed: {e}")
            return False
        except Exception as e:
            print(f"✗ Connection error: {e}")
            return False
    
    def disconnect(self):
        """Close SSH connection."""
        if self.sftp:
            self.sftp.close()
        if self.client:
            self.client.close()
        print("✓ Disconnected from rover")
    
    def run_command(self, command: str, timeout: int = 30) -> Tuple[int, str, str]:
        """
        Execute a command on the rover.
        Returns: (exit_code, stdout, stderr)
        """
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
    
    def read_file(self, remote_path: str) -> Optional[str]:
        """Read a file from the rover."""
        if not self.sftp:
            raise ConnectionError("Not connected to rover. Call connect() first.")
        
        try:
            with self.sftp.file(remote_path, 'r') as f:
                content = f.read().decode('utf-8')
            print(f"✓ Read file: {remote_path}")
            return content
        except FileNotFoundError:
            print(f"✗ File not found: {remote_path}")
            return None
        except Exception as e:
            print(f"✗ Error reading file: {e}")
            return None
    
    def write_file(self, remote_path: str, content: str) -> bool:
        """Write a file to the rover."""
        if not self.sftp:
            raise ConnectionError("Not connected to rover. Call connect() first.")
        
        try:
            with self.sftp.file(remote_path, 'w') as f:
                f.write(content.encode('utf-8'))
            print(f"✓ Wrote file: {remote_path}")
            return True
        except Exception as e:
            print(f"✗ Error writing file: {e}")
            return False
    
    def list_dir(self, remote_path: str = '.') -> Optional[list]:
        """List directory contents on the rover."""
        if not self.sftp:
            raise ConnectionError("Not connected to rover. Call connect() first.")
        
        try:
            items = self.sftp.listdir_attr(remote_path)
            return [(item.filename, item.st_size, item.st_mode) for item in items]
        except Exception as e:
            print(f"✗ Error listing directory: {e}")
            return None
    
    def file_exists(self, remote_path: str) -> bool:
        """Check if a file exists on the rover."""
        if not self.sftp:
            raise ConnectionError("Not connected to rover. Call connect() first.")
        
        try:
            self.sftp.stat(remote_path)
            return True
        except FileNotFoundError:
            return False
    
    def upload_file(self, local_path: str, remote_path: str) -> bool:
        """Upload a file to the rover."""
        if not self.sftp:
            raise ConnectionError("Not connected to rover. Call connect() first.")
        
        try:
            self.sftp.put(local_path, remote_path)
            print(f"✓ Uploaded: {local_path} -> {remote_path}")
            return True
        except Exception as e:
            print(f"✗ Error uploading file: {e}")
            return False
    
    def download_file(self, remote_path: str, local_path: str) -> bool:
        """Download a file from the rover."""
        if not self.sftp:
            raise ConnectionError("Not connected to rover. Call connect() first.")
        
        try:
            self.sftp.get(remote_path, local_path)
            print(f"✓ Downloaded: {remote_path} -> {local_path}")
            return True
        except Exception as e:
            print(f"✗ Error downloading file: {e}")
            return False
    
    def interactive_shell(self):
        """Start an interactive shell session."""
        if not self.client:
            raise ConnectionError("Not connected to rover. Call connect() first.")
        
        print("\n=== Interactive Rover Shell ===")
        print("Type 'exit' to quit\n")
        
        channel = self.client.invoke_shell()
        
        try:
            while True:
                command = input("rover$ ")
                if command.strip().lower() == 'exit':
                    break
                
                channel.send(command + '\n')
                
                # Wait a bit for output
                import time
                time.sleep(0.5)
                
                if channel.recv_ready():
                    output = channel.recv(4096).decode('utf-8')
                    print(output, end='')
        
        except KeyboardInterrupt:
            print("\n\n✓ Shell closed")
        finally:
            channel.close()


def main():
    parser = argparse.ArgumentParser(description='SSH Bridge for Rover Access')
    parser.add_argument('--config', '-c', help='Path to config file', default=None)
    
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Run command
    cmd_parser = subparsers.add_parser('exec', help='Execute a command')
    cmd_parser.add_argument('cmd', help='Command to execute')
    cmd_parser.add_argument('--timeout', '-t', type=int, default=30, help='Command timeout')
    
    # Read file
    read_parser = subparsers.add_parser('read', help='Read a file')
    read_parser.add_argument('path', help='Remote file path')
    
    # Write file
    write_parser = subparsers.add_parser('write', help='Write a file')
    write_parser.add_argument('path', help='Remote file path')
    write_parser.add_argument('content', help='File content or @local_file to upload')
    
    # List directory
    ls_parser = subparsers.add_parser('ls', help='List directory')
    ls_parser.add_argument('path', nargs='?', default='.', help='Remote directory path')
    
    # Upload file
    upload_parser = subparsers.add_parser('upload', help='Upload a file')
    upload_parser.add_argument('local', help='Local file path')
    upload_parser.add_argument('remote', help='Remote file path')
    
    # Download file
    download_parser = subparsers.add_parser('download', help='Download a file')
    download_parser.add_argument('remote', help='Remote file path')
    download_parser.add_argument('local', help='Local file path')
    
    # Interactive shell
    subparsers.add_parser('shell', help='Start interactive shell')
    
    # Test connection
    subparsers.add_parser('test', help='Test connection')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return
    
    # Initialize bridge
    bridge = RoverSSHBridge(args.config)
    
    # Connect to rover
    if not bridge.connect():
        sys.exit(1)
    
    try:
        if args.command == 'exec':
            exit_code, stdout, stderr = bridge.run_command(args.cmd, args.timeout)
            print(stdout, end='')
            if stderr:
                print(stderr, end='', file=sys.stderr)
            sys.exit(exit_code)
        
        elif args.command == 'read':
            content = bridge.read_file(args.path)
            if content is not None:
                print(content)
        
        elif args.command == 'write':
            if args.content.startswith('@'):
                # Read from local file
                local_file = args.content[1:]
                with open(local_file, 'r') as f:
                    content = f.read()
            else:
                content = args.content
            bridge.write_file(args.path, content)
        
        elif args.command == 'ls':
            items = bridge.list_dir(args.path)
            if items:
                for name, size, mode in items:
                    print(f"{name:40s} {size:>10d} bytes")
        
        elif args.command == 'upload':
            bridge.upload_file(args.local, args.remote)
        
        elif args.command == 'download':
            bridge.download_file(args.remote, args.local)
        
        elif args.command == 'shell':
            bridge.interactive_shell()
        
        elif args.command == 'test':
            print("✓ Connection test successful!")
            exit_code, stdout, stderr = bridge.run_command('uname -a')
            print(f"Rover system: {stdout.strip()}")
    
    finally:
        bridge.disconnect()


if __name__ == '__main__':
    main()

