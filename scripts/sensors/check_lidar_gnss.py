#!/usr/bin/env python3
"""
Comprehensive LiDAR and GNSS Sensor Diagnostic Script
Checks both dual LiDAR sensors and GNSS receiver
"""

import subprocess
import sys
import time
import os
import socket
import struct

# Colors for output
GREEN = '\033[0;32m'
RED = '\033[0;31m'
YELLOW = '\033[1;33m'
BLUE = '\033[0;34m'
CYAN = '\033[0;36m'
NC = '\033[0m'  # No Color

def print_header(text):
    """Print a section header"""
    print("\n" + "="*70)
    print(f"  {text}")
    print("="*70)

def print_status(message, status="info"):
    """Print status message with color"""
    if status == "ok":
        print(f"{GREEN}✓{NC} {message}")
    elif status == "error":
        print(f"{RED}✗{NC} {message}")
    elif status == "warning":
        print(f"{YELLOW}⚠{NC} {message}")
    elif status == "info":
        print(f"{CYAN}ℹ{NC} {message}")
    else:
        print(f"  {message}")

def check_network_interface():
    """Check if lidar network interface is configured"""
    print_header("NETWORK INTERFACE CHECK")
    
    try:
        result = subprocess.run(
            ['ip', 'addr', 'show', 'eno1'],
            capture_output=True,
            text=True,
            timeout=2
        )
        
        if '192.168.2.100' in result.stdout:
            print_status("Jetson IP configured: 192.168.2.100", "ok")
            return True
        else:
            print_status("Jetson IP not configured on eno1", "error")
            print("  Expected: 192.168.2.100/24")
            print("  Run: sudo nmcli connection up lidar-network")
            return False
    except Exception as e:
        print_status(f"Error checking network: {e}", "error")
        return False

def ping_host(ip, name):
    """Ping a host and return True if reachable"""
    try:
        result = subprocess.run(
            ['ping', '-c', '2', '-W', '1', ip],
            capture_output=True,
            timeout=5
        )
        if result.returncode == 0:
            print_status(f"{name} ({ip}) - Reachable", "ok")
            return True
        else:
            print_status(f"{name} ({ip}) - Not responding", "error")
            return False
    except Exception as e:
        print_status(f"{name} ({ip}) - Error: {e}", "error")
        return False

def check_lidar_network():
    """Check LiDAR network connectivity"""
    print_header("LIDAR NETWORK CONNECTIVITY")
    
    front_ok = ping_host('192.168.2.62', 'Front LiDAR')
    rear_ok = ping_host('192.168.2.63', 'Rear LiDAR')
    
    return front_ok, rear_ok

def check_lidar_udp_streams():
    """Check if LiDARs are streaming UDP data"""
    print_header("LIDAR UDP DATA STREAMS")
    
    # First, check network interface statistics
    try:
        result = subprocess.run(
            ['ip', '-s', 'link', 'show', 'eno1'],
            capture_output=True,
            text=True,
            timeout=2
        )
        
        if result.returncode == 0:
            # Extract RX statistics
            lines = result.stdout.split('\n')
            rx_line = None
            for i, line in enumerate(lines):
                if 'RX:' in line or 'RX packets' in line:
                    rx_line = lines[i+1] if i+1 < len(lines) else line
                    break
            
            if rx_line:
                # Parse packet count (second number is usually packets)
                parts = rx_line.split()
                if len(parts) >= 2:
                    try:
                        rx_packets = int(parts[1].replace(',', ''))
                        rx_bytes = int(parts[0].replace(',', ''))
                        print_status(f"Network interface receiving: {rx_packets:,} packets ({rx_bytes/1024/1024:.1f} MB)", "ok")
                        
                        # If we're receiving significant traffic, LiDARs are likely streaming
                        if rx_packets > 1000:
                            print_status("High network activity detected - LiDARs are likely streaming", "ok")
                            print("  Note: No process listening on ports 6201/6202")
                            print("  Start ROS2 nodes to receive data: ros2 launch tank_sensors dual_lidar.launch.py")
                            return True, True
                    except (ValueError, IndexError):
                        pass
    except Exception as e:
        pass
    
    # Try to capture packets with tcpdump (requires sudo)
    print("\nAttempting packet capture (requires sudo)...")
    print("  Checking for UDP traffic from LiDAR IPs...")
    
    try:
        # Use tcpdump to capture packets - check for any UDP from LiDAR IPs
        result = subprocess.run(
            ['sudo', 'timeout', '3', 'tcpdump', '-i', 'eno1', '-n', 
             'udp', 'and', '(host', '192.168.2.62', 'or', 'host', '192.168.2.63)', '-c', '20'],
            capture_output=True,
            text=True,
            timeout=8
        )
        
        output = result.stdout + result.stderr
        
        # Count packets from each LiDAR (more flexible pattern matching)
        front_count = len([l for l in output.split('\n') if '192.168.2.62' in l and 'UDP' in l])
        rear_count = len([l for l in output.split('\n') if '192.168.2.63' in l and 'UDP' in l])
        
        # Also check for port patterns
        if front_count == 0:
            front_count = output.count('192.168.2.62') + output.count('.62')
        if rear_count == 0:
            rear_count = output.count('192.168.2.63') + output.count('.63')
        
        if front_count > 0:
            print_status(f"Front LiDAR streaming: {front_count} UDP packets detected", "ok")
        else:
            print_status("Front LiDAR: No UDP packets detected in capture", "warning")
            print("  (But network activity suggests data may be flowing)")
        
        if rear_count > 0:
            print_status(f"Rear LiDAR streaming: {rear_count} UDP packets detected", "ok")
        else:
            print_status("Rear LiDAR: No UDP packets detected in capture", "warning")
            print("  (But network activity suggests data may be flowing)")
        
        return front_count > 0, rear_count > 0
        
    except subprocess.TimeoutExpired:
        print_status("Timeout waiting for packets", "warning")
        return None, None
    except FileNotFoundError:
        print_status("tcpdump not found - install with: sudo apt install tcpdump", "warning")
        return None, None
    except PermissionError:
        print_status("Permission denied - cannot capture packets without sudo", "warning")
        print("  Network statistics show traffic is being received")
        return None, None
    except Exception as e:
        print_status(f"Error capturing packets: {e}", "warning")
        return None, None

def check_ros2_environment():
    """Check if ROS2 environment is sourced"""
    print_header("ROS2 ENVIRONMENT CHECK")
    
    if 'ROS_DISTRO' not in os.environ:
        print_status("ROS2 environment not sourced", "error")
        print("  Source with: source ~/Tank_projects/tank_ws/install/setup.bash")
        return False
    
    print_status(f"ROS2 {os.environ.get('ROS_DISTRO', 'unknown')} environment active", "ok")
    return True

def check_lidar_ros2_nodes():
    """Check if LiDAR ROS2 nodes are running"""
    print_header("LIDAR ROS2 NODES")
    
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=5,
            env=os.environ
        )
        
        nodes = result.stdout.split('\n')
        front_running = any('lidar_front' in node for node in nodes)
        rear_running = any('lidar_rear' in node for node in nodes)
        
        if front_running:
            print_status("Front LiDAR node running", "ok")
        else:
            print_status("Front LiDAR node not running", "warning")
            print("  Start with: ros2 launch tank_sensors dual_lidar.launch.py")
        
        if rear_running:
            print_status("Rear LiDAR node running", "ok")
        else:
            print_status("Rear LiDAR node not running", "warning")
            print("  Start with: ros2 launch tank_sensors dual_lidar.launch.py")
        
        return front_running, rear_running
        
    except Exception as e:
        print_status(f"Error checking ROS2 nodes: {e}", "error")
        return False, False

def check_lidar_ros2_topics():
    """Check if LiDAR ROS2 topics are publishing"""
    print_header("LIDAR ROS2 TOPICS")
    
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=5,
            env=os.environ
        )
        
        topics = result.stdout.split('\n')
        
        front_pointcloud = any('/lidar_front/pointcloud' in topic for topic in topics)
        front_imu = any('/lidar_front/imu' in topic for topic in topics)
        rear_pointcloud = any('/lidar_rear/pointcloud' in topic for topic in topics)
        rear_imu = any('/lidar_rear/imu' in topic for topic in topics)
        
        print("\nFront LiDAR Topics:")
        if front_pointcloud:
            # Check publishing rate
            try:
                hz_result = subprocess.run(
                    ['timeout', '3', 'ros2', 'topic', 'hz', '/lidar_front/pointcloud'],
                    capture_output=True,
                    text=True,
                    timeout=5,
                    env=os.environ
                )
                if 'average rate' in hz_result.stdout:
                    hz = hz_result.stdout.split('average rate:')[1].split()[0]
                    print_status(f"  /lidar_front/pointcloud - Publishing at {hz} Hz", "ok")
                else:
                    print_status("  /lidar_front/pointcloud - Topic exists but not publishing", "warning")
            except:
                print_status("  /lidar_front/pointcloud - Topic exists", "ok")
        else:
            print_status("  /lidar_front/pointcloud - Not found", "error")
        
        if front_imu:
            print_status("  /lidar_front/imu - Available", "ok")
        else:
            print_status("  /lidar_front/imu - Not found", "warning")
        
        print("\nRear LiDAR Topics:")
        if rear_pointcloud:
            try:
                hz_result = subprocess.run(
                    ['timeout', '3', 'ros2', 'topic', 'hz', '/lidar_rear/pointcloud'],
                    capture_output=True,
                    text=True,
                    timeout=5,
                    env=os.environ
                )
                if 'average rate' in hz_result.stdout:
                    hz = hz_result.stdout.split('average rate:')[1].split()[0]
                    print_status(f"  /lidar_rear/pointcloud - Publishing at {hz} Hz", "ok")
                else:
                    print_status("  /lidar_rear/pointcloud - Topic exists but not publishing", "warning")
            except:
                print_status("  /lidar_rear/pointcloud - Topic exists", "ok")
        else:
            print_status("  /lidar_rear/pointcloud - Not found", "error")
        
        if rear_imu:
            print_status("  /lidar_rear/imu - Available", "ok")
        else:
            print_status("  /lidar_rear/imu - Not found", "warning")
        
        return front_pointcloud and rear_pointcloud
        
    except Exception as e:
        print_status(f"Error checking ROS2 topics: {e}", "error")
        return False

def check_gnss_device():
    """Check if GNSS device is connected"""
    print_header("GNSS DEVICE CHECK")
    
    # Check common GNSS device paths
    gnss_devices = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
    found_device = None
    
    for device in gnss_devices:
        if os.path.exists(device):
            print_status(f"GNSS device found: {device}", "ok")
            found_device = device
            
            # Check permissions
            if os.access(device, os.R_OK | os.W_OK):
                print_status(f"  Permissions: OK", "ok")
            else:
                print_status(f"  Permissions: Need to add user to dialout group", "warning")
                print(f"    Run: sudo usermod -a -G dialout $USER")
            
            break
    
    if not found_device:
        print_status("GNSS device not found", "error")
        print("  Check USB connection")
        print("  Check: ls -l /dev/ttyACM* /dev/ttyUSB*")
        print("  Check: dmesg | tail -20")
    
    return found_device

def check_gnss_ros2_node():
    """Check if GNSS ROS2 node is running"""
    print_header("GNSS ROS2 NODE")
    
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=5,
            env=os.environ
        )
        
        nodes = result.stdout.split('\n')
        gnss_running = any('gnss_node' in node for node in nodes)
        
        if gnss_running:
            print_status("GNSS node running", "ok")
        else:
            print_status("GNSS node not running", "warning")
            print("  Start with: ros2 launch tank_sensors gnss.launch.py")
        
        return gnss_running
        
    except Exception as e:
        print_status(f"Error checking ROS2 nodes: {e}", "error")
        return False

def check_gnss_ros2_topics():
    """Check if GNSS ROS2 topics are publishing"""
    print_header("GNSS ROS2 TOPICS")
    
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=5,
            env=os.environ
        )
        
        topics = result.stdout.split('\n')
        
        # Check for ublox topics
        ubx_topics = [t for t in topics if 'ubx' in t or 'gnss' in t]
        
        if ubx_topics:
            print_status(f"Found {len(ubx_topics)} GNSS topics", "ok")
            
            # Check key topics
            key_topics = {
                '/ubx_nav_pvt': 'Position/Velocity/Time',
                '/ubx_nav_hp_pos_llh': 'High-Precision Position',
                '/ubx_nav_dop': 'Dilution of Precision',
                '/gnss/fix': 'NavSatFix (standard)',
            }
            
            for topic, description in key_topics.items():
                if any(topic in t for t in topics):
                    try:
                        hz_result = subprocess.run(
                            ['timeout', '3', 'ros2', 'topic', 'hz', topic],
                            capture_output=True,
                            text=True,
                            timeout=5,
                            env=os.environ
                        )
                        if 'average rate' in hz_result.stdout:
                            hz = hz_result.stdout.split('average rate:')[1].split()[0]
                            print_status(f"  {topic} ({description}) - {hz} Hz", "ok")
                        else:
                            print_status(f"  {topic} ({description}) - Topic exists but not publishing", "warning")
                    except:
                        print_status(f"  {topic} ({description}) - Topic exists", "ok")
                else:
                    print_status(f"  {topic} ({description}) - Not found", "warning")
        else:
            print_status("No GNSS topics found", "error")
            return False
        
        return len(ubx_topics) > 0
        
    except Exception as e:
        print_status(f"Error checking ROS2 topics: {e}", "error")
        return False

def check_gnss_fix():
    """Check if GNSS has a fix"""
    print_header("GNSS FIX STATUS")
    
    try:
        # Try to get position data
        result = subprocess.run(
            ['timeout', '5', 'ros2', 'topic', 'echo', '/ubx_nav_hp_pos_llh', '--once'],
            capture_output=True,
            text=True,
            timeout=8,
            env=os.environ
        )
        
        if result.returncode == 0 and result.stdout:
            print_status("GNSS position data received", "ok")
            
            # Extract key values
            output = result.stdout
            if 'lat:' in output:
                lat_line = [l for l in output.split('\n') if 'lat:' in l][0]
                print(f"  {lat_line.strip()}")
            if 'lon:' in output:
                lon_line = [l for l in output.split('\n') if 'lon:' in l][0]
                print(f"  {lon_line.strip()}")
            if 'height:' in output:
                height_line = [l for l in output.split('\n') if 'height:' in l][0]
                print(f"  {height_line.strip()}")
            
            return True
        else:
            print_status("GNSS position data not available", "warning")
            print("  GNSS may still be acquiring satellites")
            print("  Ensure antenna has clear sky view")
            return False
            
    except Exception as e:
        print_status(f"Error checking GNSS fix: {e}", "warning")
        return False

def main():
    print("\n" + "╔" + "═"*68 + "╗")
    print("║" + " "*20 + "LIDAR & GNSS SENSOR DIAGNOSTICS" + " "*20 + "║")
    print("╚" + "═"*68 + "╝")
    
    # Check network
    network_ok = check_network_interface()
    
    # Check LiDAR network connectivity
    front_network_ok, rear_network_ok = check_lidar_network()
    
    # Check if ports are being listened on
    print_header("LIDAR PORT LISTENERS")
    try:
        result = subprocess.run(
            ['netstat', '-ulnp'],
            capture_output=True,
            text=True,
            timeout=2
        )
        listening_6201 = '6201' in result.stdout
        listening_6202 = '6202' in result.stdout
        
        if not listening_6201 and not listening_6202:
            # Try ss command as alternative
            result = subprocess.run(
                ['ss', '-ulnp'],
                capture_output=True,
                text=True,
                timeout=2
            )
            listening_6201 = '6201' in result.stdout
            listening_6202 = '6202' in result.stdout
        
        if listening_6201:
            print_status("Port 6201 (Front LiDAR): Process listening", "ok")
        else:
            print_status("Port 6201 (Front LiDAR): No listener", "warning")
            print("  Start: ros2 launch tank_sensors dual_lidar.launch.py")
        
        if listening_6202:
            print_status("Port 6202 (Rear LiDAR): Process listening", "ok")
        else:
            print_status("Port 6202 (Rear LiDAR): No listener", "warning")
            print("  Start: ros2 launch tank_sensors dual_lidar.launch.py")
    except:
        print_status("Could not check port listeners", "warning")
    
    # Check LiDAR UDP streams
    front_stream_ok, rear_stream_ok = check_lidar_udp_streams()
    
    # Check ROS2 environment
    ros2_ok = check_ros2_environment()
    
    lidar_nodes_ok = False
    lidar_topics_ok = False
    if ros2_ok:
        # Check LiDAR ROS2 nodes
        front_node_ok, rear_node_ok = check_lidar_ros2_nodes()
        lidar_nodes_ok = front_node_ok and rear_node_ok
        
        # Check LiDAR ROS2 topics
        lidar_topics_ok = check_lidar_ros2_topics()
    
    # Check GNSS device
    gnss_device_ok = check_gnss_device()
    
    gnss_node_ok = False
    gnss_topics_ok = False
    gnss_fix_ok = False
    if ros2_ok:
        # Check GNSS ROS2 node
        gnss_node_ok = check_gnss_ros2_node()
        
        # Check GNSS ROS2 topics
        gnss_topics_ok = check_gnss_ros2_topics()
        
        # Check GNSS fix
        if gnss_topics_ok:
            gnss_fix_ok = check_gnss_fix()
    
    # Summary
    print_header("DIAGNOSTIC SUMMARY")
    
    print("\n📡 LiDAR Sensors:")
    if front_network_ok:
        if front_stream_ok is True:
            print_status("  Front LiDAR: Streaming data detected", "ok")
        elif front_stream_ok is None:
            print_status("  Front LiDAR: Network OK (traffic detected, need listener)", "warning")
        else:
            print_status("  Front LiDAR: Network reachable", "ok")
    else:
        print_status("  Front LiDAR: Network Issues", "error")
    
    if rear_network_ok:
        if rear_stream_ok is True:
            print_status("  Rear LiDAR: Streaming data detected", "ok")
        elif rear_stream_ok is None:
            print_status("  Rear LiDAR: Network OK (traffic detected, need listener)", "warning")
        else:
            print_status("  Rear LiDAR: Network reachable", "ok")
    else:
        print_status("  Rear LiDAR: Network Issues", "error")
    
    if lidar_nodes_ok:
        print_status("  LiDAR ROS2 Nodes: Running", "ok")
    else:
        print_status("  LiDAR ROS2 Nodes: Not Running", "warning")
    
    if lidar_topics_ok:
        print_status("  LiDAR ROS2 Topics: Publishing", "ok")
    else:
        print_status("  LiDAR ROS2 Topics: Not Publishing", "warning")
    
    print("\n🛰️  GNSS Sensor:")
    if gnss_device_ok:
        print_status("  GNSS Device: Connected", "ok")
    else:
        print_status("  GNSS Device: Not Found", "error")
    
    if gnss_node_ok:
        print_status("  GNSS ROS2 Node: Running", "ok")
    else:
        print_status("  GNSS ROS2 Node: Not Running", "warning")
    
    if gnss_topics_ok:
        print_status("  GNSS ROS2 Topics: Publishing", "ok")
    else:
        print_status("  GNSS ROS2 Topics: Not Publishing", "warning")
    
    if gnss_fix_ok:
        print_status("  GNSS Fix: Acquired", "ok")
    else:
        print_status("  GNSS Fix: Not Acquired (may need sky view)", "warning")
    
    print("\n" + "="*70)
    
    all_ok = (
        front_network_ok and rear_network_ok and
        (front_stream_ok is True or front_stream_ok is None) and
        (rear_stream_ok is True or rear_stream_ok is None) and
        gnss_device_ok
    )
    
    if all_ok:
        print(f"  {GREEN}🎉 ALL SENSORS CONNECTED!{NC}")
        if lidar_nodes_ok and lidar_topics_ok and gnss_node_ok and gnss_topics_ok:
            print(f"  {GREEN}✅ ALL ROS2 NODES RUNNING!{NC}")
        else:
            print(f"  {YELLOW}⚠️  Start ROS2 nodes to begin data streaming{NC}")
    else:
        print(f"  {RED}⚠️  SOME ISSUES DETECTED - Review details above{NC}")
    
    print("="*70 + "\n")
    
    print("Next Steps:")
    if not lidar_nodes_ok:
        print("  • Start LiDAR nodes: ros2 launch tank_sensors dual_lidar.launch.py")
    if not gnss_node_ok:
        print("  • Start GNSS node: ros2 launch tank_sensors gnss.launch.py")
    if all_ok and (lidar_nodes_ok or gnss_node_ok):
        print("  • View data: rviz2")
        print("  • Monitor topics: ros2 topic list")
    print()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n❗ Interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

