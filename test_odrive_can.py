#!/usr/bin/env python3
"""
ODrive CAN Test Script - Python version
More sophisticated testing with protocol parsing
"""

import socket
import struct
import sys
import time
import select

# CAN frame structure
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_MAX_DLEN = 8

class ODriveCANTest:
    def __init__(self, interface='can0'):
        self.interface = interface
        self.sock = None
        
    def setup_can_socket(self):
        """Create and bind CAN socket"""
        try:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.bind((self.interface,))
            print(f"✓ CAN socket bound to {self.interface}")
            return True
        except OSError as e:
            print(f"✗ Failed to bind CAN socket: {e}")
            print("  Make sure can0 is up: sudo ip link set up can0")
            return False
    
    def send_can_frame(self, can_id, data):
        """Send a CAN frame"""
        # Pack CAN frame: ID (4 bytes) + DLC (1 byte) + padding (3 bytes) + data (8 bytes)
        dlc = len(data)
        frame = struct.pack("=IB3x8s", can_id, dlc, data.ljust(8, b'\x00'))
        
        try:
            self.sock.send(frame)
            return True
        except OSError as e:
            print(f"✗ Failed to send frame: {e}")
            return False
    
    def receive_can_frame(self, timeout=1.0):
        """Receive CAN frame with timeout"""
        ready = select.select([self.sock], [], [], timeout)
        if ready[0]:
            frame, _ = self.sock.recvfrom(16)
            can_id, dlc = struct.unpack("=IB3x", frame[:8])
            data = frame[8:8+dlc]
            return can_id, data
        return None, None
    
    def listen_for_heartbeat(self, duration=5):
        """Listen for ODrive heartbeat messages"""
        print(f"\nListening for ODrive heartbeat ({duration}s)...")
        start_time = time.time()
        detected_axes = set()
        
        while time.time() - start_time < duration:
            can_id, data = self.receive_can_frame(timeout=0.1)
            
            if can_id is not None:
                # Check if this is a heartbeat message (0x001, 0x021, 0x041, etc.)
                cmd_id = can_id & 0x1F
                axis_id = (can_id >> 5) & 0x1F
                
                if cmd_id == 0x001:  # Heartbeat
                    detected_axes.add(axis_id)
                    
                    # Parse heartbeat data
                    axis_error, axis_state, controller_flags = struct.unpack('<IIB', data[:9])
                    
                    print(f"  Axis {axis_id} heartbeat: state={axis_state}, error={axis_error:#x}")
        
        if detected_axes:
            print(f"\n✓ Detected ODrive axes: {sorted(detected_axes)}")
            return True
        else:
            print("\n✗ No ODrive heartbeat detected")
            return False
    
    def send_velocity_command(self, axis_id, velocity, torque_ff=0.0):
        """Send velocity setpoint to ODrive"""
        # CAN ID = 0x00B | (axis_id << 5)
        can_id = 0x00B | (axis_id << 5)
        
        # Pack velocity and torque feedforward as floats
        data = struct.pack('<ff', velocity, torque_ff)
        
        print(f"Sending velocity command to axis {axis_id}: vel={velocity} rad/s")
        return self.send_can_frame(can_id, data)
    
    def request_encoder_estimates(self, axis_id):
        """Request encoder position and velocity"""
        # CAN ID = 0x009 | (axis_id << 5)
        can_id = 0x009 | (axis_id << 5)
        
        print(f"Requesting encoder estimates from axis {axis_id}...")
        
        # Send request (empty frame triggers response)
        self.send_can_frame(can_id, b'')
        
        # Wait for response
        time.sleep(0.1)
        can_id_resp, data = self.receive_can_frame(timeout=1.0)
        
        if can_id_resp and len(data) >= 8:
            position, velocity = struct.unpack('<ff', data[:8])
            print(f"  Position: {position:.3f} rad, Velocity: {velocity:.3f} rad/s")
            return position, velocity
        else:
            print("  ✗ No response received")
            return None, None
    
    def send_estop(self, axis_id):
        """Send emergency stop to axis"""
        # CAN ID = 0x002 | (axis_id << 5)
        can_id = 0x002 | (axis_id << 5)
        
        print(f"Sending E-STOP to axis {axis_id}")
        return self.send_can_frame(can_id, b'')
    
    def close(self):
        """Close CAN socket"""
        if self.sock:
            self.sock.close()


def main():
    print("=" * 50)
    print("ODrive CAN Test Script (Python)")
    print("=" * 50)
    
    tester = ODriveCANTest('can0')
    
    # Setup CAN socket
    if not tester.setup_can_socket():
        sys.exit(1)
    
    # Test 1: Listen for heartbeat
    print("\n[Test 1] ODrive Detection")
    odrive_detected = tester.listen_for_heartbeat(duration=5)
    
    if not odrive_detected:
        print("\n⚠ ODrive not detected. Make sure:")
        print("  1. ODrive is connected to CAN Pal")
        print("  2. CAN is enabled on ODrive:")
        print("     odrv0.config.enable_can_a = True")
        print("     odrv0.axis0.config.can_node_id = 0")
        print("     odrv0.can.config.baud_rate = 500000")
        print("  3. 120Ω termination is enabled on CAN Pal")
        tester.close()
        sys.exit(1)
    
    # Test 2: Request encoder estimates
    print("\n[Test 2] Encoder Feedback")
    for axis_id in [0, 1]:
        pos, vel = tester.request_encoder_estimates(axis_id)
        time.sleep(0.5)
    
    # Test 3: Send velocity command (optional)
    print("\n[Test 3] Velocity Command Test")
    print("⚠ WARNING: Motor will move if enabled!")
    response = input("Send velocity command to axis 0? (y/N): ")
    
    if response.lower() == 'y':
        # Send 0.5 rad/s command
        tester.send_velocity_command(axis_id=0, velocity=0.5, torque_ff=0.0)
        print("Command sent. Motor should rotate slowly if in CLOSED_LOOP_CONTROL state.")
        
        # Wait and check encoder
        time.sleep(2)
        pos, vel = tester.request_encoder_estimates(0)
        
        if vel and abs(vel) > 0.1:
            print(f"✓ Motor responding (velocity = {vel:.2f} rad/s)")
        else:
            print("⚠ Motor not responding. Check:")
            print("  - Axis state (should be CLOSED_LOOP_CONTROL)")
            print("  - Motor enabled")
            print("  - No errors")
        
        # Stop motor
        tester.send_velocity_command(axis_id=0, velocity=0.0)
        print("Motor stopped")
    
    # Test 4: E-stop test (optional)
    print("\n[Test 4] E-STOP Test")
    response = input("Test E-STOP command? (y/N): ")
    
    if response.lower() == 'y':
        tester.send_estop(axis_id=0)
        tester.send_estop(axis_id=1)
        print("✓ E-STOP commands sent")
    
    print("\n" + "=" * 50)
    print("SUMMARY:")
    print("✓ CAN communication working")
    print("✓ ODrive responding on CAN bus")
    print("\nREADY to implement tank_odrive_can ROS2 package!")
    print("See workspace_structure.md for implementation details")
    print("=" * 50)
    
    tester.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

