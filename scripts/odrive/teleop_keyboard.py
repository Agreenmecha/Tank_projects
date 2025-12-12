#!/usr/bin/env python3
"""
Simple Keyboard Teleop for Tank
Use arrow keys to drive the tank
"""

import odrive
from odrive.enums import *
import sys
import termios
import tty
import time

# Velocity limit (turns/s)
VEL_LIMIT = 10.0

def get_key():
    """Get a single keypress from the user"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def print_controls():
    """Print control instructions"""
    print("\n" + "="*50)
    print("Tank Keyboard Teleop")
    print("="*50)
    print("\nControls:")
    print("  ↑ : Forward")
    print("  ↓ : Backward")
    print("  ← : Turn Left")
    print("  → : Turn Right")
    print("  Space : Stop")
    print("  q : Quit")
    print(f"\nVelocity Limit: {VEL_LIMIT} turns/s")
    print("="*50 + "\n")

def main():
    print("Connecting to ODrive...")
    try:
        dev0 = odrive.find_any(timeout=5)
        if dev0 is None:
            print("❌ ODrive not found")
            sys.exit(1)
        
        print(f"✅ Connected: {dev0.serial_number}")
        print("Waiting for index search (5 seconds)...")
        time.sleep(5)
        
        # Check axis states before entering closed-loop
        print(f"Axis 0: State={dev0.axis0.current_state} Error={hex(dev0.axis0.error)}")
        print(f"Axis 1: State={dev0.axis1.current_state} Error={hex(dev0.axis1.error)}")
        
        # Enter closed-loop control
        print("Entering closed-loop control...")
        dev0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        dev0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1)
        
        # Verify both axes are in closed-loop
        print(f"Axis 0: State={dev0.axis0.current_state} (8=CLOSED_LOOP)")
        print(f"Axis 1: State={dev0.axis1.current_state} (8=CLOSED_LOOP)")
        if dev0.axis1.current_state != 8:
            print(f"⚠️  WARNING: Axis 1 not in closed-loop! Error: {hex(dev0.axis1.error)}")
        
        print_controls()
        
        # Main control loop
        left_vel = 0.0
        right_vel = 0.0
        
        while True:
            key = get_key()
            
            # Parse arrow keys (they send 3 characters: \x1b[A, \x1b[B, etc.)
            if key == '\x1b':  # Escape sequence
                key2 = sys.stdin.read(1)
                if key2 == '[':
                    key3 = sys.stdin.read(1)
                    
                    if key3 == 'A':  # Up arrow - Forward
                        left_vel = -VEL_LIMIT  # Reversed for axis 0
                        right_vel = VEL_LIMIT
                        print(f"↑ Forward: Axis0={left_vel:.1f} Axis1={right_vel:.1f}")
                        
                    elif key3 == 'B':  # Down arrow - Backward
                        left_vel = VEL_LIMIT   # Reversed for axis 0
                        right_vel = -VEL_LIMIT
                        print(f"↓ Backward: Axis0={left_vel:.1f} Axis1={right_vel:.1f}")
                        
                    elif key3 == 'D':  # Left arrow - Turn Left
                        left_vel = -VEL_LIMIT         # Fast forward (axis 0 reversed)
                        right_vel = -VEL_LIMIT * 0.5  # Slow/backward
                        print(f"← Turn Left: Axis0={left_vel:.1f} Axis1={right_vel:.1f}")
                        
                    elif key3 == 'C':  # Right arrow - Turn Right
                        left_vel = VEL_LIMIT * 0.5    # Slow/backward (axis 0 reversed)
                        right_vel = VEL_LIMIT         # Fast forward
                        print(f"→ Turn Right: Axis0={left_vel:.1f} Axis1={right_vel:.1f}")
            
            elif key == ' ':  # Space - Stop
                left_vel = 0.0
                right_vel = 0.0
                print("⏸  Stop")
                
            elif key == 'q':  # Quit
                print("\nQuitting...")
                break
            
            # Send velocities to ODrive
            dev0.axis0.controller.input_vel = left_vel
            dev0.axis1.controller.input_vel = right_vel
            
            # Debug: verify values were set
            actual_0 = dev0.axis0.controller.input_vel
            actual_1 = dev0.axis1.controller.input_vel
            if abs(actual_0 - left_vel) > 0.1 or abs(actual_1 - right_vel) > 0.1:
                print(f"⚠️  Mismatch! Sent: [{left_vel:.1f}, {right_vel:.1f}] Got: [{actual_0:.1f}, {actual_1:.1f}]")
        
        # Cleanup: Stop and idle motors
        print("Stopping motors...")
        dev0.axis0.controller.input_vel = 0.0
        dev0.axis1.controller.input_vel = 0.0
        time.sleep(0.5)
        
        dev0.axis0.requested_state = AXIS_STATE_IDLE
        dev0.axis1.requested_state = AXIS_STATE_IDLE
        print("✅ Motors set to IDLE")
        
    except KeyboardInterrupt:
        print("\n\n❗ Interrupted! Stopping motors...")
        try:
            dev0.axis0.controller.input_vel = 0.0
            dev0.axis1.controller.input_vel = 0.0
            dev0.axis0.requested_state = AXIS_STATE_IDLE
            dev0.axis1.requested_state = AXIS_STATE_IDLE
            print("✅ Motors stopped safely")
        except:
            pass
        sys.exit(0)
    
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        try:
            dev0.axis0.controller.input_vel = 0.0
            dev0.axis1.controller.input_vel = 0.0
            dev0.axis0.requested_state = AXIS_STATE_IDLE
            dev0.axis1.requested_state = AXIS_STATE_IDLE
        except:
            pass
        sys.exit(1)

if __name__ == '__main__':
    main()

