#!/usr/bin/env python3
"""
Gamepad Teleop for Tank
Use Flydigi Vader 4 Pro controller to drive the tank
"""

import odrive
from odrive.enums import *
import sys
import time

try:
    import pygame
except ImportError:
    print("❌ pygame not installed!")
    print("Install with: pip3 install pygame")
    sys.exit(1)

# Velocity limit (turns/s)
VEL_LIMIT = 80.0

# Deadzone for joysticks (prevent drift)
DEADZONE = 0.15

# Controller mapping for ShanWan Generic Controller
AXIS_LEFT_X = 0      # Left stick horizontal
AXIS_LEFT_Y = 1      # Left stick vertical (forward/backward)
AXIS_RIGHT_X = 2     # Right stick horizontal (turning)
AXIS_RIGHT_Y = 3     # Right stick vertical (don't use for driving)
BUTTON_SQUARE = 3    # Square button - quit
BUTTON_TRIANGLE = 0  # Triangle button
BUTTON_X = 2         # X button - emergency stop
BUTTON_O = 1         # O button

def apply_deadzone(value, deadzone=DEADZONE):
    """Apply deadzone to joystick input"""
    if abs(value) < deadzone:
        return 0.0
    # Scale the remaining range to 0-1
    sign = 1 if value > 0 else -1
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)

def print_controls():
    """Print control instructions"""
    print("\n" + "="*50)
    print("Tank Gamepad Teleop - ShanWan Controller")
    print("="*50)
    print("\nControls:")
    print("  Left Stick Y    : Forward/Backward")
    print("  Right Stick X   : Turn Left/Right")
    print("  X Button        : Emergency Stop")
    print("  Square Button   : Quit")
    print(f"\nVelocity Limit: {VEL_LIMIT} turns/s")
    print(f"Deadzone: {DEADZONE}")
    print("="*50 + "\n")

def main():
    # Initialize pygame and joystick
    # Set SDL to use evdev backend directly
    import os
    os.environ['SDL_JOYSTICK_DEVICE'] = '/dev/input/event*'
    
    pygame.init()
    pygame.joystick.init()
    
    # Give pygame time to detect devices
    time.sleep(0.5)
    pygame.event.pump()
    
    # Check for connected joysticks
    if pygame.joystick.get_count() == 0:
        print("❌ No gamepad detected!")
        print("\nDetected input devices:")
        import glob
        for dev in glob.glob('/dev/input/event*'):
            print(f"  {dev}")
        print("\nTroubleshooting:")
        print("  1. Check USB dongle is connected: lsusb | grep Xbox")
        print("  2. Turn on the controller")
        print("  3. Controller may need permissions:")
        print("     sudo chmod 666 /dev/input/event*")
        sys.exit(1)
    
    # Get the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    print(f"🎮 Connected: {joystick.get_name()}")
    print(f"   Axes: {joystick.get_numaxes()}")
    print(f"   Buttons: {joystick.get_numbuttons()}")
    print()
    
    # Connect to ODrive
    print("Connecting to ODrive...")
    try:
        dev0 = odrive.find_any(timeout=5)
        if dev0 is None:
            print("❌ ODrive not found")
            sys.exit(1)
        
        print(f"✅ Connected: {dev0.serial_number}")
        print("Waiting for index search (5 seconds)...")
        time.sleep(5)
        
        # Check axis states
        print(f"Axis 0: State={dev0.axis0.current_state} Error={hex(dev0.axis0.error)}")
        print(f"Axis 1: State={dev0.axis1.current_state} Error={hex(dev0.axis1.error)}")
        
        # Enter closed-loop control
        print("Entering closed-loop control...")
        dev0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        dev0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1)
        
        print(f"Axis 0: State={dev0.axis0.current_state} (8=CLOSED_LOOP)")
        print(f"Axis 1: State={dev0.axis1.current_state} (8=CLOSED_LOOP)")
        
        print_controls()
        
        # Main control loop
        clock = pygame.time.Clock()
        running = True
        emergency_stop = False
        
        while running:
            # Process pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BUTTON_X:
                        emergency_stop = True
                        print("🛑 EMERGENCY STOP")
                    elif event.button == BUTTON_SQUARE:
                        print("\nQuitting...")
                        running = False
                elif event.type == pygame.JOYBUTTONUP:
                    if event.button == BUTTON_X:
                        emergency_stop = False
                        print("▶️  Resume")
            
            # Read joystick axes
            if not emergency_stop:
                # Left stick Y - forward/backward (inverted because stick up = -1)
                forward = -apply_deadzone(joystick.get_axis(AXIS_LEFT_Y))
                
                # Right stick X - turning (positive = turn right)
                turn = apply_deadzone(joystick.get_axis(AXIS_RIGHT_X))
                
                # Differential drive kinematics
                # Left motor: forward - turn (reversed for axis 0)
                # Right motor: forward + turn
                left_vel = -(forward - turn) * VEL_LIMIT  # Negated for axis 0 reversal
                right_vel = (forward + turn) * VEL_LIMIT
                
                # Normalize to velocity limit while preserving turn radius
                # This maintains differential drive proportions at any speed
                max_vel = max(abs(left_vel), abs(right_vel))
                if max_vel > VEL_LIMIT:
                    scale = VEL_LIMIT / max_vel
                    left_vel *= scale
                    right_vel *= scale
                
            else:
                left_vel = 0.0
                right_vel = 0.0
            
            # Send to ODrive
            dev0.axis0.controller.input_vel = left_vel
            dev0.axis1.controller.input_vel = right_vel
            
            # Display status (only when moving)
            if abs(left_vel) > 0.1 or abs(right_vel) > 0.1:
                print(f"Axis0: {left_vel:6.2f} | Axis1: {right_vel:6.2f}    ", end='\r')
            
            # Run at 50Hz
            clock.tick(50)
        
        # Cleanup
        print("\nStopping motors...")
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
    
    finally:
        pygame.quit()

if __name__ == '__main__':
    main()

