#!/usr/bin/env python3
"""
Quick script to put ODrive motors in IDLE state
Use this to stop motors safely at any time
"""

import odrive
from odrive.enums import *
import sys

print("Connecting to ODrive...")
try:
    dev0 = odrive.find_any(timeout=5)
    if dev0 is None:
        print("❌ ODrive not found")
        sys.exit(1)
    
    print(f"✅ Connected: {dev0.serial_number}")
    print()
    
    # Stop motors first
    print("Stopping motors...")
    dev0.axis0.controller.input_vel = 0.0
    dev0.axis1.controller.input_vel = 0.0
    print("✅ Velocities set to 0")
    
    # Put in idle state
    print("Setting motors to IDLE state...")
    dev0.axis0.requested_state = AXIS_STATE_IDLE
    dev0.axis1.requested_state = AXIS_STATE_IDLE
    
    print("✅ Motors in IDLE state")
    print()
    print(f"Axis 0 state: {dev0.axis0.current_state} (1 = IDLE)")
    print(f"Axis 1 state: {dev0.axis1.current_state} (1 = IDLE)")
    print()
    
except Exception as e:
    print(f"❌ ERROR: {e}")
    sys.exit(1)

