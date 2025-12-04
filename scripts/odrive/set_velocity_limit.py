#!/usr/bin/env python3
"""
Set ODrive velocity limit
For 13:1 gearbox setup with 6000 RPM motors
"""

import odrive
from odrive.enums import *
import sys

print("=" * 60)
print("ODrive Velocity Limit Configuration")
print("=" * 60)
print()

print("Connecting to ODrive...")
try:
    dev0 = odrive.find_any(timeout=5)
    if dev0 is None:
        print("❌ ODrive not found")
        sys.exit(1)
    
    print(f"✅ Connected: {dev0.serial_number}")
    print()
    
    # Set velocity limit to 100 turns/s (6000 RPM with 13:1 gearbox)
    velocity_limit = 100.0  # turns/s
    
    print(f"Setting velocity limit to {velocity_limit} turns/s...")
    dev0.axis0.controller.config.vel_limit = velocity_limit
    dev0.axis1.controller.config.vel_limit = velocity_limit
    
    # Verify
    actual_0 = dev0.axis0.controller.config.vel_limit
    actual_1 = dev0.axis1.controller.config.vel_limit
    
    print(f"✅ Axis 0 velocity limit: {actual_0} turns/s")
    print(f"✅ Axis 1 velocity limit: {actual_1} turns/s")
    print()
    print("For 13:1 gearbox:")
    print(f"  Motor speed: {velocity_limit * 60:.0f} RPM")
    print(f"  Output speed: {velocity_limit * 60 / 13:.1f} RPM")
    print()
    
    # Save to ODrive
    print("Saving configuration to ODrive...")
    dev0.save_configuration()
    print("✅ Configuration saved (ODrive will reboot)")
    print()
    
except Exception as e:
    print(f"❌ ERROR: {e}")
    sys.exit(1)

