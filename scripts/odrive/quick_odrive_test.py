#!/usr/bin/env python3
"""
Quick ODrive Connection Test
Minimal test to verify ODrive is accessible
"""

import odrive
from odrive.enums import *
import sys

print("=" * 60)
print("Quick ODrive Connection Test")
print("=" * 60)
print()

print("Searching for ODrive (10 second timeout)...")
try:
    dev0 = odrive.find_any(timeout=10)
    
    if dev0 is None:
        print("❌ ERROR: ODrive not found!")
        print()
        print("Troubleshooting:")
        print("  1. Check USB connection: lsusb | grep ODrive")
        print("  2. Fix permissions: sudo ./fix_odrive_permissions.sh")
        print("  3. Then run: newgrp dialout")
        print("  4. Or log out and back in")
        sys.exit(1)
    
    print(f"✅ Found ODrive!")
    print(f"   Serial: {dev0.serial_number}")
    print(f"   Firmware: v{dev0.fw_version_major}.{dev0.fw_version_minor}.{dev0.fw_version_revision}")
    print(f"   Hardware: v{dev0.hw_version_major}.{dev0.hw_version_minor}")
    print(f"   Bus Voltage: {dev0.vbus_voltage:.2f}V")
    print()
    
    print("Axis 0 Status:")
    print(f"   State: {dev0.axis0.current_state}")
    print(f"   Error: {hex(dev0.axis0.error)}")
    print(f"   Motor Error: {hex(dev0.axis0.motor.error)}")
    print(f"   Encoder Error: {hex(dev0.axis0.encoder.error)}")
    print(f"   Encoder Ready: {dev0.axis0.encoder.is_ready}")
    print()
    
    print("Axis 1 Status:")
    print(f"   State: {dev0.axis1.current_state}")
    print(f"   Error: {hex(dev0.axis1.error)}")
    print(f"   Motor Error: {hex(dev0.axis1.motor.error)}")
    print(f"   Encoder Error: {hex(dev0.axis1.encoder.error)}")
    print(f"   Encoder Ready: {dev0.axis1.encoder.is_ready}")
    print()
    
    print("=" * 60)
    print("✅ Connection successful!")
    print("=" * 60)
    print()
    print("Next steps:")
    print("  1. If encoders not ready, run calibration in odrivetool")
    print("  2. Run full test: python3 test_odrive_usb.py")
    print("  3. Then test ROS2: ./test_odrive_ros2.sh")
    print()
    
except Exception as e:
    print(f"❌ ERROR: {e}")
    print()
    print("This is likely a permissions issue.")
    print("Run: sudo ./fix_odrive_permissions.sh")
    print("Then: newgrp dialout")
    sys.exit(1)

