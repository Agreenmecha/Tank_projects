#!/usr/bin/env python3
"""
Set ODrive velocity ramp rate
Allows testing different acceleration profiles
"""

import odrive
from odrive.enums import *
import sys

def set_ramp_rate(rate):
    """Set velocity ramp rate for both axes."""
    if rate < 0.1 or rate > 100:
        print(f"❌ ERROR: Ramp rate {rate} out of safe range (0.1-100)")
        return False
    
    print("Connecting to ODrive...")
    try:
        dev0 = odrive.find_any(timeout=5)
        if dev0 is None:
            print("❌ ODrive not found")
            return False
        
        print(f"✅ Connected: {dev0.serial_number}")
        print()
        
        # Set ramp rate
        print(f"Setting velocity ramp rate to {rate} turns/s²...")
        dev0.axis0.controller.config.vel_ramp_rate = rate
        dev0.axis1.controller.config.vel_ramp_rate = rate
        
        # Verify
        actual_0 = dev0.axis0.controller.config.vel_ramp_rate
        actual_1 = dev0.axis1.controller.config.vel_ramp_rate
        
        print(f"✅ Axis 0 ramp rate: {actual_0} turns/s²")
        print(f"✅ Axis 1 ramp rate: {actual_1} turns/s²")
        print()
        
        if rate >= 3 and rate <= 15:
            print("✓ Recommended range (3-15)")
        elif rate < 3:
            print("⚠️  Low ramp rate - very gradual acceleration")
        else:
            print("⚠️  High ramp rate - faster acceleration")
        
        print()
        return True
        
    except Exception as e:
        print(f"❌ ERROR: {e}")
        return False


if __name__ == '__main__':
    print("=" * 60)
    print("ODrive Velocity Ramp Rate Configuration")
    print("=" * 60)
    print()
    
    if len(sys.argv) != 2:
        print("Usage: python3 set_ramp_rate.py <rate>")
        print()
        print("Examples:")
        print("  python3 set_ramp_rate.py 5   # Gentle acceleration")
        print("  python3 set_ramp_rate.py 10  # Balanced (recommended)")
        print("  python3 set_ramp_rate.py 15  # Responsive acceleration")
        print()
        print("Recommended range: 3-15 turns/s²")
        print("  3-7:   Gentle, smooth acceleration")
        print("  8-12:  Balanced performance")
        print("  13-15: Quick, responsive acceleration")
        print()
        sys.exit(1)
    
    try:
        rate = float(sys.argv[1])
        success = set_ramp_rate(rate)
        sys.exit(0 if success else 1)
    except ValueError:
        print(f"❌ ERROR: Invalid rate '{sys.argv[1]}' - must be a number")
        sys.exit(1)

