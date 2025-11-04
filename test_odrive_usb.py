#!/usr/bin/env python3
"""
Simple test script for ODrive USB connection
Tests: Connection, encoder reading, basic velocity control
Run BEFORE using with ROS2 to verify ODrive is working
"""

import odrive
from odrive.enums import *
import time
import sys


def test_connection():
    """Test ODrive USB connection."""
    print("=" * 60)
    print("ODrive USB Connection Test")
    print("=" * 60)
    print("\n[1/5] Searching for ODrive...")
    
    try:
        odrv = odrive.find_any(timeout=10)
        if odrv is None:
            print("❌ ERROR: ODrive not found!")
            print("\nTroubleshooting:")
            print("  1. Check USB cable is connected")
            print("  2. Run: lsusb | grep ODrive")
            print("  3. Check permissions: sudo usermod -a -G dialout $USER")
            return None
        
        print(f"✅ Found ODrive: {odrv.serial_number}")
        print(f"   Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
        print(f"   Hardware: v{odrv.hw_version_major}.{odrv.hw_version_minor}")
        
        if odrv.fw_version_major != 0 or odrv.fw_version_minor != 5 or odrv.fw_version_revision != 6:
            print(f"⚠️  WARNING: Expected firmware v0.5.6, found v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
            print("   This script is tested with v0.5.6. Other versions may work but are untested.")
        
        return odrv
        
    except Exception as e:
        print(f"❌ ERROR: {e}")
        return None


def test_axes(odrv):
    """Test both axes."""
    print("\n[2/5] Testing axes...")
    
    for axis_num in [0, 1]:
        axis = odrv.axis0 if axis_num == 0 else odrv.axis1
        print(f"\n   Axis {axis_num}:")
        
        # Check errors
        if axis.error != 0:
            print(f"   ❌ Axis error: {hex(axis.error)}")
            print("      Run: odrv0.clear_errors() in odrivetool")
            return False
        
        if axis.motor.error != 0:
            print(f"   ❌ Motor error: {hex(axis.motor.error)}")
            return False
        
        if axis.encoder.error != 0:
            print(f"   ❌ Encoder error: {hex(axis.encoder.error)}")
            return False
        
        # Check encoder configuration
        print(f"   Encoder CPR: {axis.encoder.config.cpr}")
        print(f"   Encoder ready: {axis.encoder.is_ready}")
        print(f"   Current state: {axis.current_state}")
        
        if not axis.encoder.is_ready:
            print("   ⚠️  Encoder not ready - run calibration")
        else:
            print("   ✅ Axis OK")
    
    return True


def test_encoder_reading(odrv):
    """Test encoder reading."""
    print("\n[3/5] Testing encoder readings...")
    print("   Reading encoder positions for 2 seconds...")
    
    start_time = time.time()
    last_print = start_time
    
    while time.time() - start_time < 2.0:
        if time.time() - last_print > 0.5:
            pos0 = odrv.axis0.encoder.pos_estimate
            pos1 = odrv.axis1.encoder.pos_estimate
            vel0 = odrv.axis0.encoder.vel_estimate
            vel1 = odrv.axis1.encoder.vel_estimate
            
            print(f"   Axis 0: pos={pos0:8.2f} turns, vel={vel0:6.2f} turns/s")
            print(f"   Axis 1: pos={pos1:8.2f} turns, vel={vel1:6.2f} turns/s")
            print()
            
            last_print = time.time()
        
        time.sleep(0.01)
    
    print("   ✅ Encoder reading OK")
    print("   (If motors are stationary, velocity should be near 0)")
    return True


def test_closed_loop(odrv):
    """Test entering closed loop control."""
    print("\n[4/5] Testing closed loop control...")
    print("   ⚠️  Motors may move! Ensure robot is on blocks or has clearance.")
    
    response = input("   Continue? (y/n): ")
    if response.lower() != 'y':
        print("   Skipped")
        return True
    
    try:
        # Set to closed loop control
        print("   Setting axis 0 to closed loop control...")
        odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1)
        
        print("   Setting axis 1 to closed loop control...")
        odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1)
        
        # Check state
        if odrv.axis0.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            print("   ✅ Axis 0 in closed loop control")
        else:
            print(f"   ❌ Axis 0 state: {odrv.axis0.current_state}")
            return False
        
        if odrv.axis1.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            print("   ✅ Axis 1 in closed loop control")
        else:
            print(f"   ❌ Axis 1 state: {odrv.axis1.current_state}")
            return False
        
        return True
        
    except Exception as e:
        print(f"   ❌ ERROR: {e}")
        return False


def test_velocity_control(odrv):
    """Test basic velocity control."""
    print("\n[5/5] Testing velocity control...")
    print("   ⚠️  Motors WILL move! Ensure robot is safe.")
    
    response = input("   Continue? (y/n): ")
    if response.lower() != 'y':
        print("   Skipped")
        return True
    
    try:
        # Set control mode
        odrv.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        odrv.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        odrv.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        odrv.axis1.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        
        print("\n   Test 1: Slow forward (0.5 turns/s for 2 seconds)")
        odrv.axis0.controller.input_vel = 0.5
        odrv.axis1.controller.input_vel = 0.5
        time.sleep(2)
        
        print("   Test 2: Stop")
        odrv.axis0.controller.input_vel = 0.0
        odrv.axis1.controller.input_vel = 0.0
        time.sleep(1)
        
        print("   Test 3: Slow reverse (-0.5 turns/s for 2 seconds)")
        odrv.axis0.controller.input_vel = -0.5
        odrv.axis1.controller.input_vel = -0.5
        time.sleep(2)
        
        print("   Test 4: Stop")
        odrv.axis0.controller.input_vel = 0.0
        odrv.axis1.controller.input_vel = 0.0
        time.sleep(1)
        
        print("   ✅ Velocity control test complete")
        return True
        
    except Exception as e:
        print(f"   ❌ ERROR: {e}")
        # Try to stop motors
        try:
            odrv.axis0.controller.input_vel = 0.0
            odrv.axis1.controller.input_vel = 0.0
        except:
            pass
        return False


def main():
    print("\nODrive USB Test Script for Tank Project")
    print("Firmware: v0.5.6")
    print("Interface: USB (native protocol)\n")
    
    # Test connection
    odrv = test_connection()
    if odrv is None:
        sys.exit(1)
    
    # Test axes
    if not test_axes(odrv):
        print("\n❌ Axis test failed. Fix errors before continuing.")
        sys.exit(1)
    
    # Test encoder reading
    if not test_encoder_reading(odrv):
        print("\n❌ Encoder reading test failed.")
        sys.exit(1)
    
    # Test closed loop
    if not test_closed_loop(odrv):
        print("\n❌ Closed loop test failed.")
        sys.exit(1)
    
    # Test velocity control
    if not test_velocity_control(odrv):
        print("\n❌ Velocity control test failed.")
        sys.exit(1)
    
    # Success
    print("\n" + "=" * 60)
    print("✅ ALL TESTS PASSED!")
    print("=" * 60)
    print("\nODrive is ready for ROS2 integration.")
    print("\nNext steps:")
    print("  1. Configure ROS2 parameters in config/odrive_params.yaml")
    print("  2. Launch: ros2 launch tank_control motor_control.launch.py")
    print("  3. Test with: ros2 topic pub /cmd_vel geometry_msgs/Twist ...")
    print()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(0)

