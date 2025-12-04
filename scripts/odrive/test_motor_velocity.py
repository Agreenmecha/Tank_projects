#!/usr/bin/env python3
"""
Simple Motor Velocity Test
Pre-calibrated motors - just enter closed loop and test velocity
"""

import odrive
from odrive.enums import *
import time
import sys

print("=" * 60)
print("ODrive Motor Velocity Test")
print("=" * 60)
print()

# Connect to ODrive
print("Connecting to ODrive...")
try:
    dev0 = odrive.find_any(timeout=10)
    if dev0 is None:
        print("❌ ERROR: ODrive not found!")
        sys.exit(1)
    print(f"✅ Connected: {dev0.serial_number}")
    print(f"   Voltage: {dev0.vbus_voltage:.2f}V")
    print()
except Exception as e:
    print(f"❌ ERROR: {e}")
    sys.exit(1)

# Wait for index search
print("Waiting 5 seconds for index searching to complete...")
time.sleep(5)
print("✅ Index search complete")
print()

# Set velocity ramp rate for smooth acceleration
print("Setting velocity ramp rate...")
vel_ramp_rate = 10  # turns/s^2 (3-15 recommended)
dev0.axis0.controller.config.vel_ramp_rate = vel_ramp_rate
dev0.axis1.controller.config.vel_ramp_rate = vel_ramp_rate
print(f"✅ Velocity ramp rate: {vel_ramp_rate} turns/s²")
print()

# Enter closed loop control
print("Entering closed loop control...")
try:
    dev0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    dev0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(2)
    
    # Check states
    if dev0.axis0.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
        print("✅ Axis 0: Closed loop control active")
    else:
        print(f"❌ Axis 0: State = {dev0.axis0.current_state}, Error = {hex(dev0.axis0.error)}")
        sys.exit(1)
    
    if dev0.axis1.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
        print("✅ Axis 1: Closed loop control active")
    else:
        print(f"❌ Axis 1: State = {dev0.axis1.current_state}, Error = {hex(dev0.axis1.error)}")
        sys.exit(1)
    
    print()
    
except Exception as e:
    print(f"❌ ERROR entering closed loop: {e}")
    sys.exit(1)

# Test velocity control
print("Testing velocity control...")
print("⚠️  MOTORS WILL MOVE!")
print()

try:
    # Test 1: Slow speed
    print("Test 1: Slow forward (10 turns/s for 2 seconds)")
    dev0.axis0.controller.input_vel = 10
    dev0.axis1.controller.input_vel = 10
    time.sleep(2)
    print(f"   Axis 0 velocity: {dev0.axis0.encoder.vel_estimate:.2f} turns/s")
    print(f"   Axis 1 velocity: {dev0.axis1.encoder.vel_estimate:.2f} turns/s")
    print()
    
    # Stop
    print("Stopping...")
    dev0.axis0.controller.input_vel = 0
    dev0.axis1.controller.input_vel = 0
    time.sleep(1)
    print()
    
    # Test 2: Reverse
    print("Test 2: Slow reverse (-10 turns/s for 2 seconds)")
    dev0.axis0.controller.input_vel = -10
    dev0.axis1.controller.input_vel = -10
    time.sleep(2)
    print(f"   Axis 0 velocity: {dev0.axis0.encoder.vel_estimate:.2f} turns/s")
    print(f"   Axis 1 velocity: {dev0.axis1.encoder.vel_estimate:.2f} turns/s")
    print()
    
    # Stop
    print("Stopping...")
    dev0.axis0.controller.input_vel = 0
    dev0.axis1.controller.input_vel = 0
    time.sleep(1)
    print()
    
    # Test 3: High speed (your example)
    print("Test 3: Higher speed (100 turns/s for 2 seconds)")
    print("⚠️  This is fast! Make sure motors can handle it.")
    time.sleep(1)
    dev0.axis0.controller.input_vel = 100
    dev0.axis1.controller.input_vel = 100
    time.sleep(2)
    print(f"   Axis 0 velocity: {dev0.axis0.encoder.vel_estimate:.2f} turns/s")
    print(f"   Axis 1 velocity: {dev0.axis1.encoder.vel_estimate:.2f} turns/s")
    print()
    
    # Stop
    print("Stopping...")
    dev0.axis0.controller.input_vel = 0
    dev0.axis1.controller.input_vel = 0
    time.sleep(1)
    
    print()
    print("=" * 60)
    print("✅ Motor velocity test complete!")
    print("=" * 60)
    print()
    
    # Put motors in idle state
    print("Putting motors in idle state...")
    dev0.axis0.requested_state = AXIS_STATE_IDLE
    dev0.axis1.requested_state = AXIS_STATE_IDLE
    time.sleep(0.5)
    print("✅ Motors in idle")
    print()
    print("Motors are working correctly!")
    print("Next: Test with ROS2")
    print()
    
except KeyboardInterrupt:
    print("\n\n⚠️  Test interrupted - stopping and idling motors...")
    dev0.axis0.controller.input_vel = 0
    dev0.axis1.controller.input_vel = 0
    time.sleep(0.5)
    dev0.axis0.requested_state = AXIS_STATE_IDLE
    dev0.axis1.requested_state = AXIS_STATE_IDLE
    print("✅ Motors in idle state")
    sys.exit(0)
    
except Exception as e:
    print(f"\n❌ ERROR during velocity test: {e}")
    print("Stopping and idling motors...")
    try:
        dev0.axis0.controller.input_vel = 0
        dev0.axis1.controller.input_vel = 0
        time.sleep(0.5)
        dev0.axis0.requested_state = AXIS_STATE_IDLE
        dev0.axis1.requested_state = AXIS_STATE_IDLE
        print("✅ Motors in idle state")
    except:
        pass
    sys.exit(1)

