#!/usr/bin/env python3
"""
Sensor Diagnostic Script for Tank
Checks all sensors and controllers
"""

import odrive
from odrive.enums import *
import sys
import time

try:
    import pygame
except ImportError:
    print("⚠️  pygame not installed - gamepad check will be skipped")
    pygame = None

def print_header(text):
    """Print a section header"""
    print("\n" + "="*60)
    print(f"  {text}")
    print("="*60)

def check_gamepad():
    """Check if gamepad is connected and working"""
    print_header("GAMEPAD CONTROLLER CHECK")
    
    if pygame is None:
        print("❌ pygame not available")
        return False
    
    import os
    os.environ['SDL_JOYSTICK_DEVICE'] = '/dev/input/event*'
    
    pygame.init()
    pygame.joystick.init()
    time.sleep(0.5)
    pygame.event.pump()
    
    if pygame.joystick.get_count() == 0:
        print("❌ No gamepad detected!")
        print("\nTroubleshooting:")
        print("  1. Check USB dongle: lsusb")
        print("  2. Turn on the controller")
        print("  3. Check permissions: ls -l /dev/input/event*")
        return False
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    print(f"✅ Gamepad Connected: {joystick.get_name()}")
    print(f"   Axes: {joystick.get_numaxes()}")
    print(f"   Buttons: {joystick.get_numbuttons()}")
    print(f"   Hats: {joystick.get_numhats()}")
    
    # Read current values
    print("\nCurrent Readings:")
    for i in range(joystick.get_numaxes()):
        value = joystick.get_axis(i)
        print(f"   Axis {i}: {value:+.3f}")
    
    for i in range(joystick.get_numbuttons()):
        state = joystick.get_button(i)
        if state:
            print(f"   Button {i}: PRESSED")
    
    pygame.quit()
    return True

def check_odrive_connection():
    """Check ODrive connection"""
    print_header("ODRIVE CONNECTION CHECK")
    
    print("Searching for ODrive...")
    try:
        dev0 = odrive.find_any(timeout=10)
        if dev0 is None:
            print("❌ ODrive not found")
            print("\nTroubleshooting:")
            print("  1. Check USB connection: lsusb | grep ODrive")
            print("  2. Check dmesg: dmesg | tail -20")
            print("  3. Try: odrivetool")
            return None
        
        print(f"✅ ODrive Connected")
        print(f"   Serial Number: {dev0.serial_number}")
        print(f"   Hardware Version: v{dev0.hw_version_major}.{dev0.hw_version_minor}")
        print(f"   Firmware Version: v{dev0.fw_version_major}.{dev0.fw_version_minor}.{dev0.fw_version_revision}")
        print(f"   Vbus Voltage: {dev0.vbus_voltage:.2f}V")
        
        return dev0
    
    except Exception as e:
        print(f"❌ Error connecting to ODrive: {e}")
        return None

def check_axis_errors(dev0, axis_num):
    """Check and decode axis errors"""
    axis = getattr(dev0, f'axis{axis_num}')
    
    errors = []
    if axis.error != 0:
        errors.append(f"Axis Error: {hex(axis.error)}")
    if axis.motor.error != 0:
        errors.append(f"Motor Error: {hex(axis.motor.error)}")
    if axis.encoder.error != 0:
        errors.append(f"Encoder Error: {hex(axis.encoder.error)}")
    if axis.controller.error != 0:
        errors.append(f"Controller Error: {hex(axis.controller.error)}")
    
    return errors

def check_encoders(dev0):
    """Check encoder readings"""
    print_header("ENCODER SENSORS CHECK")
    
    for axis_num in [0, 1]:
        axis = getattr(dev0, f'axis{axis_num}')
        encoder = axis.encoder
        
        print(f"\nAxis {axis_num} Encoder:")
        
        # Check errors
        errors = check_axis_errors(dev0, axis_num)
        if errors:
            print("❌ ERRORS DETECTED:")
            for error in errors:
                print(f"   {error}")
        else:
            print("✅ No errors")
        
        # Check if encoder is ready
        if encoder.is_ready:
            print("✅ Encoder Ready")
        else:
            print("❌ Encoder Not Ready")
            if not encoder.index_found:
                print("   ⚠️  Index not found (may need homing)")
        
        # Read encoder values
        print(f"\nReadings:")
        print(f"   Position: {encoder.pos_estimate:.3f} turns")
        print(f"   Velocity: {encoder.vel_estimate:.3f} turns/s")
        print(f"   Shadow Count: {encoder.shadow_count}")
        print(f"   Count in CPR: {encoder.count_in_cpr}")
        
        # Check if encoder is configured
        print(f"\nConfiguration:")
        print(f"   CPR: {encoder.config.cpr}")
        print(f"   Mode: {encoder.config.mode}")
        print(f"   Bandwidth: {encoder.config.bandwidth} Hz")
        print(f"   Use Index: {encoder.config.use_index}")
        print(f"   Index Found: {encoder.index_found}")

def check_motors(dev0):
    """Check motor status"""
    print_header("MOTOR STATUS CHECK")
    
    for axis_num in [0, 1]:
        axis = getattr(dev0, f'axis{axis_num}')
        motor = axis.motor
        
        print(f"\nAxis {axis_num} Motor:")
        
        # Check state
        state_names = {
            0: "UNDEFINED",
            1: "IDLE",
            2: "STARTUP_SEQUENCE",
            3: "FULL_CALIBRATION_SEQUENCE",
            4: "MOTOR_CALIBRATION",
            5: "ENCODER_INDEX_SEARCH",
            6: "ENCODER_OFFSET_CALIBRATION",
            7: "CLOSED_LOOP_CONTROL",
            8: "LOCKIN_SPIN",
            9: "ENCODER_DIR_FIND",
            10: "HOMING",
            11: "ENCODER_HALL_POLARITY_CALIBRATION",
            12: "ENCODER_HALL_PHASE_CALIBRATION"
        }
        
        state = axis.current_state
        state_name = state_names.get(state, f"UNKNOWN({state})")
        print(f"   State: {state_name}")
        
        # Check errors
        if motor.error != 0:
            print(f"❌ Motor Error: {hex(motor.error)}")
        else:
            print("✅ No motor errors")
        
        # Motor measurements
        print(f"\nMeasurements:")
        print(f"   Current (q-axis): {motor.current_control.Iq_measured:.3f} A")
        print(f"   Current (d-axis): {motor.current_control.Id_measured:.3f} A")
        print(f"   DC Current: {motor.I_bus:.3f} A")
        print(f"   DC Power: {motor.I_bus * dev0.vbus_voltage:.3f} W")
        
        # Temperature if available
        try:
            if hasattr(motor, 'fet_thermistor') and hasattr(motor.fet_thermistor, 'temperature'):
                temp = motor.fet_thermistor.temperature
                if temp is not None:
                    print(f"   FET Temperature: {temp:.1f}°C")
        except:
            pass
        
        # Configuration
        print(f"\nConfiguration:")
        print(f"   Current Limit: {motor.config.current_lim} A")
        print(f"   Calibration Current: {motor.config.calibration_current} A")
        print(f"   Pole Pairs: {motor.config.pole_pairs}")
        print(f"   Resistance (R): {motor.config.phase_resistance:.4f} Ω")
        print(f"   Inductance (L): {motor.config.phase_inductance * 1e6:.2f} µH")

def check_controller(dev0):
    """Check controller status"""
    print_header("CONTROLLER STATUS CHECK")
    
    for axis_num in [0, 1]:
        axis = getattr(dev0, f'axis{axis_num}')
        controller = axis.controller
        
        print(f"\nAxis {axis_num} Controller:")
        
        # Check errors
        if controller.error != 0:
            print(f"❌ Controller Error: {hex(controller.error)}")
        else:
            print("✅ No controller errors")
        
        # Control mode
        mode_names = {
            0: "VOLTAGE_CONTROL",
            1: "TORQUE_CONTROL",
            2: "VELOCITY_CONTROL",
            3: "POSITION_CONTROL"
        }
        mode = controller.config.control_mode
        mode_name = mode_names.get(mode, f"UNKNOWN({mode})")
        print(f"   Control Mode: {mode_name}")
        
        # Current setpoints and measurements
        print(f"\nSetpoints:")
        print(f"   Input Velocity: {controller.input_vel:.3f} turns/s")
        print(f"   Input Position: {controller.input_pos:.3f} turns")
        
        print(f"\nGains:")
        print(f"   Velocity Gain: {controller.config.vel_gain:.4f}")
        print(f"   Velocity Integrator Gain: {controller.config.vel_integrator_gain:.4f}")
        print(f"   Velocity Limit: {controller.config.vel_limit:.2f} turns/s")

def live_monitor(dev0, duration=5):
    """Monitor sensors in real-time"""
    print_header(f"LIVE SENSOR MONITOR ({duration}s)")
    
    print("\nMonitoring encoder velocity and motor current...")
    print("Time(s) | Axis0 Vel | Axis0 Cur | Axis1 Vel | Axis1 Cur")
    print("-" * 60)
    
    start_time = time.time()
    while time.time() - start_time < duration:
        elapsed = time.time() - start_time
        
        vel0 = dev0.axis0.encoder.vel_estimate
        cur0 = dev0.axis0.motor.current_control.Iq_measured
        vel1 = dev0.axis1.encoder.vel_estimate
        cur1 = dev0.axis1.motor.current_control.Iq_measured
        
        print(f"{elapsed:6.2f}  | {vel0:9.3f} | {cur0:9.3f} | {vel1:9.3f} | {cur1:9.3f}", end='\r')
        time.sleep(0.1)
    
    print("\n✅ Monitoring complete")

def main():
    print("\n" + "╔" + "═"*58 + "╗")
    print("║" + " "*15 + "TANK SENSOR DIAGNOSTICS" + " "*20 + "║")
    print("╚" + "═"*58 + "╝")
    
    # Check gamepad
    gamepad_ok = check_gamepad()
    
    # Check ODrive
    dev0 = check_odrive_connection()
    if dev0 is None:
        print("\n❌ Cannot continue without ODrive connection")
        sys.exit(1)
    
    # Check encoders
    check_encoders(dev0)
    
    # Check motors
    check_motors(dev0)
    
    # Check controller
    check_controller(dev0)
    
    # Live monitoring
    try:
        print("\n\nPress Ctrl+C to skip live monitoring...")
        time.sleep(2)
        live_monitor(dev0)
    except KeyboardInterrupt:
        print("\n⏭️  Skipped live monitoring")
    
    # Summary
    print_header("DIAGNOSTIC SUMMARY")
    
    all_ok = True
    
    if gamepad_ok:
        print("✅ Gamepad: OK")
    else:
        print("❌ Gamepad: FAILED")
        all_ok = False
    
    print(f"✅ ODrive: Connected (S/N: {dev0.serial_number})")
    
    for axis_num in [0, 1]:
        errors = check_axis_errors(dev0, axis_num)
        axis = getattr(dev0, f'axis{axis_num}')
        
        if errors:
            print(f"❌ Axis {axis_num}: ERRORS DETECTED")
            all_ok = False
        elif not axis.encoder.is_ready:
            print(f"⚠️  Axis {axis_num}: Encoder not ready")
            all_ok = False
        else:
            print(f"✅ Axis {axis_num}: OK")
    
    print("\n" + "="*60)
    if all_ok:
        print("  🎉 ALL SENSORS WORKING!")
    else:
        print("  ⚠️  SOME ISSUES DETECTED - Review details above")
    print("="*60 + "\n")

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


