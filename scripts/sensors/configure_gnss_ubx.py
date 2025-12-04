#!/usr/bin/env python3
"""
Configure ZED-F9P to output UBX messages instead of NMEA
Run this once to save configuration to flash memory
"""

import serial
import struct
import time

# UBX protocol helpers
def ubx_checksum(msg):
    """Calculate UBX checksum"""
    ck_a = 0
    ck_b = 0
    for byte in msg:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])

def send_ubx(ser, msg_class, msg_id, payload):
    """Send UBX message"""
    header = b'\xb5\x62'  # UBX header
    msg_body = bytes([msg_class, msg_id]) + struct.pack('<H', len(payload)) + payload
    checksum = ubx_checksum(msg_body)
    
    packet = header + msg_body + checksum
    ser.write(packet)
    time.sleep(0.1)
    print(f"Sent: 0x{msg_class:02X} 0x{msg_id:02X}")

def configure_device(port='/dev/ttyACM0'):
    """Configure ZED-F9P for UBX output"""
    
    print("=" * 50)
    print("ZED-F9P UBX Configuration")
    print("=" * 50)
    print(f"Opening {port}...")
    
    try:
        ser = serial.Serial(port, baudrate=38400, timeout=1)
        time.sleep(1)
        
        # 1. Disable NMEA output on USB
        print("\n1. Disabling NMEA output on USB...")
        # CFG-VALSET: CFG_USBOUTPROT_NMEA = 0
        payload = struct.pack('<BBHI', 0, 0x01, 0, 0x10780002) + struct.pack('B', 0)  # key + value
        send_ubx(ser, 0x06, 0x8A, payload)
        
        # 2. Enable UBX output on USB
        print("2. Enabling UBX output on USB...")
        payload = struct.pack('<BBHI', 0, 0x01, 0, 0x10780001) + struct.pack('B', 1)  # key + value
        send_ubx(ser, 0x06, 0x8A, payload)
        
        # 3. Disable INFO messages
        print("3. Disabling INFO messages...")
        payload = struct.pack('<BBHI', 0, 0x01, 0, 0x20920001) + struct.pack('B', 0)  # CFG_INFMSG_UBX_USB
        send_ubx(ser, 0x06, 0x8A, payload)
        
        # 4. Enable NAV-PVT on USB (position/velocity/time)
        print("4. Enabling NAV-PVT...")
        payload = struct.pack('<BBHI', 0, 0x01, 0, 0x20910007) + struct.pack('B', 1)  # CFG_MSGOUT_UBX_NAV_PVT_USB
        send_ubx(ser, 0x06, 0x8A, payload)
        
        # 5. Enable NAV-HP-POS-LLH on USB (high-precision lat/lon/height)
        print("5. Enabling NAV-HPPOSLLH...")
        payload = struct.pack('<BBHI', 0, 0x01, 0, 0x20910036) + struct.pack('B', 1)  # CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB
        send_ubx(ser, 0x06, 0x8A, payload)
        
        # 6. Enable NAV-DOP (dilution of precision)
        print("6. Enabling NAV-DOP...")
        payload = struct.pack('<BBHI', 0, 0x01, 0, 0x20910039) + struct.pack('B', 1)  # CFG_MSGOUT_UBX_NAV_DOP_USB
        send_ubx(ser, 0x06, 0x8A, payload)
        
        # 7. Enable NAV-COV (covariance)
        print("7. Enabling NAV-COV...")
        payload = struct.pack('<BBHI', 0, 0x01, 0, 0x20910084) + struct.pack('B', 1)  # CFG_MSGOUT_UBX_NAV_COV_USB
        send_ubx(ser, 0x06, 0x8A, payload)
        
        # 8. Enable NAV-STATUS
        print("8. Enabling NAV-STATUS...")
        payload = struct.pack('<BBHI', 0, 0x01, 0, 0x2091001B) + struct.pack('B', 1)  # CFG_MSGOUT_UBX_NAV_STATUS_USB
        send_ubx(ser, 0x06, 0x8A, payload)
        
        # 9. Enable NAV-SAT (satellite info)
        print("9. Enabling NAV-SAT...")
        payload = struct.pack('<BBHI', 0, 0x01, 0, 0x20910016) + struct.pack('B', 1)  # CFG_MSGOUT_UBX_NAV_SAT_USB
        send_ubx(ser, 0x06, 0x8A, payload)
        
        # 10. Set update rate to 200ms (5 Hz)
        print("10. Setting update rate to 5 Hz...")
        payload = struct.pack('<BBHI', 0, 0x01, 0, 0x30210001) + struct.pack('<H', 200)  # CFG_RATE_MEAS = 200ms
        send_ubx(ser, 0x06, 0x8A, payload)
        
        # 11. Save configuration to flash memory
        print("\n11. Saving configuration to flash memory...")
        # CFG-CFG: Save all sections to flash
        payload = struct.pack('<III', 0xFFFFFFFF, 0, 0)  # clearMask, saveMask, loadMask
        send_ubx(ser, 0x06, 0x09, payload)
        
        time.sleep(1)
        
        print("\n" + "=" * 50)
        print("✓ Configuration complete!")
        print("✓ Settings saved to flash memory")
        print("=" * 50)
        print("\nThe ZED-F9P is now configured to output UBX messages.")
        print("You can now use it with ROS 2!")
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"ERROR: Could not open {port}")
        print(f"       {e}")
        print("\nMake sure:")
        print("  1. ZED-F9P is connected via USB")
        print("  2. No other program is using the device")
        print("  3. You have proper permissions (run with sudo if needed)")
        return False
    
    return True

if __name__ == "__main__":
    import sys
    
    port = '/dev/ttyACM0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print("\n⚠️  WARNING: This will reconfigure your ZED-F9P")
    print("   Make sure no other programs are using the device")
    print("   (Close u-center, stop ROS 2 node, etc.)")
    print()
    input("Press Enter to continue or Ctrl+C to cancel...")
    
    success = configure_device(port)
    
    if success:
        print("\n✓ Done! Now restart the ROS 2 node:")
        print("  ros2 launch tank_sensors gnss.launch.py")
    else:
        sys.exit(1)

