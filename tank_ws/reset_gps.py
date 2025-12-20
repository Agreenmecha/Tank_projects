#!/usr/bin/env python3
"""
Factory reset ZED-F9P to enable NMEA output
"""
import serial
import struct
import time

def ubx_checksum(msg):
    """Calculate UBX checksum"""
    ck_a = 0
    ck_b = 0
    for byte in msg:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])

def send_ubx_reset():
    """Send UBX-CFG-CFG to reset to factory defaults and save"""
    port = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
    
    # UBX-CFG-CFG: Clear, Save, and Load all settings to/from flash
    # Class: 0x06, ID: 0x09
    # Payload: clearMask (4 bytes), saveMask (4 bytes), loadMask (4 bytes), deviceMask (1 byte)
    # 0xFFFF1F1F = all settings
    # deviceMask: 0x17 = devBBR | devFLASH | devEEPROM
    
    msg_class = 0x06
    msg_id = 0x09
    
    # Reset all settings
    payload = struct.pack('<IIIB',
        0x00001F1F,  # clearMask: Clear all (except antenna config)
        0x00001F1F,  # saveMask: Save all
        0x00001F1F,  # loadMask: Load all  
        0x17         # deviceMask: All storage
    )
    
    length = len(payload)
    msg = bytes([msg_class, msg_id]) + struct.pack('<H', length) + payload
    packet = b'\xb5\x62' + msg + ubx_checksum(msg)
    
    print(f"Sending factory reset command to GPS...")
    port.write(packet)
    time.sleep(2)  # Wait for GPS to reset
    
    port.close()
    print("GPS reset complete. It should now output NMEA sentences by default.")

if __name__ == '__main__':
    send_ubx_reset()

