#!/usr/bin/env python3
"""
Controller Input Diagnostic Tool
Press each button/stick to see what it maps to
"""

import pygame
import sys
import time

def main():
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("❌ No controller detected!")
        sys.exit(1)
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    print("="*60)
    print(f"Controller: {joystick.get_name()}")
    print(f"Axes: {joystick.get_numaxes()}")
    print(f"Buttons: {joystick.get_numbuttons()}")
    print(f"Hats: {joystick.get_numhats()}")
    print("="*60)
    print()
    print("TEST PLAN:")
    print("  1. Left Stick: Up, Down, Left, Right")
    print("  2. Right Stick: Up, Down, Left, Right")
    print("  3. Buttons: X, O, Triangle, Square")
    print()
    print("Press Ctrl+C to quit")
    print("="*60)
    print()
    
    clock = pygame.time.Clock()
    
    # Track previous values to only print changes
    prev_axes = [0.0] * joystick.get_numaxes()
    prev_buttons = [0] * joystick.get_numbuttons()
    prev_hats = [(0, 0)] * joystick.get_numhats()
    
    try:
        while True:
            pygame.event.pump()
            
            # Check axes
            for i in range(joystick.get_numaxes()):
                value = joystick.get_axis(i)
                # Only print if value changed significantly (deadzone)
                if abs(value - prev_axes[i]) > 0.1:
                    print(f"AXIS {i}: {value:+.3f}")
                    prev_axes[i] = value
            
            # Check buttons
            for i in range(joystick.get_numbuttons()):
                value = joystick.get_button(i)
                if value != prev_buttons[i]:
                    state = "PRESSED" if value else "RELEASED"
                    print(f"BUTTON {i}: {state}")
                    prev_buttons[i] = value
            
            # Check hats (D-pad)
            for i in range(joystick.get_numhats()):
                value = joystick.get_hat(i)
                if value != prev_hats[i]:
                    print(f"HAT {i}: {value}")
                    prev_hats[i] = value
            
            clock.tick(60)  # 60 Hz polling
            
    except KeyboardInterrupt:
        print("\n\nDone!")
        pygame.quit()

if __name__ == '__main__':
    main()

