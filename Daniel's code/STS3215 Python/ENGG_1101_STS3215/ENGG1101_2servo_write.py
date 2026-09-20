#!/usr/bin/env python
#
# Two Servo Control Example (IDs 1 and 2)
#
# This script controls two ST3215 servos via USB-serial.
# It prompts the user to enter target positions for both servos, then writes them
# with a fixed speed and acceleration.
# Modified by Kelvin Chan

import sys
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Add path to the scservo_sdk (adjust if needed)
sys.path.append("..")
from scservo_sdk import *      # Uses SC Servo SDK library

# Configuration
SERVO_ID_1 = 2
SERVO_ID_2 = 3
BAUDRATE   = 115200           # Must match the ESP32's serial forwarding baudrate
# Check Raspberry Pi serial port with "dmesg" command and look for the port connectng to cp210x
DEVICENAME = 'COM5'   # Change to your port (e.g., '/dev/ttyUSB0', 'COM3')

# Position limits for ST servos (0-4095)
MIN_POS = 0
MAX_POS = 4095

# Movement settings
MOVING_SPEED = 2400            # Speed (0-4095 for ST)
MOVING_ACC   = 50              # Acceleration (0-255)

# Initialize port and packet handler
portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)   # Use sms_sts for ST servos

# Open port
if not portHandler.openPort():
    print("Failed to open the port")
    sys.exit(1)
print("Succeeded to open the port")

# Set baudrate
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set baudrate")
    portHandler.closePort()
    sys.exit(1)
print("Succeeded to set baudrate")

def set_servo_position(servo_id, position, speed=MOVING_SPEED, acc=MOVING_ACC):
    """Write position to a single servo and print result."""
    comm_result, error = packetHandler.WritePosEx(servo_id, position, speed, acc)
    if comm_result != COMM_SUCCESS:
        print(f"  Servo {servo_id}: {packetHandler.getTxRxResult(comm_result)}")
    elif error != 0:
        print(f"  Servo {servo_id}: {packetHandler.getRxPacketError(error)}")
    else:
        print(f"  Servo {servo_id}: position {position} sent successfully")

def read_servo_position(servo_id):
    """Read present position and speed of a servo."""
    pos, speed, comm_result, error = packetHandler.ReadPosSpeed(servo_id)
    if comm_result != COMM_SUCCESS:
        print(f"  Read error on {servo_id}: {packetHandler.getTxRxResult(comm_result)}")
        return None, None
    if error != 0:
        print(f"  Read error on {servo_id}: {packetHandler.getRxPacketError(error)}")
        return None, None
    return pos, speed

print("\nTwo‑servo control ready.")
print("Enter positions for Servo 1 and Servo 2 (0–4095).")
print("Press ESC at any prompt to quit.\n")

while True:
    # Get target positions from user
    try:
        # Servo 1
        print(f"Servo {SERVO_ID_1} target position [{MIN_POS}-{MAX_POS}]: ", end="")
        val1 = input().strip()
        if val1 == chr(27):      # ESC key (in raw input, but here input() strips)
            break
        pos1 = int(val1)
        if not (MIN_POS <= pos1 <= MAX_POS):
            print(f"Position out of range. Using {MIN_POS}..{MAX_POS}.")
            continue

        # Servo 2
        print(f"Servo {SERVO_ID_2} target position [{MIN_POS}-{MAX_POS}]: ", end="")
        val2 = input().strip()
        if val2 == chr(27):
            break
        pos2 = int(val2)
        if not (MIN_POS <= pos2 <= MAX_POS):
            print(f"Position out of range. Using {MIN_POS}..{MAX_POS}.")
            continue

    except ValueError:
        print("Invalid input. Please enter numbers.")
        continue
    except KeyboardInterrupt:
        break

    # Send commands to both servos
    print("\nSending commands...")
    set_servo_position(SERVO_ID_1, pos1)
    set_servo_position(SERVO_ID_2, pos2)

    # Optional: read back positions to confirm
    print("\nCurrent positions:")
    p1, s1 = read_servo_position(SERVO_ID_1)
    if p1 is not None:
        print(f"  Servo {SERVO_ID_1}: position = {p1}, speed = {s1}")
    p2, s2 = read_servo_position(SERVO_ID_2)
    if p2 is not None:
        print(f"  Servo {SERVO_ID_2}: position = {p2}, speed = {s2}")
    print()

# Clean up
portHandler.closePort()
print("Port closed. Exiting.")