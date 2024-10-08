#!/usr/bin/env python

import os
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

from STservo_sdk import *                      # Uses STServo SDK library

# Default setting
SCSCL_ID                      = 1                 # STServo ID : 1
BAUDRATE                    = 1000000           # STServo default baudrate : 1000000
DEVICENAME                  = '/dev/tty.usbmodem585A0080511'  # '/dev/ttyACM0'

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = scscl(portHandler)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break
    # Read STServo present position and speed
    scscl_present_position, scscl_present_speed, scscl_comm_result, scscl_error = packetHandler.ReadPosSpeed(SCSCL_ID)
    if scscl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(scscl_comm_result))
    else:
        print("[ID:%03d] PresPos:%d PresSpd:%d" % (SCSCL_ID, scscl_present_position, scscl_present_speed))
    if scscl_error != 0:
        print(packetHandler.getRxPacketError(scscl_error))

# Close port
portHandler.closePort()
