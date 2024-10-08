#!/usr/bin/env python

import sys, tty, termios
from STservo_sdk import *

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Default setting
SCSCL_ID                  = 1                 # SCServo ID : 1
BAUDRATE                = 1000000           # SCServo default baudrate : 1000000
DEVICENAME              = '/dev/tty.usbmodem585A0080511' # '/dev/ttyACM0'

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

# Try to ping the STServo
# Get STServo model number
scscl_model_number, scscl_comm_result, scscl_error = packetHandler.ping(SCSCL_ID)
if scscl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scscl_comm_result))
else:
    print("[ID:%03d] ping Succeeded. SCServo model number : %d" % (SCSCL_ID, scscl_model_number))
if scscl_error != 0:
    print("%s" % packetHandler.getRxPacketError(scscl_error))

# Close port
portHandler.closePort()
