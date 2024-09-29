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

parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)
from STservo_sdk import *                      # Uses STServo SDK library

# Default setting
BAUDRATE                    = 1000000           # STServo default baudrate : 1000000
DEVICENAME                  = '/dev/tty.usbmodem585A0080511' # '/dev/ttyACM0'    # Check which port is being used on your controller

STS_MINIMUM_POSITION_VALUE  = 0                 # STServo will rotate between this value
STS_MAXIMUM_POSITION_VALUE  = 4095
STS_MOVING_SPEED            = 1500              # STServo moving speed
STS_MOVING_ACC              = 50                # STServo moving acc

SCSCL_MINIMUM_POSITION_VALUE  = 1023                # SCSCLervo will rotate between this value
SCSCL_MAXIMUM_POSITION_VALUE  = 0
SCSCL_MOVING_SPEED            = 1500              # SCSCLervo moving speed
SCSCL_MOVING_ACC              = 500                # SCSCLervo moving acc

index = 0
sts_goal_position = [STS_MINIMUM_POSITION_VALUE, STS_MAXIMUM_POSITION_VALUE]         # Goal position
scscl_goal_position = [SCSCL_MINIMUM_POSITION_VALUE, SCSCL_MAXIMUM_POSITION_VALUE]         # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)
portHandlersc = PortHandler(DEVICENAME)

# Initialize packetHandler_sts instance
# Get methods and members of Protocol
packetHandler_sts = sts(portHandler)
packetHandler_scscl = scscl(portHandlersc)

# Open port
if portHandler.openPort() and portHandlersc.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE) and portHandlersc.setBaudRate(BAUDRATE):
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

    # for sts_id in range(1, 11):
    sts_id = 2
    scscl_id = 1
    # Add STServo#1~10 goal position\moving speed\moving accc value to the Syncwrite parameter storage
    sts_addparam_result = packetHandler_sts.SyncWritePosEx(sts_id, sts_goal_position[index], STS_MOVING_SPEED, STS_MOVING_ACC)
    scscl_addparam_result = packetHandler_scscl.SyncWritePos(scscl_id, scscl_goal_position[index], SCSCL_MOVING_SPEED, SCSCL_MOVING_ACC)
    if sts_addparam_result != True:
        print("[ID: %03d] groupSyncWrite STS addparam failed" % sts_id)
    
    if scscl_addparam_result != True:
        print("[ID: %03d] groupSyncWrite SCSCL addparam failed" % scscl_id)

    # Syncwrite goal position
    sts_comm_result = packetHandler_sts.groupSyncWrite.txPacket()
    scscl_comm_result = packetHandler_scscl.groupSyncWrite.txPacket()
    if sts_comm_result != COMM_SUCCESS:
        print("STS: %s" % packetHandler_sts.getTxRxResult(sts_comm_result))
    
    if scscl_comm_result != COMM_SUCCESS:
        print("SCSCL: %s" % packetHandler_scscl.getTxRxResult(scscl_comm_result))

    # Clear syncwrite parameter storage
    packetHandler_sts.groupSyncWrite.clearParam()
    packetHandler_scscl.groupSyncWrite.clearParam()

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

# Close port
portHandler.closePort()
