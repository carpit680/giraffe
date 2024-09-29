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
DEFAULT_ID                   = 1


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = sts(portHandler)
    
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


print("Input total number of servos : ")
total_servo = int(input())

for index in range(DEFAULT_ID, DEFAULT_ID + total_servo):
    new_id = total_servo + DEFAULT_ID - index
    if index == DEFAULT_ID:
        print("Attach Base Servo and then press Enter.")
    else:
        print("Attach next Servo and then press Enter.")
    input()
    sts_model_number, sts_comm_result, sts_error = packetHandler.ping(DEFAULT_ID)
    if sts_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(sts_comm_result))
    else:
        print("[ID:%03d] ping Succeeded. STServo model number : %d" % (DEFAULT_ID, sts_model_number))
        print("[ID:%03d] Unlocking Eprom." % DEFAULT_ID)
        packetHandler.unLockEprom(DEFAULT_ID)
        print("Replacing ID:%03d -> ID:%03d" % (DEFAULT_ID, new_id))
        packetHandler.setID(DEFAULT_ID, new_id)
        print("[ID:%03d] Locking Eprom." % new_id)
        packetHandler.LockEprom(DEFAULT_ID)
        sts_model_number, sts_comm_result, sts_error = packetHandler.ping(new_id)
        if sts_comm_result != COMM_SUCCESS:
            print("Failed to replace ID with error: %s" % packetHandler.getTxRxResult(sts_comm_result))
            print("Press any key to terminate...")
            getch()
            quit()
        else:
            print("[ID:%03d] ID replacement Succeeded. STServo model number : %d" % (new_id, sts_model_number))
        if sts_error != 0:
            print("STS Error while pinging new ID: %s" % packetHandler.getRxPacketError(sts_error))
            print("Press any key to terminate...")
            getch()
            quit()
    if sts_error != 0:
        print("STS Error while pinging new servo: %s" % packetHandler.getRxPacketError(sts_error))
        print("Press any key to terminate...")
        getch()
        quit()
    

print("Succeeded in setting all servo IDs, I think? Exiting...")

# Close port
portHandler.closePort()
