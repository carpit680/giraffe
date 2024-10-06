#!/usr/bin/env python

import os
import time
import sys, tty, termios
import serial.tools.list_ports
import pyfiglet

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

    
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

portHandler = None
packetHandler = None
def menu_select():
    options = {
        '1': 'Detect and connect to Waveshare servo driver port',
        '2': 'Ping servo',
        '3': 'Find and reset servo ID',
        '4': 'Set servo ID',
        '5': 'Set all servo IDs',
        '6': 'Quit'
    }
    print()
    for key, value in options.items():
        print(f'{key}) {value}')

    choice = input("\n>> Select menu item: ")
    if choice == '':
        choice = 1

    if choice == '1':
        detect_servo_port()
        print(">> Please reconnect Waveshare servo driver and press Enter.")
        getch()
        time.sleep(2)
        return True

    if choice == '2':
        while True:
            if ping_servo():
                continue_choice = input(">> Would you like to continue? (y/n): ")
                if continue_choice == 'y' or continue_choice == 'Y':
                    continue
                else: return True
            else: return True

    elif choice == '3':
        while True:
            if reset_servo_id():
                continue_choice = input(">> Would you like to continue? (y/n): ")
                if continue_choice == 'y' or continue_choice == 'Y':
                    continue
                else: return True
            else: return True

    elif choice == '4':
        while True:
            if set_servo_id():
                continue_choice = input(">> Would you like to continue? (y/n): ")
                if continue_choice == 'y' or continue_choice == 'Y':
                    continue
                else: return True
            else: return True

    elif choice == '5':
        while True:
            if set_all_servo_id():
                continue_choice = input(">> Would you like to continue? (y/n): ")
                if continue_choice == 'y' or continue_choice == 'Y':
                    continue
                else: return True
            else: return True

    elif choice == '6':
        return False

    else: return True

def main():

    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    global portHandler, packetHandler
    connection_choice = input(">> Would you like to connect to default Waveshare servo driver port? (y/n): ")
    if connection_choice != 'y' and connection_choice != 'Y':
        detect_servo_port()
        print(">> Please reconnect Waveshare servo driver and press Enter.")
        getch()
        time.sleep(2)

    portHandler = PortHandler(DEVICENAME)
    # Initialize PacketHandler instance
    # Get methods and members of Protocol
    packetHandler = sts(portHandler)

    # Open port
    try:
        portHandler.openPort()
        print(f"{bcolors.OKGREEN}\nSuccessfully opened the port! {bcolors.ENDC}")
    except Exception as e:
        print(f"{bcolors.FAIL}Failed to open the port. \n{e} {bcolors.ENDC}")
        print(">> Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    try:
        portHandler.setBaudRate(BAUDRATE)
        print(f"{bcolors.OKGREEN}Successfully set the baudrate! {bcolors.ENDC}")
    except Exception as e:
        print(f"{bcolors.FAIL}Failed to set the baudrate. {e} {bcolors.ENDC}")
        print(">> Press any key to terminate...")
        getch()
        quit()

    while True:
        continue_choice = menu_select()
        if continue_choice:
            continue
        else:
            break

    # Close port
    portHandler.closePort()

def detect_servo_port():
    def list_serial_ports():
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    print("Please ensure the Waveshare servo driver is connected.")
    input(">> Press Enter to continue...")
    initial_ports = list_serial_ports()

    print("Available serial ports:", initial_ports)

    print("\nPlease disconnect the Waveshare servo driver.")
    input(">> Press Enter after disconnecting the servo driver...")
    
    time.sleep(2)
    updated_ports = list_serial_ports()

    disconnected_port = list(set(initial_ports) - set(updated_ports))

    if len(disconnected_port) == 1:
        servo_port = disconnected_port[0]
    elif len(disconnected_port) > 1:
        print(f"{bcolors.FAIL}\nMore than one port was disconnected. Please check connections. {bcolors.ENDC}")
        return False
    else:
        print(f"{bcolors.FAIL}\nNo port was detected as disconnected. Please try again from the main Menu. {bcolors.ENDC}")
        return False

    print(f"{bcolors.OKGREEN}\nDetected Waveshare servo driver on port: {servo_port} {bcolors.ENDC}")

    global DEVICENAME
    DEVICENAME = servo_port

    return True

def ping_servo():
    servo_id = int(input(">> Enter the servo ID: "))
    sts_model_number, sts_comm_result, sts_error = packetHandler.ping(servo_id)

    if sts_comm_result != COMM_SUCCESS:
        print(f"{bcolors.FAIL}{packetHandler.getTxRxResult(sts_comm_result)} Is the servo connected?{bcolors.ENDC}")
        return False
    else:
        print(f"{bcolors.OKGREEN}[ID: {servo_id}] ping Succeeded. STServo model number : {sts_model_number} {bcolors.ENDC}")
    if sts_error != 0:
        print(f"{bcolors.FAIL}[ID: {servo_id}] STS Error while pinging servo: {packetHandler.getRxPacketError(sts_error)} {bcolors.ENDC}")
        return False

    return True

def set_servo_id():
    old_id = int(input(">> Enter current servo ID: "))
    new_id = int(input(">> Enter new servo ID: "))
    sts_model_number, sts_comm_result, sts_error = packetHandler.ping(old_id)
    if sts_comm_result != COMM_SUCCESS:
        print(f"{bcolors.FAIL}{packetHandler.getTxRxResult(sts_comm_result)} {bcolors.ENDC}")
        return False
    else:
        print(f"{bcolors.OKGREEN}[ID: {old_id}] ping Succeeded. STServo model number : {sts_model_number} {bcolors.ENDC}")
        print(f"[ID: {old_id}] Unlocking Eprom.")
        packetHandler.unLockEprom(old_id)
        print(f"Replacing ID: {old_id} -> ID: {new_id}")
        packetHandler.setID(old_id, new_id)
        print(f"[ID: {new_id}] Locking Eprom.")
        packetHandler.LockEprom(new_id)
        sts_model_number, sts_comm_result, sts_error = packetHandler.ping(new_id)
        if sts_comm_result != COMM_SUCCESS:
            print(f"{bcolors.FAIL}[ID: {new_id}] Failed to replace ID with error: {packetHandler.getTxRxResult(sts_comm_result)} {bcolors.ENDC}")
            print(">> Press any key to go back to the main menu...")
            getch()
            return False
        else:
            print(f"{bcolors.OKGREEN}[ID: {new_id}] ID reset Succeeded. STServo model number : {sts_model_number} {bcolors.ENDC}")

        if sts_error != 0:
            print(f"{bcolors.FAIL}[ID: {new_id}] STS Error while pinging new ID: {packetHandler.getRxPacketError(sts_error)} {bcolors.ENDC}")
            print(">> Press any key to go back to the main menu...")
            getch()
            return False

        return True

def reset_servo_id():
    input("\n>> Attach servo and then press Enter.")
    for servo_id in range(1,100):
        sts_model_number, sts_comm_result, sts_error = packetHandler.ping(servo_id)
        if sts_comm_result != COMM_SUCCESS:
            print(f"{bcolors.FAIL}{packetHandler.getTxRxResult(sts_comm_result)} {bcolors.ENDC}")
            print(f"{bcolors.WARNING}Trying again with ID: {servo_id+1} {bcolors.ENDC}")
            continue
        else:
            print(f"{bcolors.OKGREEN}[ID: {servo_id}] ping Succeeded. STServo model number : {sts_model_number} {bcolors.ENDC}")
            print(f"[ID: {servo_id}] Unlocking Eprom.")
            packetHandler.unLockEprom(servo_id)
            print(f"Replacing ID: {servo_id} -> ID: 1")
            packetHandler.setID(servo_id, 1)
            print("[ID: 1] Locking Eprom.")
            packetHandler.LockEprom(1)
            sts_model_number, sts_comm_result, sts_error = packetHandler.ping(1)
            if sts_comm_result != COMM_SUCCESS:
                print(f"{bcolors.FAIL}[ID: {servo_id}] Failed to reset ID with error: {packetHandler.getTxRxResult(sts_comm_result)} {bcolors.ENDC}")
                print(">> Press any key to go back to the main menu...")
                getch()
                return False
            else:
                print(f"{bcolors.OKGREEN}[ID: {servo_id}] ID reset Succeeded. STServo model number : {sts_model_number} {bcolors.ENDC}")

            if sts_error != 0:
                print(f"{bcolors.FAIL}[ID: {servo_id}] STS Error while pinging new ID: {packetHandler.getRxPacketError(sts_error)} {bcolors.ENDC}")
                print(">> Press any key to go back to the main menu...")
                getch()
                return False

            return True

def set_all_servo_id():
    total_servo = int(input("\n>> Input total number of servos : "))

    for index in range(DEFAULT_ID, DEFAULT_ID + total_servo):
        new_id = total_servo + DEFAULT_ID - index
        if index == DEFAULT_ID:
            input(">> Attach Base servo and then press Enter.")
        else:
            input(">> Attach next servo and then press Enter.")

        sts_model_number, sts_comm_result, sts_error = packetHandler.ping(DEFAULT_ID)
        if sts_comm_result != COMM_SUCCESS:
            print(f"{bcolors.FAIL}{packetHandler.getTxRxResult(sts_comm_result)} {bcolors.ENDC}")
            return False
        else:
            print(f"{bcolors.OKGREEN}[ID: {DEFAULT_ID}] ping Succeeded. STServo model number : {sts_model_number} {bcolors.ENDC}")
            print(f"[ID:{DEFAULT_ID}] Unlocking Eprom.")
            packetHandler.unLockEprom(DEFAULT_ID)
            print(f"Replacing ID:{DEFAULT_ID} -> ID:{new_id}")
            packetHandler.setID(DEFAULT_ID, new_id)
            print(f"[ID:{new_id}] Locking Eprom.")
            packetHandler.LockEprom(DEFAULT_ID)
            sts_model_number, sts_comm_result, sts_error = packetHandler.ping(new_id)
            if sts_comm_result != COMM_SUCCESS:
                print(f"{bcolors.FAIL}[ID: {DEFAULT_ID}]Failed to replace ID with error: {packetHandler.getTxRxResult(sts_comm_result)} {bcolors.ENDC}")
                print(">> Press any key to terminate...")
                getch()
                quit()
            else:
                print(f"{bcolors.OKGREEN}[ID: {new_id}] ID replacement Succeeded. STServo model number : {sts_model_number} {bcolors.ENDC}")

            if sts_error != 0:
                print(f"{bcolors.FAIL}STS Error while pinging new ID: {packetHandler.getRxPacketError(sts_error)} {bcolors.ENDC}")
                return False

        if sts_error != 0:
            print(f"{bcolors.FAIL}STS Error while pinging new ID: {packetHandler.getRxPacketError(sts_error)} {bcolors.ENDC}")
            return False

    print(f"{bcolors.OKGREEN}Succeeded in setting all servo IDs.\n {bcolors.ENDC}")
    return True

if __name__ == "__main__":
    ASCII_art_1 = pyfiglet.figlet_format('giraffe',font='cricket')
    print(f"{bcolors.OKCYAN}\n\n\n{ASCII_art_1} {bcolors.ENDC}")
    main()
