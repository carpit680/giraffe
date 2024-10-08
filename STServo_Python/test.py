import serial
import time

# Serial port configuration (adjust 'COM3' or '/dev/ttyUSB0' based on your system)
SERIAL_PORT = '/dev/tty.usbmodem585A0080511'  # Replace with your port (Windows: 'COMx', Linux: '/dev/ttyUSBx')
BAUD_RATE = 1000000     # According to the servo specs (confirm this from the manual)

# Servo ID and protocol details
SERVO_ID = 1  # Replace with the ID of your servo (1-254)
CONTINUOUS_ROTATION_SPEED = 1023  # Set the speed (-1023 to 1023 for full speed control)

# Define function to create command packet
def create_command_packet(servo_id, command, parameters):
    """
    Create the command packet following the Waveshare communication protocol.
    """
    packet_length = len(parameters) + 2
    checksum = (servo_id + packet_length + command + sum(parameters)) & 0xFF
    checksum = (~checksum) & 0xFF  # Calculate the checksum
    
    packet = [0x55, 0x55]  # Header
    packet += [servo_id, packet_length, command]
    packet += parameters
    packet.append(checksum)
    
    print(f"Command packet: {packet}")
    return bytearray(packet)

# Function to send command to servo
def send_command(serial_port, command_packet):
    serial_port.write(command_packet)
    time.sleep(0.01)  # Give the servo time to respond (adjust delay if necessary)
    print(f"Sent command packet: {command_packet.hex()}")

# Function to control servo speed for continuous rotation
def run_servo_continuous(serial_port, servo_id, speed):
    """
    Send command to run servo continuously at a given speed.
    Speed range is from -1023 (full reverse) to 1023 (full forward).
    """
    direction = 0 if speed >= 0 else 1  # 0 for forward, 1 for reverse
    speed = abs(speed) & 0x03FF  # Limit speed value to 10 bits (0 to 1023)
    
    low_byte = speed & 0xFF
    high_byte = (speed >> 8) & 0xFF
    
    parameters = [direction, low_byte, high_byte]
    
    # Command 1 specifies control of motor speed (based on Waveshare protocol)
    command_packet = create_command_packet(servo_id, 0x01, parameters)
    
    send_command(serial_port, command_packet)

# Main function to run the servo
def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print("Serial connection established.")
            while True:
                # Run the servo continuously at defined speed
                run_servo_continuous(ser, SERVO_ID, CONTINUOUS_ROTATION_SPEED)
                time.sleep(2)  # Adjust timing as necessary
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except KeyboardInterrupt:
        print("Program interrupted by user.")

if __name__ == '__main__':
    main()
