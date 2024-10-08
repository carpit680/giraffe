/*
The factory default speed of the servo is 0.0146 rpm, with a speed setting of V = 1500
*/

#include <iostream>
#include <unistd.h> // For usleep function
#include "SCServo.h"

SCSCL sc;

/**
 * @brief Example of using the SCSCL class to control multiple motors.
 *
 * This example shows how to use the SCSCL class to control multiple motors. It
 * initializes the motors, moves the motors to position 1000 with maximum speed
 * V = 1500 steps/second, and then moves the motors to position 20 with maximum
 * speed V = 1500 steps/second. The program will then loop indefinitely.
 *
 * @param argc The number of arguments passed to the program.
 * @param argv The arguments passed to the program. The first argument should be
 *             the serial port name.
 *
 * @return 0 if the program is successful, 1 if there is an error.
 */
int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <serial_port>" << std::endl;
        return 1;
    }

    std::cout << "Serial port: " << argv[1] << std::endl;
    
    if (!sc.begin(115200, argv[1])) {
        std::cout << "Failed to initialize SCSCL motor!" << std::endl;
        return 1;
    }

    while (true) {
        sc.RegWritePos(1, 1000, 0, 1500); // Move servo ID1 to position 1000
        sc.RegWritePos(2, 1000, 0, 1500); // Move servo ID2 to position 1000
        sc.RegWriteAction();
        std::cout << "Position = 1000" << std::endl;
        usleep(754 * 1000); // Wait for calculated time

        sc.RegWritePos(1, 20, 0, 1500); // Move servo ID1 to position 20
        sc.RegWritePos(2, 20, 0, 1500); // Move servo ID2 to position 20
        sc.RegWriteAction();
        std::cout << "Position = 20" << std::endl;
        usleep(754 * 1000); // Wait for calculated time
    }

    sc.end();
    return 0;
}
