/*
The factory default speed of the servo is 0.0146 rpm, with a speed setting of V = 1500
*/

#include <iostream>
#include "SCServo.h"

SCSCL sc;

/**
 * @brief Example of using the SCSCL class to control a motor.
 *
 * This example shows how to use the SCSCL class to control a motor. It
 * initializes the motor, moves the motor to position 1000 with maximum
 * speed V = 1500 steps/second, and then moves the motor to position 20
 * with maximum speed V = 1500 steps/second.
 *
 * @param argc The number of arguments passed to the program.
 * @param argv The arguments passed to the program. The first argument
 *             should be the serial port name.
 *
 * @return 0 if the program is successful, 1 if there is an error.
 */
int main(int argc, char **argv)
{
	if (argc < 2)
	{
		std::cout << "argc error!" << std::endl;
		return 0;
	}
	std::cout << "serial: " << argv[1] << std::endl;
	if (!sc.begin(115200, argv[1]))
	{
		std::cout << "Failed to init scscl motor!" << std::endl;
		return 0;
	}
	while (1)
	{
		sc.WritePos(1, 1000, 0, 1500); // Move servo (ID1) to position 1000 with maximum speed V = 1500 steps/second
		std::cout << "pos = " << 1000 << std::endl;
		usleep(754 * 1000); // [(P1-P0)/V]*1000 + 100

		sc.WritePos(1, 20, 0, 1500); // Move servo (ID1) to position 20 with maximum speed V = 1500 steps/second
		std::cout << "pos = " << 20 << std::endl;
		usleep(754 * 1000); // [(P1-P0)/V]*1000 + 100
	}
	sc.end();
	return 1;
}
