/*
The factory default speed of the servo is 0.0146 rpm, with a speed setting of V = 1500
*/

#include <iostream>
#include "SCServo.h"

SCSCL sc;

u8 ID[2] = {1, 2};
u16 Position[2];
u16 Speed[2];

/**
 * @brief Example of using the SCSCL class to control multiple motors.
 *
 * This example shows how to use the SCSCL class to control multiple motors. It
 * initializes the motors, moves the motors to position 1000 with maximum speed
 * V = 1500 steps/second, and then moves the motors to position 20 with maximum
 * speed V = 1500 steps/second.
 *
 * @param argc The number of arguments passed to the program.
 * @param argv The arguments passed to the program. The first argument should be
 *             the serial port name.
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
		Position[0] = 1000;
		Position[1] = 1000;
		Speed[0] = 1500;
		Speed[1] = 1500;
		sc.SyncWritePos(ID, 2, Position, 0, Speed); // Move servos (ID1/ID2) to position 1000 with maximum speed V = 1500 steps/second
		std::cout << "pos = " << 1000 << std::endl;
		usleep(754 * 1000); // [(P1-P0)/V]*1000 + 100

		Position[0] = 20;
		Position[1] = 20;
		Speed[0] = 1500;
		Speed[1] = 1500;
		sc.SyncWritePos(ID, 2, Position, 0, Speed); // Move servos (ID1/ID2) to position 20 with maximum speed V = 1500 steps/second
		std::cout << "pos = " << 20 << std::endl;
		usleep(754 * 1000); // [(P1-P0)/V]*1000 + 100
	}
	sc.end();
	return 1;
}
