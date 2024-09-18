/*
Ping command test to check if the servo with the corresponding ID on the bus is ready. The broadcast command is only applicable when there is a single servo on the bus.
*/

#include <iostream>
#include "SCServo.h"

SCSCL sc;

/**
 * @brief Example of using the SCSCL class to control a motor.
 *
 * @param argc The number of arguments passed to the program.
 * @param argv The arguments passed to the program.
 *
 * This example shows how to use the SCSCL class to control a motor. It
 * initializes the motor, reads the ID of the motor, and prints the ID to
 * the console. If the ID is not found, it prints an error message.
 *
 * @return 0 if the program is successful, 1 if there is an error.
 */
int main(int argc, char **argv)
{
	if(argc<2){
        std::cout<<"argc error!"<<std::endl;
        return 0;
	}
	std::cout<<"serial:"<<argv[1]<<std::endl;
    if(!sc.begin(115200, argv[1])){
        std::cout<<"Failed to init scscl motor!"<<std::endl;
        return 0;
    }
	int ID = sc.Ping(1);
	if(ID!=-1){
		std::cout<<"ID:"<<ID<<std::endl;
	}else{
		std::cout<<"Ping servo ID error!"<<std::endl;
	}
	sc.end();
	return 1;
}
