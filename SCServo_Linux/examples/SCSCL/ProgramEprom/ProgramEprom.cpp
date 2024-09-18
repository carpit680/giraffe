#include <iostream>
#include "SCServo.h"

SCSCL sc;


/**
 * @brief Example of using the SCSCL class to write EPROM data.
 *
 * This example shows how to use the SCSCL class to write data to the EPROM of a motor.
 * It first enables the EPROM save function, sets the ID of the motor, and then writes
 * the minimum and maximum angle limits. Finally, it disables the EPROM save function.
 *
 * @param argc The number of arguments passed to the program.
 * @param argv The arguments passed to the program. The first argument should be the
 *             serial port name.
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

	sc.unLockEprom(1); // Enable EPROM save function
	std::cout<<"unLock Eprom"<<std::endl;
	sc.writeByte(1, SCSCL_ID, 2);//ID
	std::cout<<"write ID:"<<2<<std::endl;
	sc.writeWord(2, SCSCL_MIN_ANGLE_LIMIT_L, 20);
	std::cout<<"write min angle limit:"<<20<<std::endl;
	sc.writeWord(2, SCSCL_MAX_ANGLE_LIMIT_L, 1000);
	std::cout<<"write max angle limit:"<<1000<<std::endl;
	sc.LockEprom(2); // Disable EPROM save function
	std::cout<<"Lock Eprom"<<std::endl;
	sc.end();
	return 1;
}

