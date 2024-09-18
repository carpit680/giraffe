#include <iostream>
#include "SCServo.h"

SCSCL sc;

/**
 * @brief Example of using the SCSCL class to control a motor in PWM mode.
 *
 * The program takes one command line argument, the serial port name.
 * The program will then continuously set the PWM of the motor to 500, -500, and 0.
 *
 * @param argc The number of arguments passed to the program.
 * @param argv The arguments passed to the program. The first argument should be the serial port name.
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
	sc.PWMMode(1);
	std::cout<<"mode = "<<1<<std::endl;
	while(1){
		sc.WritePWM(1, 500);
		std::cout<<"pwm = "<<500<<std::endl;
		sleep(2);
		sc.WritePWM(1, 0);
		std::cout<<"pwm = "<<0<<std::endl;
		sleep(2);
		sc.WritePWM(1, -500);
		std::cout<<"pwm = "<<-500<<std::endl;
		sleep(2);
		sc.WritePWM(1,0);
		std::cout<<"pwm = "<<0<<std::endl;
		sleep(2);
	}
	sc.end();
	return 1;
}

