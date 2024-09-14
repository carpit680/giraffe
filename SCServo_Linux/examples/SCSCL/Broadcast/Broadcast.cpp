/*
舵机出厂速度单位是0.0146rpm，速度为V=1500
*/

#include <iostream>
#include "SCServo.h"

SCSCL sc;

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
	while(1){
		sc.WritePos(0xfe, 1000, 0, 1500);//舵机(ID1)以最高速度V=1500步/秒,运行至P1=1000
		std::cout<<"pos = "<<1000<<std::endl;
		usleep(754*1000);//[(P1-P0)/V]*1000+100
  
		sc.WritePos(0xfe, 20, 0, 1500);//舵机(ID1)以最高V=1500步/秒,运行至P1=20
		std::cout<<"pos = "<<20<<std::endl;
		usleep(754*1000);//[(P1-P0)/V]*1000+100
	}
	sc.end();
	return 1;
}

