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

	sc.unLockEprom(1);//打开EPROM保存功能
	std::cout<<"unLock Eprom"<<std::endl;
	sc.writeByte(1, SCSCL_ID, 2);//ID
	std::cout<<"write ID:"<<2<<std::endl;
	sc.writeWord(2, SCSCL_MIN_ANGLE_LIMIT_L, 20);
	std::cout<<"write min angle limit:"<<20<<std::endl;
	sc.writeWord(2, SCSCL_MAX_ANGLE_LIMIT_L, 1000);
	std::cout<<"write max angle limit:"<<1000<<std::endl;
	sc.LockEprom(2);////关闭EPROM保存功能
	std::cout<<"Lock Eprom"<<std::endl;
	sc.end();
	return 1;
}

