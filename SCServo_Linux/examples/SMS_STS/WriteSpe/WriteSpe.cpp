/*
舵机出厂速度单位是0.0146rpm，速度改为V=2400
*/


#include <iostream>
#include "SCServo.h"

SMS_STS sm_st;

int main(int argc, char **argv)
{
	if(argc<2){
        std::cout<<"argc error!"<<std::endl;
        return 0;
	}
	std::cout<<"serial:"<<argv[1]<<std::endl;
    if(!sm_st.begin(115200, argv[1])){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
    }
	sm_st.WheelMode(1);//恒速模式
	std::cout<<"mode = "<<1<<std::endl;
	while(1){
		sm_st.WriteSpe(1, 2400, 50);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，旋转
		std::cout<<"speed = "<<2400<<std::endl;
		sleep(2);
		sm_st.WriteSpe(1, 0, 50);//舵机(ID1)以加速度A=50(50*100步/秒^2)，停止旋转(V=0)
		std::cout<<"speed = "<<0<<std::endl;
		sleep(2);
		sm_st.WriteSpe(1, -2400, 50);//舵机(ID1)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，反向旋转
		std::cout<<"speed = "<<-2400<<std::endl;
		sleep(2);
		sm_st.WriteSpe(1, 0, 50);//舵机(ID1)以加速度A=50(50*100步/秒^2)，停止旋转(V=0)
		std::cout<<"speed = "<<0<<std::endl;
		sleep(2);
	}
	sm_st.end();
	return 1;
}

