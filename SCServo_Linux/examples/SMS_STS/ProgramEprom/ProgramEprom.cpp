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

	sm_st.unLockEprom(1);//打开EPROM保存功能
	std::cout<<"unLock Eprom"<<std::endl;
	sm_st.writeByte(1, SMSBL_ID, 2);//ID
	std::cout<<"write ID:"<<2<<std::endl;
	sm_st.LockEprom(2);////关闭EPROM保存功能
	std::cout<<"Lock Eprom"<<std::endl;
	sm_st.end();
	return 1;
}

