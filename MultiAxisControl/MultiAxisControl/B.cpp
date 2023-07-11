#include "B.h"


B::B()
{
}


B::~B()
{
}



void B::test(std::string str, void* pointer) {
	B *pUser = (B*)pointer;
	if (pUser == nullptr)
		return;
	std::cout << str<<std::endl;
}

void B::start()
{
	mac.setCallBack(test, (void*)this);
//	mac.run();
}
