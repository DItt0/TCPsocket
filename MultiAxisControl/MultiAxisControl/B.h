#pragma once

#include "MultiAxisControl.h"

class B
{
public:
	B();
	~B();

private:
	MultiAxisControl mac;
public:
	static void test(/*bool status,*/ std::string str, void* pointer);
	void start();
};

