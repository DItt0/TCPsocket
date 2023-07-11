#pragma once

#include<stdio.h>
#include<iostream>
#include<string>
#include<thread>
#include <sstream>
#include <iomanip>
#include<vector>
#include<mutex>
#include<condition_variable>
#include<WinSock2.h>
#include <Ws2tcpip.h>
#include<Windows.h>
#include<fstream>
#pragma comment(lib,"WS2_32.lib")
#pragma comment(lib,"WSOCK32.lib")

using namespace std;

enum DEVICE_STATE
{
	//初始状态
	INIT_STATE = 100,
	//命令应答状态
	COMMAND_ANSWER_STATE,
	//姿态应答状态
	ATTITUDE_RESPONSE_STATE,
	//参数应答状态
	PARAMETER_RESPONSE_STATE,
};

typedef void(*parseCommandCallback)(std::string val, void* pointer);
typedef void(*floatDataCallback)(float* data, void* pointer);

typedef struct S_STEP_INFO{
	char  		c_motor;
	uint8_t   m_delay_ms;
	char  		c_execute;
	uint8_t   	reserved;
	long      m_dest_position;
}StepSequence;

class MultiAxisControl {
public:
	MultiAxisControl();
	~MultiAxisControl();

	//Function
	//连接控制器
	bool connect();
	//断开连接
	bool disconnect();
	//发送
	bool sendCommand(std::string sendBuf);
	//接收
	int receiveCommand(unsigned char* buffer);
	//解析
	std::string parseCommand(unsigned char* buffer);
	//计算校验位
	//char数组方式
	void excuteCheck(const char* identifier,char commandCheck[]);
	//计算校验位
	//string拼接方式
	std::string excuteCheck(std::string identifier);
	//将char类型转成2位16进制string输出
	std::string IPToHex(const char* c);
	//将int转成4位16进制string输出
	std::string intToHex(int c);
	std::string convertMacAddress(const std::string& mac);
	//从缓冲池中寻找指令头和尾
	int findHead(unsigned char* buffer);
	int findTail(unsigned char* buffer);
	//应答状态判断
	bool WaitReplay(DEVICE_STATE state, int timeout);
	void setCallBack(parseCommandCallback call, void* pPointer);
	void run();
	//整数转为16进制字符串,第二个参数为字节数*2
	std::string intToHex(int num, int digit);
	void intToHex(char* Distination, int x, int digit);
	//将步序列字符数组转码,移动距离按LSB格式
	int stepSequenceTranse(uint8_t* stepCommand, int step, StepSequence* stepInfo);
	std::string excuteStepCheck(uint8_t* command,int totalNum);

	//Interface
	//接收接口
	//获取控制器接收并将接收进行解析
	void getReceive();
	//发送接口
	//控制方式设置 1:手动控制模式	2:指令控制模式
	bool sendSCMDE(const char* on);	
	//工作模式设置 1:标定模式	2:测量模式
	bool sendSWRKP(const char* on);	
	/*
	系统各轴零位设置
	控制轴编号：X，Y，Z，R，P，H		
	系统零点参数，6位字符型，带符号位
	X 轴零位设置范围-10000~10000，单位 0.01 毫米【0.01mm】。默认为 0。 
	Y 轴零位设置范围-32500~32500，单位 0.01 毫米【0.01mm】。默认为 0。 
	Z 轴零位设置范围-15000~15000，单位 0.01 毫米【0.01mm】。默认为 0。 
	R 轴零位设置范围 00000~36000，单位 0.01 度【0.01°】。默认为 0。 
	P 轴零位设置范围-03000~03000，单位 0.01 度【0.01°】。默认为 0。 
	H 轴零位设置范围-18000~18000，单位 0.01 度【0.01°】。默认为 0。*/
	bool sendSSCZP(char Axis,float sysZeroPara);	
	/*
	零位校准 
	控制轴编号：X，Y，Z，R，P，H*/
	bool sendBTOSZ(char Axis);	
	/*
	系统各轴绝对运动位移设置
	X 轴零位设置范围-10000~10000，单位 0.01 毫米【0.01mm】。默认为 0。
	Y 轴零位设置范围-32500~32500，单位 0.01 毫米【0.01mm】。默认为 0。
	Z 轴零位设置范围-15000~15000，单位 0.01 毫米【0.01mm】。默认为 0。
	R 轴零位设置范围 00000~36000，单位 0.01 度【0.01°】。默认为 0。
	P 轴零位设置范围-03000~03000，单位 0.01 度【0.01°】。默认为 0。
	H 轴零位设置范围-18000~18000，单位 0.01 度【0.01°】。默认为 0。*/
	bool sendSABMD(char Axis, float sysRunPara);
	/*系统各轴点动控制指令
	C为顺时针	A为逆时针	N为关闭*/
	bool sendJOGCC(char Axis,char state);	
	//系统姿态查询指令
	bool sendSAASK();
	/*系统步序列控制
	移动总步数
	步序列操作*/
	bool sendCSTEP(int steps, StepSequence* stepInfo);
	//步序列控制开始
	bool sendSCSTA();
	//步序列控制停止
	bool sendSCSTP();
	//步序列控制暂停
	bool sendSCHLD();
	/*
	系统各轴回零速度设置
	控制轴编号：X，Y，Z，R，P，H
	回零速度参数
	X 轴的回零速度设置范围 1~300.0，单位 0.1 毫米/秒【0.1mm/s】。默认100 毫米/秒，上位机发给下位机的数值范围 10~3000。 
	Y 轴的回零速度设置范围 1~300.0，单位 0.1 毫米/秒【0.1mm/s】。默认100 毫米/秒，上位机发给下位机的数值范围 10~3000。 
	Z 轴的回零速度设置范围 1~300.0，单位 0.1 毫米/秒【0.1mm/s】。默认100 毫米/秒，上位机发给下位机的数值范围 10~3000。 
	R 轴的回零速度设置范围 0.1~60.0，单位 0.1 度/秒【0.1°/s】。默认 15° /秒，上位机发给下位机的数值范围 1~600。 
	P 轴的回零速度设置范围 0.1~60.0，单位 0.1 度/秒【0.1°/s】。默认 15° /秒，上位机发给下位机的数值范围 1~600。 
	H 轴的回零速度设置范围 0.1~60.0，单位 0.1 度/秒【0.1°/s】。默认 15° /秒，上位机发给下位机的数值范围 1~600。*/
	bool sendSZSPD(char Axis,float toZeroSpeed);	
	/*系统各轴运动速度设置
	控制轴编号：X，Y，Z，R，P，H
	运行速度参数
	X 轴的运动速度设置范围 1~300.0，单位 0.1 毫米/秒【0.1mm/s】。默认100 毫米/秒，上位机发给下位机的数值范围 10~3000。 
	Y 轴的运动速度设置范围 1~300.0，单位 0.1 毫米/秒【0.1mm/s】。默认100 毫米/秒，上位机发给下位机的数值范围 10~3000。 
	Z 轴的运动速度设置范围 1~300.0，单位 0.1 毫米/秒【0.1mm/s】。默认100 毫米/秒，上位机发给下位机的数值范围 10~3000。 
	R 轴的运动速度设置范围 0.1~60.0，单位 0.1 度/秒【0.1°/s】。默认 15° /秒，上位机发给下位机的数值范围 1~600。 
	P 轴的运动速度设置范围 0.1~60.0，单位 0.1 度/秒【0.1°/s】。默认 15° /秒，上位机发给下位机的数值范围 1~600。 
	H 轴的运动速度设置范围 0.1~60.0，单位 0.1 度/秒【0.1°/s】。默认 15° /秒，上位机发给下位机的数值范围 1~600。*/
	bool sendSRSPD(char Axis,float runSpeed);	
	//设置参数读取
	bool sendCOFCR();
	/*设置参数存储
	01表示网络配置参数	02表示其他参数*/
	bool sendCOFCS(int parameter);	
	//恢复出厂
	bool sendRSTFS();
	/*系统各轴软限位角度值设置
	下限参数值 上限参数值 4位带符号位
	X 轴软限位范围：-100mm~＋100mm； 
	Y 轴软限位范围：-325mm~＋325mm； 
	Z 轴软限位范围：-150mm~＋150mm； 
	P 轴软限位范围：-30°~＋30°； 
	R 轴软限位范围：无软件限位；
	H 轴软限位范围：-180°~＋180°；*/
	bool sendSSLAG(char Axis,int lowLimitV,int highLimitV);	
	/*TCP通讯重传次数设置
	三位字符型，001~255，默认为8*/
	bool sendTCPCT(int parameter);	
	/*系统各轴使能控制
	Y:使能		N:失能*/
	bool sendMOTOR(char Axis, char state);	
	/*系统各轴强制使能
	Y:强制使能开		N:强制失能关*/
	bool sendFEMOT(char Axis, char state);	
	//控制系统复位
	bool sendRSMCU();
	//设备ip设置
	bool sendSDIPA(char* ipAddress);	//ip地址的16进制	C0A80001:192.168.0.1
	//设备端口号设置
	bool sendSDPRT(int port);	//端口号16进制	083F:2111
	//设备MAC地址设置
	bool sendSDMAC(char* macAddress);	//MAC地址
	//查询通用参数接口
	//运动速度
	float* getMotionVelocity(unsigned char* buf, int head);
	//回零速度
	float* getZeroVelocity(unsigned char* buf, int head);
	//系统零位
	float* getSystemZeroSite(unsigned char* buf, int head);
	//软上下限位
	int* getLimit(unsigned char* buf, int head);
	//TCP重传设置
	int getTCPReTransNum(unsigned char* buf, int head);
	//MAC地址
	std::string getMACAddress(unsigned char* buf, int head);
//	void function();

private:
	//ip地址
	const char *ipAdress[20];
	//端口号
	int port;
	//阻塞信号
	int obstractSignal;
	//缓冲区存储应答的数量
	int front;
	//缓冲池大小
	int bufferLen = 1024 * 1024;
	//接收缓冲池
	unsigned char* buffer = new unsigned  char[bufferLen];	
	//发送区缓冲池
//	std::string sendBuf;
	//指令头
	std::string orderHead = "$";
	//校验位
	char commandCheck[5];
	//指令体
	std::string identifier = "";
	//校验位
	std::string check = "";
	//线程开关
	std::thread* recvThread = NULL;
	//链接状态字
	bool bConnect;

	//监听接收线程信号
	bool bThreadRecvRun;
	int state;
	//网络空间
	WSADATA wsa;
	//客户端套接字
	SOCKET client;
	std::string* retMsg = new std::string[5];
	//枚举对象
	DEVICE_STATE nowState;
	std::string abcMsg[100];
	//回调
	parseCommandCallback cFunc;
	floatDataCallback fFunc;
	//回调指针
	void *pPointer;

};

char intToHexChar(int value);