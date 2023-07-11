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
	//��ʼ״̬
	INIT_STATE = 100,
	//����Ӧ��״̬
	COMMAND_ANSWER_STATE,
	//��̬Ӧ��״̬
	ATTITUDE_RESPONSE_STATE,
	//����Ӧ��״̬
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
	//���ӿ�����
	bool connect();
	//�Ͽ�����
	bool disconnect();
	//����
	bool sendCommand(std::string sendBuf);
	//����
	int receiveCommand(unsigned char* buffer);
	//����
	std::string parseCommand(unsigned char* buffer);
	//����У��λ
	//char���鷽ʽ
	void excuteCheck(const char* identifier,char commandCheck[]);
	//����У��λ
	//stringƴ�ӷ�ʽ
	std::string excuteCheck(std::string identifier);
	//��char����ת��2λ16����string���
	std::string IPToHex(const char* c);
	//��intת��4λ16����string���
	std::string intToHex(int c);
	std::string convertMacAddress(const std::string& mac);
	//�ӻ������Ѱ��ָ��ͷ��β
	int findHead(unsigned char* buffer);
	int findTail(unsigned char* buffer);
	//Ӧ��״̬�ж�
	bool WaitReplay(DEVICE_STATE state, int timeout);
	void setCallBack(parseCommandCallback call, void* pPointer);
	void run();
	//����תΪ16�����ַ���,�ڶ�������Ϊ�ֽ���*2
	std::string intToHex(int num, int digit);
	void intToHex(char* Distination, int x, int digit);
	//���������ַ�����ת��,�ƶ����밴LSB��ʽ
	int stepSequenceTranse(uint8_t* stepCommand, int step, StepSequence* stepInfo);
	std::string excuteStepCheck(uint8_t* command,int totalNum);

	//Interface
	//���սӿ�
	//��ȡ���������ղ������ս��н���
	void getReceive();
	//���ͽӿ�
	//���Ʒ�ʽ���� 1:�ֶ�����ģʽ	2:ָ�����ģʽ
	bool sendSCMDE(const char* on);	
	//����ģʽ���� 1:�궨ģʽ	2:����ģʽ
	bool sendSWRKP(const char* on);	
	/*
	ϵͳ������λ����
	�������ţ�X��Y��Z��R��P��H		
	ϵͳ��������6λ�ַ��ͣ�������λ
	X ����λ���÷�Χ-10000~10000����λ 0.01 ���ס�0.01mm����Ĭ��Ϊ 0�� 
	Y ����λ���÷�Χ-32500~32500����λ 0.01 ���ס�0.01mm����Ĭ��Ϊ 0�� 
	Z ����λ���÷�Χ-15000~15000����λ 0.01 ���ס�0.01mm����Ĭ��Ϊ 0�� 
	R ����λ���÷�Χ 00000~36000����λ 0.01 �ȡ�0.01�㡿��Ĭ��Ϊ 0�� 
	P ����λ���÷�Χ-03000~03000����λ 0.01 �ȡ�0.01�㡿��Ĭ��Ϊ 0�� 
	H ����λ���÷�Χ-18000~18000����λ 0.01 �ȡ�0.01�㡿��Ĭ��Ϊ 0��*/
	bool sendSSCZP(char Axis,float sysZeroPara);	
	/*
	��λУ׼ 
	�������ţ�X��Y��Z��R��P��H*/
	bool sendBTOSZ(char Axis);	
	/*
	ϵͳ��������˶�λ������
	X ����λ���÷�Χ-10000~10000����λ 0.01 ���ס�0.01mm����Ĭ��Ϊ 0��
	Y ����λ���÷�Χ-32500~32500����λ 0.01 ���ס�0.01mm����Ĭ��Ϊ 0��
	Z ����λ���÷�Χ-15000~15000����λ 0.01 ���ס�0.01mm����Ĭ��Ϊ 0��
	R ����λ���÷�Χ 00000~36000����λ 0.01 �ȡ�0.01�㡿��Ĭ��Ϊ 0��
	P ����λ���÷�Χ-03000~03000����λ 0.01 �ȡ�0.01�㡿��Ĭ��Ϊ 0��
	H ����λ���÷�Χ-18000~18000����λ 0.01 �ȡ�0.01�㡿��Ĭ��Ϊ 0��*/
	bool sendSABMD(char Axis, float sysRunPara);
	/*ϵͳ����㶯����ָ��
	CΪ˳ʱ��	AΪ��ʱ��	NΪ�ر�*/
	bool sendJOGCC(char Axis,char state);	
	//ϵͳ��̬��ѯָ��
	bool sendSAASK();
	/*ϵͳ�����п���
	�ƶ��ܲ���
	�����в���*/
	bool sendCSTEP(int steps, StepSequence* stepInfo);
	//�����п��ƿ�ʼ
	bool sendSCSTA();
	//�����п���ֹͣ
	bool sendSCSTP();
	//�����п�����ͣ
	bool sendSCHLD();
	/*
	ϵͳ��������ٶ�����
	�������ţ�X��Y��Z��R��P��H
	�����ٶȲ���
	X ��Ļ����ٶ����÷�Χ 1~300.0����λ 0.1 ����/�롾0.1mm/s����Ĭ��100 ����/�룬��λ��������λ������ֵ��Χ 10~3000�� 
	Y ��Ļ����ٶ����÷�Χ 1~300.0����λ 0.1 ����/�롾0.1mm/s����Ĭ��100 ����/�룬��λ��������λ������ֵ��Χ 10~3000�� 
	Z ��Ļ����ٶ����÷�Χ 1~300.0����λ 0.1 ����/�롾0.1mm/s����Ĭ��100 ����/�룬��λ��������λ������ֵ��Χ 10~3000�� 
	R ��Ļ����ٶ����÷�Χ 0.1~60.0����λ 0.1 ��/�롾0.1��/s����Ĭ�� 15�� /�룬��λ��������λ������ֵ��Χ 1~600�� 
	P ��Ļ����ٶ����÷�Χ 0.1~60.0����λ 0.1 ��/�롾0.1��/s����Ĭ�� 15�� /�룬��λ��������λ������ֵ��Χ 1~600�� 
	H ��Ļ����ٶ����÷�Χ 0.1~60.0����λ 0.1 ��/�롾0.1��/s����Ĭ�� 15�� /�룬��λ��������λ������ֵ��Χ 1~600��*/
	bool sendSZSPD(char Axis,float toZeroSpeed);	
	/*ϵͳ�����˶��ٶ�����
	�������ţ�X��Y��Z��R��P��H
	�����ٶȲ���
	X ����˶��ٶ����÷�Χ 1~300.0����λ 0.1 ����/�롾0.1mm/s����Ĭ��100 ����/�룬��λ��������λ������ֵ��Χ 10~3000�� 
	Y ����˶��ٶ����÷�Χ 1~300.0����λ 0.1 ����/�롾0.1mm/s����Ĭ��100 ����/�룬��λ��������λ������ֵ��Χ 10~3000�� 
	Z ����˶��ٶ����÷�Χ 1~300.0����λ 0.1 ����/�롾0.1mm/s����Ĭ��100 ����/�룬��λ��������λ������ֵ��Χ 10~3000�� 
	R ����˶��ٶ����÷�Χ 0.1~60.0����λ 0.1 ��/�롾0.1��/s����Ĭ�� 15�� /�룬��λ��������λ������ֵ��Χ 1~600�� 
	P ����˶��ٶ����÷�Χ 0.1~60.0����λ 0.1 ��/�롾0.1��/s����Ĭ�� 15�� /�룬��λ��������λ������ֵ��Χ 1~600�� 
	H ����˶��ٶ����÷�Χ 0.1~60.0����λ 0.1 ��/�롾0.1��/s����Ĭ�� 15�� /�룬��λ��������λ������ֵ��Χ 1~600��*/
	bool sendSRSPD(char Axis,float runSpeed);	
	//���ò�����ȡ
	bool sendCOFCR();
	/*���ò����洢
	01��ʾ�������ò���	02��ʾ��������*/
	bool sendCOFCS(int parameter);	
	//�ָ�����
	bool sendRSTFS();
	/*ϵͳ��������λ�Ƕ�ֵ����
	���޲���ֵ ���޲���ֵ 4λ������λ
	X ������λ��Χ��-100mm~��100mm�� 
	Y ������λ��Χ��-325mm~��325mm�� 
	Z ������λ��Χ��-150mm~��150mm�� 
	P ������λ��Χ��-30��~��30�㣻 
	R ������λ��Χ���������λ��
	H ������λ��Χ��-180��~��180�㣻*/
	bool sendSSLAG(char Axis,int lowLimitV,int highLimitV);	
	/*TCPͨѶ�ش���������
	��λ�ַ��ͣ�001~255��Ĭ��Ϊ8*/
	bool sendTCPCT(int parameter);	
	/*ϵͳ����ʹ�ܿ���
	Y:ʹ��		N:ʧ��*/
	bool sendMOTOR(char Axis, char state);	
	/*ϵͳ����ǿ��ʹ��
	Y:ǿ��ʹ�ܿ�		N:ǿ��ʧ�ܹ�*/
	bool sendFEMOT(char Axis, char state);	
	//����ϵͳ��λ
	bool sendRSMCU();
	//�豸ip����
	bool sendSDIPA(char* ipAddress);	//ip��ַ��16����	C0A80001:192.168.0.1
	//�豸�˿ں�����
	bool sendSDPRT(int port);	//�˿ں�16����	083F:2111
	//�豸MAC��ַ����
	bool sendSDMAC(char* macAddress);	//MAC��ַ
	//��ѯͨ�ò����ӿ�
	//�˶��ٶ�
	float* getMotionVelocity(unsigned char* buf, int head);
	//�����ٶ�
	float* getZeroVelocity(unsigned char* buf, int head);
	//ϵͳ��λ
	float* getSystemZeroSite(unsigned char* buf, int head);
	//��������λ
	int* getLimit(unsigned char* buf, int head);
	//TCP�ش�����
	int getTCPReTransNum(unsigned char* buf, int head);
	//MAC��ַ
	std::string getMACAddress(unsigned char* buf, int head);
//	void function();

private:
	//ip��ַ
	const char *ipAdress[20];
	//�˿ں�
	int port;
	//�����ź�
	int obstractSignal;
	//�������洢Ӧ�������
	int front;
	//����ش�С
	int bufferLen = 1024 * 1024;
	//���ջ����
	unsigned char* buffer = new unsigned  char[bufferLen];	
	//�����������
//	std::string sendBuf;
	//ָ��ͷ
	std::string orderHead = "$";
	//У��λ
	char commandCheck[5];
	//ָ����
	std::string identifier = "";
	//У��λ
	std::string check = "";
	//�߳̿���
	std::thread* recvThread = NULL;
	//����״̬��
	bool bConnect;

	//���������߳��ź�
	bool bThreadRecvRun;
	int state;
	//����ռ�
	WSADATA wsa;
	//�ͻ����׽���
	SOCKET client;
	std::string* retMsg = new std::string[5];
	//ö�ٶ���
	DEVICE_STATE nowState;
	std::string abcMsg[100];
	//�ص�
	parseCommandCallback cFunc;
	floatDataCallback fFunc;
	//�ص�ָ��
	void *pPointer;

};

char intToHexChar(int value);