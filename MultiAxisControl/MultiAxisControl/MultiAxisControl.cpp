#include"MultiAxisControl.h"

MultiAxisControl::MultiAxisControl() {
//	*ipAdress = "192.168.100.101";
	*ipAdress = "172.16.1.23";
	port = 8080;
	//1Ϊ��������0Ϊ����
	obstractSignal = 1;
	memset(buffer, 0, sizeof(unsigned char) * 1024 * 1024);
	front = 0;
	bThreadRecvRun = false;
	bConnect = false;
	nowState = INIT_STATE;
	cFunc = nullptr;
	pPointer = nullptr;
}

MultiAxisControl::~MultiAxisControl() {

}

bool MultiAxisControl::connect() {
	//��������ռ�
	if ((WSAStartup(MAKEWORD(2, 2), &wsa)) == -1) {
//		std::cout << "WSA ERROR" << WSAGetLastError() << std::endl;
		WSACleanup();
		return false;
	}
	//�����׽���
	if ((client = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == INVALID_SOCKET) {
//		std::cout << "socket ERROR" << WSAGetLastError() << std::endl;
		WSACleanup();
		return false;
	}
	//���÷�����ģʽ
	//ioctlsocket(client, FIONBIO, (u_long FAR*)obstractSignal);

	//�ͻ��˵�ַ
	SOCKADDR_IN clientAddr;
	clientAddr.sin_family = AF_INET;
	//����ip��ַ
	inet_pton(AF_INET, *ipAdress, &clientAddr.sin_addr.s_addr);
	//���ö˿ں�
	clientAddr.sin_port = htons(port);

	struct timeval tm;
	tm.tv_sec = 1;
	tm.tv_usec = 0;
	int ret = -1;
	
	//���ӷ����
	if ((::connect(client, (SOCKADDR*)&clientAddr, sizeof(clientAddr))) == SOCKET_ERROR) {
//		std::cout << "connect ERROR" << WSAGetLastError() << std::endl;
		closesocket(client);
		WSACleanup();
		return false;
	}
/*	fd_set set;
	FD_ZERO(&set);
	FD_SET(client, &set);
	if (select(-1, NULL, &set, NULL, &tm) <= 0)
		ret = -1;
	else {
		int error = -1;
		int optLen = sizeof(int);
		
		getsockopt(client, SOL_SOCKET, SO_ERROR, (char*)&error, &optLen);
		error == 0 ? ret = 1 : ret = -1;
	}
	if (ret == -1) {
		disconnect();
		return false;
	}*/

	bConnect = true;
	//�����߳̽��ж�ȡ
	bThreadRecvRun = true;
	recvThread = new std::thread(&MultiAxisControl::getReceive, this);
	recvThread->detach();
	return true;
}

void MultiAxisControl::run() {
	//�����߳̽��ж�ȡ
	bThreadRecvRun = true;
	recvThread = new std::thread(&MultiAxisControl::getReceive, this);
	recvThread->detach();
}

bool MultiAxisControl::disconnect()
{
	if (!bConnect)
		return false;

	recvThread = NULL;
	if (::closesocket(this->client) == SOCKET_ERROR) {
		std::cout << "close error" << std::endl;
	}
	if (WSACleanup() == SOCKET_ERROR) {
		std::cout << "WSACleanup error" << std::endl;
	}

	bThreadRecvRun = false;
	std::cout << "�����ѶϿ�" << std::endl;
	return true;
}

void MultiAxisControl::setCallBack(parseCommandCallback call,void* pintor) {
	cFunc = call;
	pPointer = pintor;
}

bool MultiAxisControl::sendCommand(std::string sendBuf) {
	//���ͻ�����
	const char* sendbuf = sendBuf.c_str();
	if (::send(client, sendbuf, strlen(sendbuf), 0) < 0)
	{
		return false;
	}
	return true;
}

int MultiAxisControl::receiveCommand(unsigned char* buf) {
	int count = 0;
	do {
		count = ::recv(client, (char*)buf, bufferLen, 0);
	} while (count < 0 && (errno == EAGAIN || errno == EINTR));
	return count;
}

void MultiAxisControl::getReceive() {
	while (bThreadRecvRun) {		
		int bytes = receiveCommand(buffer + front);
		//���������ӶϿ�
		if (bytes <= 0) {
			std::cout << "���ӶϿ�*" << std::endl;
			bConnect = false;
			disconnect();
			return;
		}
		//�̼߳�����λ����Ӧ��Ӧ��ɹ�����Ϊ����ָ��ɹ�
		front += bytes;
		if (front > 1024 * 1000) {
			memset(buffer, 0, sizeof(unsigned char) * 1024 * 1024);
			front = 0;
			continue;
		}
		std::string str = parseCommand(buffer);
		if(cFunc){
//			std::cout << "**" << std::endl;
			cFunc(str, pPointer);
		}
	}
}

bool MultiAxisControl::WaitReplay(DEVICE_STATE state, int timeout)
{
	//����ָ���ȴ�Ӧ��ʱ��
	clock_t startTime = clock();
	while (1)
	{
		//��ȡӦ��ʱ��
		clock_t stopTime = clock();
		//��Ӧ��״̬Ϊָ��Ӧ�����״̬Ӧ��ʱ
		if (nowState == state)
		{
			//��ʼ��Ӧ��
			nowState = INIT_STATE;
			return true;
		}
		if (stopTime - startTime > timeout)
			return false;
		Sleep(50);
	}
}

std::string MultiAxisControl::excuteCheck(std::string identifier) {
	int check = 0;
	for (int i = 0; identifier[i] != '\0'; i++) {
		check ^= identifier[i];
	}
	return intToHex(check, 2);
//	return std::to_string(check);
}

bool MultiAxisControl::sendSCMDE(const char* on)
{
	identifier = "SCMDE";
	identifier += on;
	check = excuteCheck('S' + identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	//���÷��ͽӿ�
	if (sendCommand(identifier))
		//�ȴ�ָ���Ӧ��Ӧ��
		return WaitReplay(COMMAND_ANSWER_STATE,1000);
	else
		return false;
}

bool MultiAxisControl::sendSWRKP(const char* on)
{
	identifier = "SWRKP";
	identifier += on;
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE,1000);
	else
		return false;
}

bool MultiAxisControl::sendSSCZP(char Axis, float sysZeroPara)
{
	identifier = "SSCZP";
	if (Axis == 'X') {
		if (sysZeroPara < -100 || sysZeroPara>100) {
			return false;
		}
	}
	if (Axis == 'Y') {
		if (sysZeroPara < -325 || sysZeroPara>325) {
			return false;
		}
	}
	if (Axis == 'Z') {
		if (sysZeroPara < -150 || sysZeroPara>150) {
			return false;
		}
	}
	if (Axis == 'R') {
		if (sysZeroPara < 0 || sysZeroPara>360) {
			return false;
		}
	}
	if (Axis == 'P') {
		if (sysZeroPara < -30 || sysZeroPara>30) {
			return false;
		}
	}
	if (Axis == 'H') {
		if (sysZeroPara < -180 || sysZeroPara>180) {
			return false;
		}
	}
	std::string str = std::to_string(int(sysZeroPara) * 100);
	while (str.length() < 6) {
		if (str[0] == '-') {
			str.insert(str.begin() + 1, '0');
		}
		else {
			str.insert(str.begin(), '0');
		}
	}
	identifier = identifier + Axis + ',' + str;
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendBTOSZ(char Axis)
{
	identifier = "BTOSZ";
	identifier += Axis;
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendSABMD(char Axis, float sysRunPara)
{
	identifier = "SABMD";
	if (Axis == 'X') {
		if (sysRunPara < -100 || sysRunPara>100) {
			return false;
		}
	}
	else if (Axis == 'Y') {
		if (sysRunPara < -325 || sysRunPara>325) {
			return false;
		}
	}
	else if (Axis == 'Z') {
		if (sysRunPara < -150 || sysRunPara>150) {
			return false;
		}
	}
	else if (Axis == 'R') {
		if (sysRunPara < 0 || sysRunPara>360) {
			return false;
		}
	}
	else if (Axis == 'P') {
		if (sysRunPara < -30 || sysRunPara>30) {
			return false;
		}
	}
	else if (Axis == 'H') {
		if (sysRunPara < -180 || sysRunPara>180) {
			return false;
		}
	}
	std::string str = std::to_string(int(sysRunPara) * 100);
	while (str.length() < 6) {
		if (str[0] == '-') {
			str.insert(str.begin() + 1, '0');
		}
		else {
			str.insert(str.begin(), '0');
		}
	}
	identifier = identifier + Axis + "," + str;
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendJOGCC(char Axis, char state)
{
	identifier = "JOGCC";
	identifier = identifier + Axis + "," + state;
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendSAASK()
{
	identifier = "SAASK";
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier)) {
		return WaitReplay(ATTITUDE_RESPONSE_STATE, 1000);
	}
	else
		return false;
}

bool MultiAxisControl::sendCSTEP(int steps, StepSequence* stepInfo)
{
	unsigned char* command = new unsigned char[500];
	uint8_t* stepCommand = new uint8_t[500];
	int index = stepSequenceTranse(stepCommand,steps, stepInfo);
	std::string stepStr = std::to_string(steps);
	while (stepStr.length() < 4) {
		stepStr = '0' + stepStr;
	}
	command[0] = '$';
	::memcpy(command + 1, "SCSTEP", 6);
	::memcpy(command + 7, &stepStr, 4);
	command[11] = ',';
	::memcpy(command + 12, stepCommand, index);
	//����У��λ
	std::string stepCheck = excuteStepCheck(command,index+11);
	command[12 + index] = '*';
	::memcpy(command + 13+index, &stepCheck, 2);
	command[15 + index] = '\r';
	command[16 + index] = '\n';
	if (::send(client, (char*)command, index, 0))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendSCSTA()
{
	identifier = "SCSTA";
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendSCSTP()
{
	identifier = "SCSTP";
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendSCHLD()
{
	identifier = "SCHLD";
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendSZSPD(char Axis, float toZeroSpeed)
{
	identifier = "SZSPD";
	if (Axis == 'X') {
		if (toZeroSpeed < 1 || toZeroSpeed>300) {
			return false;
		}
	}
	if (Axis == 'Y') {
		if (toZeroSpeed < 1 || toZeroSpeed>300) {
			return false;
		}
	}
	if (Axis == 'Z') {
		if (toZeroSpeed < 1 || toZeroSpeed>300) {
			return false;
		}
	}
	if (Axis == 'R') {
		if (toZeroSpeed < 0.1 || toZeroSpeed>60) {
			return false;
		}
	}
	if (Axis == 'P') {
		if (toZeroSpeed < 0.1 || toZeroSpeed>60) {
			return false;
		}
	}
	if (Axis == 'H') {
		if (toZeroSpeed < 0.1 || toZeroSpeed>60) {
			return false;
		}
	}
	std::string str = std::to_string(int(toZeroSpeed) * 10);
	while (str.length() < 4) {
		if (str[0] == '-') {
			str.insert(str.begin() + 1, '0');
		}
		else {
			str.insert(str.begin(), '0');
		}
	}
	identifier = identifier + std::to_string(Axis)+ "," + str;
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendSRSPD(char Axis, float runSpeed)
{
	identifier = "SRSPD";
	if (Axis == 'X') {
		if (runSpeed < 1 || runSpeed>300) {
			return false;
		}
	}
	if (Axis == 'Y') {
		if (runSpeed < 1 || runSpeed>300) {
			return false;
		}
	}
	if (Axis == 'Z') {
		if (runSpeed < 1 || runSpeed>300) {
			return false;
		}
	}
	if (Axis == 'R') {
		if (runSpeed < 0.1 || runSpeed>60) {
			return false;
		}
	}
	if (Axis == 'P') {
		if (runSpeed < 0.1 || runSpeed>60) {
			return false;
		}
	}
	if (Axis == 'H') {
		if (runSpeed < 0.1 || runSpeed>60) {
			return false;
		}
	}
	std::string str = std::to_string(int(runSpeed) * 10);
	while (str.length() < 4) {
		if (str[0] == '-') {
			str.insert(str.begin() + 1, '0');
		}
		else {
			str.insert(str.begin(), '0');
		}
	}
	identifier = identifier + Axis + "," + str;
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendCOFCR()
{
	identifier = "COFCR";
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(PARAMETER_RESPONSE_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendCOFCS(int parameter)
{
	identifier = "COFCS";
	std::string str = std::to_string(parameter);
	str.insert(str.begin(), '0');
	identifier += str;
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	return sendCommand(identifier);
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendRSTFS()
{
	identifier = "RSTFS";
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	return sendCommand(identifier);
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendSSLAG(char Axis, int lowLimitV, int highLimitV)
{
	identifier = "SSLAG";
	if (Axis == 'X') {
		if (lowLimitV < -100 || highLimitV > 100 || lowLimitV > highLimitV)
			return false;
	}
	if (Axis == 'Y') {
		if (lowLimitV < -325 || highLimitV > 325 || lowLimitV > highLimitV)
			return false;
	}
	if (Axis == 'Z') {
		if (lowLimitV < -150 || highLimitV > 150 || lowLimitV > highLimitV)
			return false;
	}
	if (Axis == 'P') {
		if (lowLimitV < -30 || highLimitV > 30 || lowLimitV > highLimitV)
			return false;
	}
	if (Axis == 'H') {
		if (lowLimitV < -180 || highLimitV > 180 || lowLimitV > highLimitV)
			return false;
	}
	std::string strLow = std::to_string(int(lowLimitV));
	if (lowLimitV > 0) {
		strLow = '+' + strLow;
	}
	while (strLow.length() < 4) {
		if (strLow[0] == '-') {
			strLow.insert(strLow.begin() + 1, '0');
		}
		else {
			strLow.insert(strLow.begin(), '0');
		}
	}
	std::string strhig = std::to_string(int(highLimitV));
	while (strhig.length() < 4) {
		if (strhig[0] == '-') {
			strhig.insert(strhig.begin() + 1, '0');
		}
		else {
			strhig.insert(strhig.begin(), '0');
		}
	}
	identifier = identifier + Axis + "," + strLow + "," + strhig;
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendTCPCT(int parameter)
{
	identifier = "TCPCT";
	std::string str = std::to_string(parameter);
	while (str.length() < 3) {
		str.insert(str.begin(), '0');
	}
	identifier += str;
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendMOTOR(char Axis, char state)
{
	identifier = "MOTOR";
	identifier = identifier + Axis + "," + state;
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendFEMOT(char Axis, char state)
{
	identifier = "FEMOT";
	identifier = identifier + Axis + "," + state;
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendRSMCU()
{
	identifier = "RSMCU";
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendSDIPA(char* ipAddress)
{
	identifier = "SDIPA";
	identifier += IPToHex(ipAddress);
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendSDPRT(int port)
{
	identifier = "SDPRT";
	std::string strPort = intToHex(port);
	identifier += strPort;
	check = excuteCheck(identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

bool MultiAxisControl::sendSDMAC(char* macAddress)
{
	identifier = "SDMAC";
	identifier += convertMacAddress(macAddress);
	check = excuteCheck('S' + identifier);
	identifier = orderHead + identifier + "*" + check + "\r\n";
	if (sendCommand(identifier))
		return WaitReplay(COMMAND_ANSWER_STATE, 1000);
	else
		return false;
}

float* MultiAxisControl::getMotionVelocity(unsigned char* buf,int head) {
	//�˶��ٶ�
	float runSpeedX = ((buf[head + 7] << 8) | buf[head + 8]) * 0.1;
	float runSpeedY = ((buf[head + 10] << 8) | buf[head + 11]) * 0.1;
	float runSpeedZ = ((buf[head + 13] << 8) | buf[head + 14]) * 0.1;
	float runSpeedP = ((buf[head + 16] << 8) | buf[head + 17]) * 0.1;
	float runSpeedR = ((buf[head + 19] << 8) | buf[head + 20]) * 0.1;
	float runSpeedH = ((buf[head + 22] << 8) | buf[head + 23]) * 0.1;
	float data[6] = { runSpeedX ,runSpeedY ,runSpeedZ ,runSpeedP ,runSpeedR ,runSpeedH };
	return data;
}

float* MultiAxisControl::getZeroVelocity(unsigned char* buf, int head) {
	//�����ٶ�
	float zeroSpeedX = ((buf[head + 25] << 8) | buf[head + 26]) * 0.1;
	float zeroSpeedY = ((buf[head + 28] << 8) | buf[head + 29]) * 0.1;
	float zeroSpeedZ = ((buf[head + 31] << 8) | buf[head + 32]) * 0.1;
	float zeroSpeedP = ((buf[head + 34] << 8) | buf[head + 35]) * 0.1;
	float zeroSpeedR = ((buf[head + 37] << 8) | buf[head + 38]) * 0.1;
	float zeroSpeedH = ((buf[head + 40] << 8) | buf[head + 41]) * 0.1;
	float data[6] = { zeroSpeedX ,zeroSpeedY ,zeroSpeedZ ,zeroSpeedP ,zeroSpeedR ,zeroSpeedH };
	return data;
}

float* MultiAxisControl::getSystemZeroSite(unsigned char* buf, int head) {
	//ϵͳ��λ
	float zeroSiteX = ((buf[head + 43] << 8) | buf[head + 44]) * 0.01;
	if (zeroSiteX > 327.67) zeroSiteX -= 655.36;
	float zeroSiteY = ((buf[head + 46] << 8) | buf[head + 47]) * 0.01;
	if (zeroSiteY > 327.67) zeroSiteY -= 655.36;
	float zeroSiteZ = ((buf[head + 49] << 8) | buf[head + 50]) * 0.01;
	if (zeroSiteZ > 327.67) zeroSiteZ -= 655.36;
	float zeroSiteP = ((buf[head + 52] << 8) | buf[head + 53]) * 0.01;
	if (zeroSiteP > 327.67) zeroSiteP -= 655.36;
	float zeroSiteR = ((buf[head + 55] << 8) | buf[head + 56]) * 0.01;
	if (zeroSiteR > 327.67) zeroSiteR -= 655.36;
	float zeroSiteH = ((buf[head + 58] << 8) | buf[head + 59]) * 0.01;
	if (zeroSiteH > 327.67) zeroSiteH -= 655.36;
	float data[6] = { zeroSiteX ,zeroSiteY ,zeroSiteZ ,zeroSiteP ,zeroSiteR ,zeroSiteH };
	return data;
}

int* MultiAxisControl::getLimit(unsigned char* buf, int head) {
	//��������λ
	int higLimitX = (buf[head + 65] << 8) | buf[head + 66];
	if (higLimitX > 32767) higLimitX -= 65536;
	int lowLimitX = (buf[head + 68] << 8) | buf[head + 69];
	if (lowLimitX > 32767) lowLimitX -= 65536;

	int higLimitY = (buf[head + 71] << 8) | buf[head + 72];
	if (higLimitY > 32767) higLimitY -= 65536;
	int lowLimitY = (buf[head + 74] << 8) | buf[head + 75];
	if (lowLimitY > 32767) lowLimitY -= 65536;

	int higLimitZ = (buf[head + 77] << 8) | buf[head + 78];
	if (higLimitZ > 32767) higLimitZ -= 65536;
	int lowLimitZ = (buf[head + 80] << 8) | buf[head + 81];
	if (lowLimitZ > 32767) lowLimitZ -= 65536;

	int higLimitP = (buf[head + 83] << 8) | buf[head + 84];
	if (higLimitP > 32767) higLimitP -= 65536;
	int lowLimitP = (buf[head + 86] << 8) | buf[head + 87];
	if (lowLimitP > 32767) lowLimitP -= 65536;

	int higLimitR = (buf[head + 89] << 8) | buf[head + 90];
	if (higLimitR > 32767) higLimitR -= 65536;
	int lowLimitR = (buf[head + 92] << 8) | buf[head + 93];
	if (lowLimitR > 32767) lowLimitR -= 65536;

	int higLimitH = (buf[head + 95] << 8) | buf[head + 96];
	if (higLimitH > 32767) higLimitH -= 65536;
	int lowLimitH = (buf[head + 98] << 8) | buf[head + 99];
	if (lowLimitH > 32767) lowLimitH -= 65536;
	int data[12] = { higLimitX ,lowLimitX ,higLimitY ,lowLimitY , higLimitZ ,lowLimitZ ,
		higLimitP ,lowLimitP , higLimitR ,lowLimitR , higLimitH ,lowLimitH };
	return data;
}

int MultiAxisControl::getTCPReTransNum(unsigned char* buf, int head) {
	//TCP�ش�����
	int TCPReTransNum = buf[head + 101];
	return TCPReTransNum;
}

std::string MultiAxisControl::getMACAddress(unsigned char* buf, int head) {
	//MAC��ַ
	std::ostringstream result;
	for (int i = head + 103; i < 109; i++) {
		result << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buf[i]);
		if (i != 5) {
			result << ".";
		}
	}
	return result.str();
}

//�Կ��ƻ�Ӧ����н���
std::string MultiAxisControl::parseCommand(unsigned char* buf) {
	std::string retMsg = "";
	while (1) {
		int head = findHead(buf);
		//buffer[head] == '$'
		if (head != -1) {
			int tail = findTail(buf);
			if (tail != -1) {
				//CMDAN
				if (buf[head + 2] == 'C' && buf[head + 3] == 'M' && buf[head + 4] == 'D'
					&& buf[head + 5] == 'A' && buf[head + 6] == 'N') {
					state = 0;
					//����Ӧ��״̬
					nowState = COMMAND_ANSWER_STATE;
					if (buf[head + 7] == '0')
					{
						//retMsg = "����Ӧ��ָ�ָ������";	
						retMsg = "CMDAN0";
					}
					else if (buf[head + 7] == '1') {
						//retMsg = "����Ӧ��ָ�ָ���ʽ����";	
						retMsg = "CMDAN1";
					}
					else if (buf[head + 7] == '2') {
						//retMsg = "����Ӧ��ָ�У��ʹ���";			
						retMsg = "CMDAN2";
					}
					else if (buf[head + 7] == '3')
					{
						//retMsg = "����Ӧ��ָ�����������δ����";	
						retMsg = "CMDAN3";
					}
					else if (buf[head + 7] == '4')
					{
						//retMsg = "����Ӧ��ָ�δ����ָ�����ģʽ";	
						retMsg = "CMDAN4";
					}
					else if (buf[head + 7] == '5'){
						//retMsg = "����Ӧ��ָ�����δ�л���ָ��ģʽ";
						retMsg = "CMDAN5";
					}
					else if (buf[head + 7] == '6') {
						//retMsg = "����Ӧ��ָ�δ�л����ֶ�ģʽ";
						retMsg = "CMDAN6";
					}
					else if (buf[head + 7] == '7') {
						//retMsg = "����Ӧ��ָ���ʾTCP�ش���������";
						retMsg = "CMDAN7";
					}
				}
				//SYSAD
				else if (buf[head + 2] == 'S' && buf[head + 3] == 'Y' && buf[head + 4] == 'S'
					&& buf[head + 5] == 'A' && buf[head + 6] == 'D') {
					int combinedValue;
					state = 0;
					//����Ӧ��״̬
					nowState = ATTITUDE_RESPONSE_STATE;
/*					char str[7] = {0};
					::memcpy(str, buf + head + 7, 4);
					float siteX = std::atoi(str)*0.01;*/
					combinedValue = (buf[ head + 7] << 24) | (buf[head + 8] << 16) | (buf[head + 9] << 8) | buf[head + 10];
					if (combinedValue > 32767) combinedValue -= 65536;
					float siteX = combinedValue * 0.01;
					combinedValue = (buf[head + 12] << 24) | (buf[head + 13] << 16) | (buf[head + 14] << 8) | buf[head + 15];
					if (combinedValue > 32767) combinedValue -= 65536;
					float siteY = combinedValue * 0.01;
					combinedValue = (buf[head + 17] << 24) | (buf[head + 18] << 16) | (buf[head + 19] << 8) | buf[head + 20];
					if (combinedValue > 32767) combinedValue -= 65536;
					float siteZ = combinedValue * 0.01;
					combinedValue = (buf[head + 22] << 24) | (buf[head + 23] << 16) | (buf[head + 24] << 8) | buf[head + 25];
					if (combinedValue > 32767) combinedValue -= 65536;
					float siteP = combinedValue * 0.01;
					combinedValue = (buf[head + 27] << 24) | (buf[head + 28] << 16) | (buf[head + 29] << 8) | buf[head + 30];
					if (combinedValue > 32767) combinedValue -= 65536;
					float siteR = combinedValue * 0.01;
					combinedValue = (buf[head + 32] << 24) | (buf[head + 33] << 16) | (buf[head + 34] << 8) | buf[head + 35];
					if (combinedValue > 32767) combinedValue -= 65536;
					float siteH = combinedValue * 0.01;
					retMsg = "ϵͳ��̬Ӧ��ָ���ǰx��λ��" + std::to_string(siteX) + "mm������ǰy��λ��" + std::to_string(siteY) + "mm������ǰz��λ��" + std::to_string(siteZ) + "mm������ǰp��λ��" + std::to_string(siteP) + "�㴦����ǰr��λ��" + std::to_string(siteR) + "�㴦����ǰh��λ��" + std::to_string(siteH) + "�㴦";
				}
				//STEPA
				else if (buf[head + 2] == 'S' && buf[head + 3] == 'T' && buf[head + 4] == 'E'
					&& buf[head + 5] == 'P' && buf[head + 6] == 'A') {
					state = 0;
					int steps = (buf[head + 7] << 8) | buf[head + 8];
					retMsg = "������������Ӧ��ָ���" + std::to_string(steps) + "���˶����";
				}
				//SROVR
				else if (buf[head + 2] == 'S' && buf[head + 3] == 'R' && buf[head + 4] == 'O'
					&& buf[head + 5] == 'V' && buf[head + 6] == 'R') {
					state = 0;
					retMsg = "�������������";
				}
				//UPLCP
				else if (buf[head + 2] == 'U' && buf[head + 3] == 'P' && buf[head + 4] == 'L'
					&& buf[head + 5] == 'C' && buf[head + 6] == 'P') {
					state = 1;
					//����Ӧ��״̬
					nowState = PARAMETER_RESPONSE_STATE;
					std::string controlModeMsg = "";
					std::string workModeMsg = "";
					//�˶��ٶ�
					float* motionVelocityData = getMotionVelocity(buf, head);
					//�����ٶ�
					float* zeroVelocityData = getZeroVelocity(buf, head);
					//ϵͳ��λ
					float* systemZeroSiteData = getSystemZeroSite(buf, head);
					//���Ʒ�ʽ
					char controlMode = buf[head + 61];
					if (controlMode == '1')
						controlModeMsg = "�ֶ�ģʽ";
					else if (controlMode == '2')
						controlModeMsg = "ָ�����ģʽ";
					//������ʽ
					char workMode = buf[head + 63];
					if (workMode == '1')
						workModeMsg = "�궨ģʽ";
					else if (workMode == '2')
						workModeMsg = "����ģʽ";
					//��������λ
					int* limitData = getLimit(buf, head);
					//TCP�ش�����
					int TCPReTransNum = getTCPReTransNum(buf,head);
					//MAC��ַ
					std::string MACData = getMACAddress(buf, head);
					int n = 0;
					for (int i = 0; i < 6; i++) {
						abcMsg[n] = std::to_string(motionVelocityData[i]);
						n++;
					}
					for (int i = 0; i < 6; i++) {
						abcMsg[n] = std::to_string(zeroVelocityData[i]);
						n++;
					}for (int i = 0; i < 6; i++) {
						abcMsg[n] = std::to_string(systemZeroSiteData[i]);
						n++;
					}
					for (int i = 0; i < 12; i++) {
						abcMsg[n] = std::to_string(limitData[i]);
						n++;
					}abcMsg[n + 1] = MACData;
					abcMsg[n + 2] = std::to_string(TCPReTransNum);
					
//				std::string* retMsg = new std::string[]{ motionVelocityData, motionVelocityData[1],motionVelocityData[2],motionVelocityData[3],motionVelocityData[4],motionVelocityData[5],zeroVelocityData[0],zeroVelocityData[1] ,zeroVelocityData[2] ,zeroVelocityData[3] ,zeroVelocityData[4] ,zeroVelocityData[5],systemZeroSiteData[0] ,systemZeroSiteData[1] ,systemZeroSiteData[2] ,systemZeroSiteData[3] ,systemZeroSiteData[4] ,systemZeroSiteData[5],controlModeMsg,workModeMsg,	limitData[0],limitData[1],limitData[2],limitData[3],limitData[4],limitData[5],limitData[6],limitData[7],limitData[8],limitData[9],limitData[10],limitData[11],TCPReTransNum,MACData};

/*					retMsg = "ͨ�ò�������ָ�x�������ٶ�" + std::to_string(motionVelocityData[0]) + "mm/s��y�������ٶ�" + std::to_string(motionVelocityData[1]) + "mm/s��z�������ٶ�" + std::to_string(motionVelocityData[2]) + "mm/s��p�������ٶ�" + std::to_string(motionVelocityData[3]) + "��/s��r�������ٶ�" + std::to_string(motionVelocityData[4]) + "��/s��h�������ٶ�" + std::to_string(motionVelocityData[5]) + "��/s��"
						+ "\n"
						+ "x������ٶ�" + std::to_string(zeroVelocityData[0]) + "mm/s��y������ٶ�" + std::to_string(zeroVelocityData[1]) + "mm/s��z������ٶ�" + std::to_string(zeroVelocityData[2]) + "mm/s��p������ٶ�" + std::to_string(zeroVelocityData[3]) + "��/s��r������ٶ�" + std::to_string(zeroVelocityData[4]) + "��/s��h������ٶ�" + std::to_string(zeroVelocityData[5]) + "��/s��"
						+ "\n"
						+ "x��ϵͳ��λ" + std::to_string(systemZeroSiteData[0]) + "mm,y��ϵͳ��λ" + std::to_string(systemZeroSiteData[1]) + "mm,z��ϵͳ��λ" + std::to_string(systemZeroSiteData[2]) + "mm,p��ϵͳ��λ" + std::to_string(systemZeroSiteData[3]) + "��,r��ϵͳ��λ" + std::to_string(systemZeroSiteData[4]) + "��,h��ϵͳ��λ" + std::to_string(systemZeroSiteData[5]) + "�㣬"
						+ "\n"
						+ "���Ʒ�ʽ:" + controlModeMsg + "������ģʽ:" + workModeMsg + "��"
						+ "\n"
						+ "x��������λ:" + std::to_string(limitData[0]) + "mm��x��������λ:" + std::to_string(limitData[1]) + "mm��y��������λ:" + std::to_string(limitData[2]) + "mm��y��������λ:" + std::to_string(limitData[3]) + "mm��z��������λ:" + std::to_string(limitData[4]) + "mm��z��������λ:" + std::to_string(limitData[5]) + "mm��p��������λ:" + std::to_string(limitData[6]) + "�㣬p��������λ:" + std::to_string(limitData[7]) + "�㣬r��������λ:" + std::to_string(limitData[8]) + "�㣬r��������λ:" + std::to_string(limitData[9]) + "�㣬h��������λ:" + std::to_string(limitData[10]) + "�㣬h��������λ:" + std::to_string(limitData[11]) + "�㣬"
						+ "\n"
						+ "TCP�ش�����:" + std::to_string(TCPReTransNum) + "MAC��ַ:" + MACData;	*/
				}
				//HWERR
				else if (buf[head + 2] == 'H' && buf[head + 3] == 'W' && buf[head + 4] == 'E'
					&& buf[head + 5] == 'R' && buf[head + 6] == 'R') {
					state = 0;
					char errParameter = buf[head + 8];
					if (errParameter == '1') {
						//retMsg = "X�Ḵλ�쳣";
						retMsg = "HWERR1";
					}else if (errParameter == '2') {
						//retMsg = "Y�Ḵλ�쳣";
						retMsg = "HWERR2";
					}
					else if (errParameter == '3') {
						//retMsg = "Z�Ḵλ�쳣";
						retMsg = "HWERR3";
					}
					else if (errParameter == '4') {
						//retMsg = "P�Ḵλ�쳣";
						retMsg = "HWERR4";
					}
					else if (errParameter == '5') {
						//retMsg = "R�Ḵλ�쳣";
						retMsg = "HWERR5";
					}
					else if (errParameter == '6') {
						//retMsg = "H�Ḵλ�쳣";
						retMsg = "HWERR6";
					}
				}
				else {
					retMsg = "��λ��Ӧ�����";				
				}
				front -= tail+1;
				::memcpy(buf, buf + tail+1, front);
				return retMsg;
			}
			else {
				return retMsg;
			}
		}
		else {
			return retMsg;
		}
		return retMsg;
	}
}

void MultiAxisControl::excuteCheck(const char* identifier,char commandCheck[]) {
	int check = 0;
	for (int i = 0; identifier[i] != '\0'; i++) {
		check = check ^ identifier[i];
	}
	std::stringstream ss;
	ss << std::hex << std::setw(2) << std::uppercase << std::setfill('0') << check;
	std::string Sstr = ss.str();
	const char* cstr = Sstr.c_str();
	::memcpy(commandCheck, cstr, strlen(cstr));
}

std::string MultiAxisControl::IPToHex(const char* ipAddress)
{
	std::stringstream ss(ipAddress);
	std::string ipPart;
	std::vector<int> ipParts;
	while (std::getline(ss, ipPart, '.')) {
		ipParts.push_back(std::stoi(ipPart));
	}
	std::stringstream hexSS;
	hexSS << std::uppercase << std::hex << std::setfill('0');
	for (const auto&part : ipParts) {
		hexSS << std::setw(2) << part;
	}
	return hexSS.str();
}

std::string MultiAxisControl::intToHex(int c) {
	std::stringstream ss;
	ss << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << c;
	return ss.str();
}

std::string MultiAxisControl::convertMacAddress(const std::string& mac) {
	std::string convertedMac;
	for (char c : mac) {
		if (c != '.')
			convertedMac += c;
	}
	return convertedMac;
}
int MultiAxisControl::findHead(unsigned char* buffer) {
	for (int i = 0; i < front; i++) {
		if (buffer[i] == '$') {
			return i;
		}
	}
	return -1;
}

int MultiAxisControl::findTail(unsigned char* buffer) {
	int head = findHead(buffer);
	for (int i = head+1; i < front; i++) {
		//ָ����շ��ִ���������������ָ��ͷ
		if (buffer[i] == '$') {
			return -1;
		}
		else if (buffer[i] == '\n') {
			return i;
		}
	}
	return -1;
}

char intToHexChar(int value) {
	std::stringstream ss;
	ss << std::hex << std::setw(2) << std::setfill('0') << value;
	int hexint;
	ss >> hexint;
	return static_cast<char>(hexint);
}

int MultiAxisControl::stepSequenceTranse(uint8_t* stepCommand,int step, StepSequence* stepInfo) {
	uint8_t* stepSequence = new uint8_t[500];
	int i, index;
	//����λ��
	index = 0;
	//����������
	for (int j = 0; j < step; j++) {
		for (i = 0; i < 5; i++) {
			//������λ
			if (i % 5 == 0) {
				stepSequence[index] = stepInfo[j].c_motor;
			}
			//������ʱλ
			else if ((i - 1) % 5 == 0) {
				int delay = int(stepInfo[j].m_delay_ms);
				uint8_t arr[4] = { 0 };
				arr[0] = delay;
				stepSequence[index] = arr[0];
				//snprintf(arr,sizeof(arr), "%02x", stepInfo[0].m_delay_ms);
				//			::memcpy(stepSequence + i, arr, 1);
			}
			//����ִ��λ
			else if ((i - 2) % 5 == 0) {
				stepSequence[index] = stepInfo[j].c_execute;
			}
			//����Ԥ��λ
			else if ((i - 3) % 5 == 0) {
				int reservedNum = int(stepInfo[j].reserved);
				uint8_t arr[4] = { 0 };
				arr[0] = reservedNum;
				stepSequence[index] = arr[0];
				//snprintf(arr,sizeof(arr), "%02x", stepInfo[0].m_delay_ms);
				//			::memcpy(stepSequence + i, arr, 1);
			}
			//�����ƶ�����λ
			else {
				uint8_t buffer[4] = { 0 };
				int distance = stepInfo[j].m_dest_position;
				buffer[0] = distance >> 24;
				buffer[1] = distance >> 16;
				buffer[2] = distance >> 8;
				buffer[3] = distance;
				stepSequence[index++] = buffer[3];
				stepSequence[index++] = buffer[2];
				stepSequence[index++] = buffer[1];
				stepSequence[index] = buffer[0];
			}
			index++;
		}
	}
	::memcpy(stepCommand, stepSequence, index);
	return index+1;
}

std::string MultiAxisControl::excuteStepCheck(uint8_t * command,int totalNum)
{
	int check = 0;
	for (int i = 1; i <= totalNum;i++) {
		check ^= command[i];
	}
	return std::to_string(check);
}

std::string MultiAxisControl::intToHex(int num, int digit) {
	std::stringstream ss;
	ss << std::hex << std::uppercase << std::setw(digit) << std::setfill('0') << num;
	return ss.str();
}

void MultiAxisControl::intToHex(char* Distination, int x, int digit) {
	std::stringstream ss;
	ss << std::hex << std::uppercase << std::setw(digit) << std::setfill('0') << x;
	std::string Sstr = ss.str();
	const char* cstr = Sstr.c_str();
	::memcpy(Distination, cstr, strlen(cstr));
}