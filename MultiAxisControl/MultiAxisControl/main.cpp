#include <bitset>
#include"MultiAxisControl.h"
#include"B.h"

/*std::string intToHex(char c) {
	int number = static_cast<int>(c);
	std::stringstream ss;
	ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (number & 0xFF);
	return ss.str();
}*/

void intToHex(char* Distination,int x,int digit) {
	std::stringstream ss;
	ss << std::hex << std::uppercase << std::setw(digit) << std::setfill('0') << x;
	std::string Sstr = ss.str();
	const char* cstr = Sstr.c_str();
	::memcpy(Distination, cstr, strlen(cstr));
}

void longToLittleEndianHex(long value, char* buffer)
{
	buffer[0] = static_cast<char>((value >> 0) & 0xFF);
	buffer[1] = static_cast<char>((value >> 8) & 0xFF);
	buffer[2] = static_cast<char>((value >> 16) & 0xFF);
	buffer[3] = static_cast<char>((value >> 24) & 0xFF);
}

char intToHexCharfunc(int value)
{
	char hexChar[3]; // 缓冲区大小为3，用于存储两位十六进制数和结尾的空字符
	snprintf(hexChar, sizeof(hexChar), "%02X", value);
	return hexChar[0]; // 返回十六进制数的第一个字符
}

using namespace std;

void main() {
	StepSequence stepInfo[10];
	stepInfo[0].c_motor = 'X';
	stepInfo[0].m_delay_ms = 1;
	stepInfo[0].c_execute = 'N';
	stepInfo[0].reserved = 0;
	stepInfo[0].m_dest_position = 5000;

	MultiAxisControl mac;
	mac.sendCSTEP(1, stepInfo);

	uint8_t sum[10] = { 0 };
	sum[0] = 0x88;
	std::cout << (char)sum[0];

/*	int numb = 5000;
	uint8_t Numb[4] = { 0 };
	Numb[0] = numb >> 16;
	Numb[1] = numb >> 16;
	Numb[2] = numb >> 8;
	Numb[3] = numb;
	printf("0x%02x 0x%02x 0x%02x 0x%02x", Numb[0], Numb[1],Numb[2],Numb[3]);*/
/*	char a[10] = { 0 };
	a[0] = 0x01;
	a[1] = 0x65;
	a[2] = 0x78;
	a[3] = 250;
	std::cout << int(a[2]);*/
/*	StepSequence databa[100];
	std::fstream igs;
	string text02, text01, text03;
	std::string textname = "C:\\Users\\Ditto\\Documents\\WeChat Files\\wxid_5w03vsaldejk22\\FileStorage\\File\\2023-07\\a.txt";
	//步数
	int i = 0;
	//缓冲区
	int n = 0;
	int j = 0;
	char ar[100000];
	igs.open(textname, ios::in);

	if (!igs.is_open())
	{
		string a = "文件读取失败";
		return;
	}
	else {
		while (!igs.eof())
		{
			if (igs.getline(ar, 100000, '\n')) {
				char *result = strtok(ar, " ");
				databa[i].c_motor = result[0];
				result = strtok(NULL, " ");
				databa[i].m_delay_ms = result[0];
				result = strtok(NULL, " ");
				databa[i].c_execute = result[0];
				result = strtok(NULL, " ");
				databa[i].reserved = result[0];
				result = strtok(NULL, " ");
				databa[i].m_dest_position = atol(result);
				i++;
			}
		}
	}*/




	const char* orderHead = "$S";
	char c = orderHead[1];
	std::cout << c;
	std::cout << c;

	





//	system("pause");
}
