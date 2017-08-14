#include "QuadSerial.h"
#include <WinBase.h>
#include <iostream>
#include <sstream>


QuadSerial::QuadSerial(int port) :
portNo(8),
terminator(""),
timeout(10),
inputBufferSize(1000),
readAsyncMode("manual"),
dataBits(8),
parity(NOPARITY),
flowControl(RTS_CONTROL_ENABLE),
stopBits(0), //ONESTOPBIT is 0
binary(1),
baudRate(115200)  ,
serial(0),
buffer(0),
video(false),
picture(false)

{
	lock.Unlock();
	portNo = port;
	char word[128];
	try
	{
		std::stringstream sstm;
		if (portNo > 9)
		{
			sprintf(word,"%s%d\0", "\\\\.\\COM", port);
		}
		else
		{
			sprintf(word,"%s%d\0", "COM", port);
		}
		logTime.start();
		
		Sleep(200);
		printf("%s\n", sstm.str().c_str());	
		std::string com(word);
		int messageSize = 8;
		buffer = new char[messageSize];
		stoper::CTimer time;
		buffer[4] = ' ';
		dataBits = 8;
		stopBits = 0;
		serial = new SRL::Serial(com, parity, dataBits, flowControl, stopBits, inputBufferSize, binary, timeout, readAsyncMode.compare("manual"), baudRate);
	}
	catch (...)
	{
		printf("can not open COM PORT\n");
	}
}


QuadSerial::~QuadSerial()
{
	delete serial;
	serial = NULL;
}

double QuadSerial::convertRange(double val)
{
	double newVal = val - CHANNEL_MIN;
	newVal *= RANGES_RATIO;
	newVal += CHANNEL_ARDUINO_MIN;
	return newVal;
}

void QuadSerial::read()
{
	char local[1000];
	serial->read(local,  32);
}

void QuadSerial::writeSyma(float toSend[4], bool picture, bool video)
{

	writeSyma(toSend[0], toSend[1], toSend[2], toSend[3], picture, video);
	return;

}
void QuadSerial::writeSyma(double _pitch, double _roll, double _throttle, double _yaw, bool picture, bool video)
{

	stoper::CTimer _time;
	_time.start();
	unsigned char pitch = (255 * ((_pitch - CHANNEL_MIN) / (CHANNEL_MAX - CHANNEL_MIN))) + 0.5;
	if (pitch < 128)
		pitch = 128 - pitch;
	unsigned char roll = (255 * ((_roll - CHANNEL_MIN) / (CHANNEL_MAX - CHANNEL_MIN))) + 0.5;
	if (roll < 128)
		roll = 128 - roll;
	unsigned char throttle = 255 - 255 * ((_throttle - CHANNEL_MIN) / (CHANNEL_MAX - CHANNEL_MIN));
	unsigned char yaw = (255 * ((_yaw - CHANNEL_MIN) / (CHANNEL_MAX - CHANNEL_MIN))) + 0.5;
	if (yaw < 128)
		yaw = 128 - yaw;
	symaBuffer[0] = 255;
	symaBuffer[1] = roll;
	symaBuffer[2] = pitch;
	symaBuffer[3] = throttle;
	symaBuffer[4] = yaw;
	symaBuffer[5] = 0;	 
	symaBuffer[6] = picture ? 150 : 0;	
	symaBuffer[7] = video ? 150 : 0;	

	int len = 1024;
	_time.start();
	serial->write((char*)symaBuffer, 8);
	double t1 = _time.getTimeAndRelease();

}

