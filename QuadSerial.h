#pragma once
#include <string>
#include "Serial.h"
#include "timer.h"
#include "ThreadLock.h"

const int ASCII_MODE = 1;
const int BINARY_MODE = 2;
const int NO_ARDUINO_MODE = 1; 

							   
const double CHANNEL_MIN = 1000.0;  
const double CHANNEL_MAX = 2000;   
const double CHANNEL_RANGE = CHANNEL_MAX - CHANNEL_MIN;

const double CHANNEL_ARDUINO_MIN = 0;
const double CHANNEL_ARDUINO_MAX = 128;
const double CHANNEL_ARDUINO_RANGE = CHANNEL_ARDUINO_MAX - CHANNEL_ARDUINO_MIN;

const double RANGES_RATIO = CHANNEL_ARDUINO_RANGE / CHANNEL_RANGE;


const int START_BIT = 255;


class QuadSerial
{
public:
	QuadSerial(int port);
	~QuadSerial();
	void writeSyma(float toSend[4], bool picture, bool video);
	void writeSyma(double pitch, double roll, double throttle, double yaw, bool picture, bool video);
	void read();
	void enableVideo(bool val) { video = val; }
	void takePicture(bool val) { picture = val; }
protected:
	double convertRange(double val);

private:
	void writeSymaTask(char* buffer, int len);
	int portNo;
	std::string terminator;
	int timeout;
	int inputBufferSize;
	std::string  readAsyncMode;
	int dataBits;
	int parity;
	int flowControl;
	int stopBits;
	int baudRate;
	int binary;
	SRL::Serial* serial;
	char *buffer;
	stoper::CTimer time;
	bool video;
	bool picture;
	char symaBuffer[8];
	ThreadLock lock;
	stoper::CTimer logTime;
};

