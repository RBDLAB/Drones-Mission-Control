#pragma once 
#include <string>
#include <stdio.h>    // Standard input/output definitions 
#include <stdlib.h> 
#include <string.h>   // String function definitions 
//#include <unistd.h>   // for usleep()
//#include <getopt.h>
#include <fcntl.h>    // File control definitions 
#include <errno.h>    // Error number definitions 
//#include <termios.h>  // POSIX terminal control definitions 
//#include <sys/ioctl.h>
#include <iostream>
//#include "arduino-serial-lib.h"
#include "SerialClass.h"
#include "Serial.h"

#define ASCII_MODE	1
#define BINARY_MODE	2
#define NO_ARDUINO_MODE	3

#define CHANNEL_MIN	1000
#define CHANNEL_MAX	2000

#define CHANNEL_ARDUINO_MIN	0
#define CHANNEL_ARDUINO_MAX	128

#define START_BIT	255

using namespace std;

const float CHANNEL_RANGE = CHANNEL_MAX - CHANNEL_MIN;
const float CHANNEL_ARDUINO_RANGE = CHANNEL_ARDUINO_MAX - CHANNEL_ARDUINO_MIN;
const float RANGES_RATIO = CHANNEL_ARDUINO_RANGE / CHANNEL_RANGE;

class QuadSerial{

public:

	char* portNo;
	int mode;

	char terminator;
	int timeout;
	int inputBufferSize;
	string readAsyncMode;
	int dataBits;
	string parity;
	string flowControl;
	int stopBits;
	int baudRate;

	int serialFD;
	//Serial serial;
	SRL::Serial* serial2;
	bool result;

	char *data;

	QuadSerial( ) : portNo("COM1") , mode(BINARY_MODE) , terminator(0) , timeout(10) , inputBufferSize(1000)
	, readAsyncMode("manual") , dataBits(8) , parity("none") , flowControl("none") , stopBits(1) , baudRate(115200) , serialFD(0), data(new char[30]) {} 
	//QuadSerial(char* port) :  mode(BINARY_MODE), terminator(0), timeout(10), inputBufferSize(1000)
	//	, readAsyncMode("manual"), dataBits(8), parity("none"), flowControl("none"), stopBits(1), baudRate(115200), serialFD(0), data(new char[30]), serial()
	//{
	//	result = false;
	//	portNo = port;
	//}
	//QuadSerial() : portNo("COM1"), mode(BINARY_MODE), terminator(0), timeout(10), inputBufferSize(1000)
	//	, readAsyncMode("manual"), dataBits(8), parity("none"), flowControl("none"), stopBits(1), baudRate(115200), serialFD(0), data(new char[30]), serial() {}

	~QuadSerial() { delete data; }
	unsigned char symaBuffer[8];
	unsigned char dummyBuffer[1024];

	void convertRange( float oldVal[4] ){

		for (int i = 0; i < 4; ++i){
			oldVal[i] = ((oldVal[i] - CHANNEL_MIN) * RANGES_RATIO ) + CHANNEL_ARDUINO_MIN;
		}
	}
	void setMode(int _mode)
	{
		mode = _mode;
	}
	void setPort(char* com)
	{
		portNo = com;
	}
	
	void open(){

		if ( serialFD != 0 ){
			close();
		}

		//serialFD = serialport_init( portNo , baudRate );
		//cout << "serialFD: " << serialFD << endl;
		
		//result = serial.Open(portNo);

		 int binary = 1;
		 int iparity = NOPARITY;
		 int flowc = RTS_CONTROL_DISABLE;
		 serial2 = new SRL::Serial(portNo, iparity, dataBits, flowc, stopBits, inputBufferSize, binary, timeout, readAsyncMode.compare("manual"), baudRate);
	}

	void close(){
//		serial.Close();
		delete serial2;
	}

	//void closeAll();

	void flush(){
		cout << "not available";
		//if ( serialFD != 0 )
		//	serialport_flush(serialFD);
	}
	void writeSyma(float toSend[4], bool picture, bool video);
	void write( float toSend[4] ){
		//if ( serialFD == 0 )
		//{
		//	cout << "exiting because serialFD == 0";
		//	return;
		//}
		int len;
		switch( mode ){

			case BINARY_MODE:
				//cout << "trying to print binary" << endl;
				len = sprintf(data, "%c%c%c%c%c", START_BIT, (char)toSend[0], (char)toSend[1], (char)toSend[2], (char)toSend[3]);
				break;
			case ASCII_MODE:
				cout << "trying to print ascii" << endl;
				len = sprintf( data , "input: %f,%f,%f,%f\n" , toSend[0] , toSend[1] , toSend[2] , toSend[3] );
				break;
			default:
				printf("Error writing data to Arduino");
				break;
		}

		if ( data != 0 )
		{
			//serial.WriteData(data, len);
			serial2->write(data, len);
		}
		else
			cout << "data = 0" << endl;
	}

	void writeLadyBird( float data[4] ){
		convertRange( data );
		write( data );
	}

	void test(){

		float data[4] = { 1000.0 , 1200.0 , 1500.0 , 2000.0 };
		open();
		writeLadyBird( data );
		close();
	}

};
