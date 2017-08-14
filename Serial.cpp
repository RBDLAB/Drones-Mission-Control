/** Serial.cpp
*
* A very simple serial port control class that does NOT require MFC/AFX.
*
* @author Hans de Ruiter
*
* @version 0.1 -- 28 October 2008
*/

#include <iostream>
using namespace std;

#include "Serial.h"
using namespace SRL;
//Serial::Serial(tstring &commPortName, int bitRate)
Serial::Serial(std::string commPortName, int parity, int dataBits, int flowControl, int stopBits, int inputBufferSize, int binary, int timeOut, int readAsync, int bitRate)
{
	std::string stemp = commPortName;// std::wstring(commPortName.begin(), commPortName.end());
									 //LPCWSTR sw = stemp.c_str();	 
	LPCSTR sw = stemp.c_str();
	commHandle = CreateFile(sw, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

	if (commHandle == INVALID_HANDLE_VALUE)
	{
		CloseHandle(commHandle);
		throw("ERROR: Could not open com port");
	}
	else
	{
		/*COMMTIMEOUTS timeouts;

		timeouts.ReadIntervalTimeout = 1;
		timeouts.ReadTotalTimeoutMultiplier = 1;
		timeouts.ReadTotalTimeoutConstant = 1;
		timeouts.WriteTotalTimeoutMultiplier = 1;
		timeouts.WriteTotalTimeoutConstant = 1;
		if (!SetCommTimeouts(m_hCommPort, &timeouts))*/
		// set timeouts
		COMMTIMEOUTS cto = { MAXDWORD, 0, 0, 0, 0 };
		//COMMTIMEOUTS cto = { 1, 0, 0, 0, 0 };
		//COMMTIMEOUTS cto = { 1, 1, 1, 1, 1 };
		//COMMTIMEOUTS cto = { 0, 0, 0, 1, 0 };
		DCB dcb;
		if (!SetCommTimeouts(commHandle, &cto))
		{
			Serial::~Serial();
			throw("ERROR: Could not set com port time-outs");
		}

		// set DCB
		printf("size : %d\n", sizeof(dcb));
		memset(&dcb, 0, sizeof(dcb));
		dcb.DCBlength = sizeof(dcb);
		dcb.BaudRate = bitRate;
		dcb.fBinary = 1;	  //binary;//


		dcb.Parity = NOPARITY;	 //parity;//
		dcb.StopBits = ONESTOPBIT;	   //stopBits;//
		dcb.ByteSize = 8;		// dataBits;// 
		dcb.fOutX = flowControl;
		dcb.fInX = flowControl;
		//dcb.fRtsControl = true;
		//dcb.fDtrControl = true;
		if (flowControl == RTS_CONTROL_ENABLE)
		{
			dcb.fDtrControl = DTR_CONTROL_ENABLE;
			dcb.fRtsControl = RTS_CONTROL_ENABLE;
		}
		else
		{
			dcb.fDtrControl = DTR_CONTROL_DISABLE;
			dcb.fRtsControl = RTS_CONTROL_DISABLE;
		}

		if (!SetCommState(commHandle, &dcb))
		{
			Serial::~Serial();
			throw("ERROR: Could not set com port parameters");
		}
	}
}

Serial::~Serial()
{
	CloseHandle(commHandle);
}

int Serial::write(const char *buffer)
{
	DWORD numWritten;
	WriteFile(commHandle, buffer, strlen(buffer), &numWritten, NULL);

	return numWritten;
}

int Serial::write(const char *buffer, int buffLen)
{
	DWORD numWritten;
	if (!WriteFile(commHandle, buffer, buffLen, &numWritten, NULL))
	{

		//Get various information about the connection
		COMSTAT status;
		//Keep track of last error
		DWORD errors;

		DWORD dw = GetLastError();
		//ClearCommError(commHandle, &errors, &status);
		printf("\nerror Code:%ld,%ld\n", dw, errors);
		int x = 11;
		int y = x + 1;
	}
	//FlushFileBuffers(commHandle);
	LPDWORD erros;
	LPCOMSTAT	status;
	//if(ClearCommError(commHandle, erros, status))
	//{
	//	int err = 5;
	//	int y = err;
	//}
	//printf("written:%ld", numWritten);
	return numWritten;
}

int Serial::read(char *buffer, int buffLen, bool nullTerminate)
{
	DWORD numRead;
	if (nullTerminate)
	{
		--buffLen;
	}

	BOOL ret = ReadFile(commHandle, buffer, buffLen, &numRead, NULL);
	printf("readed: %ld\n", numRead);
	if (!ret)
	{
		return 0;
	}

	if (nullTerminate)
	{
		buffer[numRead] = '\0';
	}
	printf("str:%s\n", buffer);
	return numRead;
}

#define FLUSH_BUFFSIZE 10

void Serial::flush()
{
	char buffer[FLUSH_BUFFSIZE];
	int numBytes = read(buffer, FLUSH_BUFFSIZE, false);
	while (numBytes != 0)
	{
		numBytes = read(buffer, FLUSH_BUFFSIZE, false);
	}
}
