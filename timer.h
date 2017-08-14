
// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the TIMER_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// TIMER_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.
#pragma once

#include <windows.h>
namespace stoper {
	// This class is exported from the timer.dll
	class  CTimer {
	public:
		CTimer(void);
		void start();
		double getTimeAndStop();
		_LARGE_INTEGER getBeginTime();
		double getTime();
		double getTimeAndRelease();
		double getTimeFrom(_LARGE_INTEGER beginTime);
		// TODO: add your methods here.
	private:
		_LARGE_INTEGER m_Freq;
		double m_FreqD;
		_LARGE_INTEGER m_beginTime;
	};

};

