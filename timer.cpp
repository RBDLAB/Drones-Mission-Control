// timer.cpp : Defines the entry point for the DLL application.
//


#include "timer.h"
#include <windows.h>




using namespace stoper;

// This is the constructor of a class that has been exported.
// see timer.h for the class definition
CTimer::CTimer()
{
	QueryPerformanceFrequency(&m_Freq); //<<--- seconds
	m_FreqD = 1000.0 / (double)m_Freq.QuadPart; //1/x;   //<<---- miliseconds
	return;
}
void CTimer::start()
{
	QueryPerformanceCounter(&m_beginTime);
}
double CTimer::getTimeAndStop()
{
	double temp = getTime();
	m_beginTime.QuadPart = 0;
	return temp;
}
_LARGE_INTEGER CTimer::getBeginTime()
{
	QueryPerformanceCounter(&m_beginTime);
	return m_beginTime;
}
double CTimer::getTime()
{
	if (m_beginTime.QuadPart == 0)
	{
		return 0.0; //it's stoped
	}
	_LARGE_INTEGER endData;
	QueryPerformanceCounter(&endData);
	return (endData.QuadPart - m_beginTime.QuadPart) * m_FreqD;
}
double CTimer::getTimeAndRelease()
{
	double temp = getTime();
	start();
	return temp;

}
double CTimer::getTimeFrom(_LARGE_INTEGER beginTime)
{
	_LARGE_INTEGER endData;
	QueryPerformanceCounter(&endData);
	return (endData.QuadPart - beginTime.QuadPart) * m_FreqD;
}

